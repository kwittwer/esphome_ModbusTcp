#include "modbus_tcp_slave.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome::modbus_tcp_slave {

static const char *const TAG = "modbus_tcp_slave";

void ModbusTcpSlave::setup() {
  this->server_ = std::make_unique<WiFiServer>(this->port_);
  this->server_->begin();
  this->server_->setNoDelay(true);
}

void ModbusTcpSlave::dump_config() {
  ESP_LOGCONFIG(TAG, "Modbus TCP Slave:");
  ESP_LOGCONFIG(TAG, "  Port: %u", this->port_);
  ESP_LOGCONFIG(TAG, "  Unit ID: %u", this->unit_id_);
  ESP_LOGCONFIG(TAG, "  Register definitions: %zu", this->server_registers_.size());
  ESP_LOGCONFIG(TAG, "  Courtesy response: %s", YESNO(this->server_courtesy_response_.enabled));
}

void ModbusTcpSlave::loop() {
  if (!this->client_ || !this->client_.connected()) {
    if (this->client_) {
      this->client_.stop();
    }
    auto next_client = this->server_->available();
    if (next_client) {
      ESP_LOGI(TAG, "Client connected");
      this->client_ = next_client;
      this->client_.setTimeout(50);
      this->rx_buffer_.clear();
    } else {
      return;
    }
  }

  while (this->client_.available() > 0) {
    this->rx_buffer_.push_back(this->client_.read());
  }

  if (!this->rx_buffer_.empty()) {
    this->process_buffer_();
  }
}

void ModbusTcpSlave::process_buffer_() {
  while (this->rx_buffer_.size() >= 7) {
    const uint16_t protocol_id = (uint16_t(this->rx_buffer_[2]) << 8) | this->rx_buffer_[3];
    const uint16_t length = (uint16_t(this->rx_buffer_[4]) << 8) | this->rx_buffer_[5];
    const size_t frame_size = 6 + length;

    if (protocol_id != 0) {
      ESP_LOGW(TAG, "Ignoring frame with invalid protocol id");
      this->rx_buffer_.clear();
      return;
    }

    if (this->rx_buffer_.size() < frame_size) {
      return;
    }

    const uint16_t transaction_id = (uint16_t(this->rx_buffer_[0]) << 8) | this->rx_buffer_[1];
    std::vector<uint8_t> frame(this->rx_buffer_.begin() + 6, this->rx_buffer_.begin() + frame_size);
    this->rx_buffer_.erase(this->rx_buffer_.begin(), this->rx_buffer_.begin() + frame_size);
    this->handle_request_(transaction_id, frame);
  }
}

void ModbusTcpSlave::handle_request_(uint16_t transaction_id, const std::vector<uint8_t> &frame) {
  if (frame.size() < 2) {
    return;
  }

  const uint8_t unit_id = frame[0];
  const uint8_t function_code = frame[1];
  if (unit_id != this->unit_id_) {
    return;
  }

  if ((function_code == modbus::ModbusFunctionCode::READ_HOLDING_REGISTERS ||
       function_code == modbus::ModbusFunctionCode::READ_INPUT_REGISTERS) &&
      frame.size() >= 6) {
    const uint16_t start_address = (uint16_t(frame[2]) << 8) | frame[3];
    const uint16_t count = (uint16_t(frame[4]) << 8) | frame[5];
    std::vector<uint16_t> words;
    if (!this->fill_read_response_(start_address, count, words)) {
      this->send_exception_(transaction_id, function_code, modbus::ModbusExceptionCode::ILLEGAL_DATA_ADDRESS);
      return;
    }

    std::vector<uint8_t> pdu;
    pdu.reserve(2 + words.size() * 2);
    pdu.push_back(function_code);
    pdu.push_back(static_cast<uint8_t>(words.size() * 2));
    for (auto word : words) {
      pdu.push_back(static_cast<uint8_t>(word >> 8));
      pdu.push_back(static_cast<uint8_t>(word & 0xFF));
    }
    this->send_response_(transaction_id, pdu);
    return;
  }

  if (function_code == modbus::ModbusFunctionCode::WRITE_SINGLE_REGISTER && frame.size() >= 6) {
    const uint16_t address = (uint16_t(frame[2]) << 8) | frame[3];
    const uint16_t value = (uint16_t(frame[4]) << 8) | frame[5];
    if (!this->write_single_register_(address, value)) {
      this->send_exception_(transaction_id, function_code, modbus::ModbusExceptionCode::ILLEGAL_DATA_ADDRESS);
      return;
    }

    std::vector<uint8_t> pdu(frame.begin() + 1, frame.begin() + 6);
    this->send_response_(transaction_id, pdu);
    return;
  }

  if (function_code == modbus::ModbusFunctionCode::WRITE_MULTIPLE_REGISTERS && frame.size() >= 7) {
    const uint16_t address = (uint16_t(frame[2]) << 8) | frame[3];
    const uint16_t count = (uint16_t(frame[4]) << 8) | frame[5];
    const uint8_t byte_count = frame[6];
    if (frame.size() < static_cast<size_t>(7 + byte_count)) {
      this->send_exception_(transaction_id, function_code, modbus::ModbusExceptionCode::ILLEGAL_DATA_VALUE);
      return;
    }
    std::vector<uint8_t> payload(frame.begin() + 7, frame.begin() + 7 + byte_count);
    if (!this->write_multiple_registers_(address, count, payload)) {
      this->send_exception_(transaction_id, function_code, modbus::ModbusExceptionCode::ILLEGAL_DATA_ADDRESS);
      return;
    }

    std::vector<uint8_t> pdu{function_code, frame[2], frame[3], frame[4], frame[5]};
    this->send_response_(transaction_id, pdu);
    return;
  }

  this->send_exception_(transaction_id, function_code, modbus::ModbusExceptionCode::ILLEGAL_FUNCTION);
}

void ModbusTcpSlave::send_exception_(uint16_t transaction_id, uint8_t function_code,
                                     modbus::ModbusExceptionCode exception_code) {
  this->send_response_(transaction_id, {static_cast<uint8_t>(function_code | 0x80U), static_cast<uint8_t>(exception_code)});
}

void ModbusTcpSlave::send_response_(uint16_t transaction_id, const std::vector<uint8_t> &pdu) {
  if (!this->client_ || !this->client_.connected()) {
    return;
  }

  const uint16_t length = 1 + pdu.size();
  std::vector<uint8_t> frame;
  frame.reserve(7 + pdu.size());
  frame.push_back(static_cast<uint8_t>(transaction_id >> 8));
  frame.push_back(static_cast<uint8_t>(transaction_id & 0xFF));
  frame.push_back(0x00);
  frame.push_back(0x00);
  frame.push_back(static_cast<uint8_t>(length >> 8));
  frame.push_back(static_cast<uint8_t>(length & 0xFF));
  frame.push_back(this->unit_id_);
  frame.insert(frame.end(), pdu.begin(), pdu.end());
  this->client_.write(frame.data(), frame.size());
}

ServerRegister *ModbusTcpSlave::find_register_(uint16_t address) const {
  for (auto *server_register : this->server_registers_) {
    if (server_register->address == address) {
      return server_register;
    }
  }
  return nullptr;
}

bool ModbusTcpSlave::fill_read_response_(uint16_t start_address, uint16_t count, std::vector<uint16_t> &words) const {
  words.clear();
  words.reserve(count);
  uint32_t current = start_address;
  const uint32_t end = start_address + count;

  while (current < end) {
    auto *server_register = this->find_register_(current);
    if (server_register != nullptr) {
      if (current + server_register->register_count > end || !server_register->read_lambda) {
        return false;
      }
      std::vector<uint16_t> payload;
      modbus::helpers::number_to_payload(payload, server_register->read_lambda(), server_register->value_type);
      words.insert(words.end(), payload.begin(), payload.end());
      current += server_register->register_count;
      continue;
    }

    if (this->server_courtesy_response_.enabled && current <= this->server_courtesy_response_.register_last_address) {
      words.push_back(this->server_courtesy_response_.register_value);
      current++;
      continue;
    }

    return false;
  }

  return true;
}

bool ModbusTcpSlave::write_single_register_(uint16_t address, uint16_t value) {
  auto *server_register = this->find_register_(address);
  if (server_register == nullptr || server_register->register_count != 1 || !server_register->write_lambda) {
    return false;
  }

  std::vector<uint8_t> payload{static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)};
  return server_register->write_lambda(
      modbus::helpers::payload_to_number(payload, server_register->value_type, 0, 0xFFFFFFFF));
}

bool ModbusTcpSlave::write_multiple_registers_(uint16_t address, uint16_t count, const std::vector<uint8_t> &payload) {
  auto *server_register = this->find_register_(address);
  if (server_register == nullptr || server_register->register_count != count || !server_register->write_lambda) {
    return false;
  }
  return server_register->write_lambda(
      modbus::helpers::payload_to_number(payload, server_register->value_type, 0, 0xFFFFFFFF));
}

}  // namespace esphome::modbus_tcp_slave
