#include "modbus_tcp_slave.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <fcntl.h>
#include <cstring>
#include <cerrno>

namespace esphome::modbus_tcp_slave {

static const char *const TAG = "modbus_tcp_slave";
static constexpr uint8_t FC_READ_COILS = 0x01;
static constexpr uint8_t FC_READ_DISCRETE_INPUTS = 0x02;
static constexpr uint8_t FC_READ_HOLDING_REGISTERS = 0x03;
static constexpr uint8_t FC_READ_INPUT_REGISTERS = 0x04;
static constexpr uint8_t FC_WRITE_SINGLE_COIL = 0x05;
static constexpr uint8_t FC_WRITE_SINGLE_REGISTER = 0x06;
static constexpr uint8_t FC_WRITE_MULTIPLE_COILS = 0x0F;
static constexpr uint8_t FC_WRITE_MULTIPLE_REGISTERS = 0x10;
static constexpr size_t BITS_PER_BYTE = 8U;

static inline size_t required_bytes_for_bits_(uint16_t count) {
  return (static_cast<size_t>(count) + (BITS_PER_BYTE - 1U)) / BITS_PER_BYTE;
}

static void number_to_payload_(std::vector<uint16_t> &data, int64_t value, SensorValueType value_type) {
  switch (value_type) {
    case SensorValueType::U_WORD:
    case SensorValueType::S_WORD:
      data.push_back(value & 0xFFFF);
      break;
    case SensorValueType::U_DWORD:
    case SensorValueType::S_DWORD:
    case SensorValueType::FP32:
      data.push_back((value & 0xFFFF0000) >> 16);
      data.push_back(value & 0xFFFF);
      break;
    case SensorValueType::U_DWORD_R:
    case SensorValueType::S_DWORD_R:
    case SensorValueType::FP32_R:
      data.push_back(value & 0xFFFF);
      data.push_back((value & 0xFFFF0000) >> 16);
      break;
    case SensorValueType::U_QWORD:
    case SensorValueType::S_QWORD:
      data.push_back((value & 0xFFFF000000000000) >> 48);
      data.push_back((value & 0xFFFF00000000) >> 32);
      data.push_back((value & 0xFFFF0000) >> 16);
      data.push_back(value & 0xFFFF);
      break;
    case SensorValueType::U_QWORD_R:
    case SensorValueType::S_QWORD_R:
      data.push_back(value & 0xFFFF);
      data.push_back((value & 0xFFFF0000) >> 16);
      data.push_back((value & 0xFFFF00000000) >> 32);
      data.push_back((value & 0xFFFF000000000000) >> 48);
      break;
    case SensorValueType::RAW:
    default:
      break;
  }
}

static int64_t payload_to_number_(const std::vector<uint8_t> &data, SensorValueType value_type) {
  switch (value_type) {
    case SensorValueType::U_WORD:
      return modbus::helpers::get_data<uint16_t>(data, 0);
    case SensorValueType::U_DWORD:
    case SensorValueType::FP32:
      return modbus::helpers::get_data<uint32_t>(data, 0);
    case SensorValueType::U_DWORD_R:
    case SensorValueType::FP32_R: {
      auto value = modbus::helpers::get_data<uint32_t>(data, 0);
      return static_cast<uint32_t>(value & 0xFFFF) << 16 | (value & 0xFFFF0000) >> 16;
    }
    case SensorValueType::S_WORD:
      return modbus::helpers::get_data<int16_t>(data, 0);
    case SensorValueType::S_DWORD:
      return modbus::helpers::get_data<int32_t>(data, 0);
    case SensorValueType::S_DWORD_R: {
      auto value = modbus::helpers::get_data<uint32_t>(data, 0);
      uint32_t sign_bit = (value & 0x8000) << 16;
      return static_cast<int32_t>(((value & 0x7FFF) << 16 | (value & 0xFFFF0000) >> 16) | sign_bit);
    }
    case SensorValueType::U_QWORD:
    case SensorValueType::S_QWORD:
      return modbus::helpers::get_data<uint64_t>(data, 0);
    case SensorValueType::U_QWORD_R:
    case SensorValueType::S_QWORD_R: {
      auto value = modbus::helpers::get_data<uint64_t>(data, 0);
      return (value << 48) | (value >> 48) | ((value & 0xFFFF0000) << 16) | ((value >> 16) & 0xFFFF0000);
    }
    case SensorValueType::RAW:
    default:
      return 0;
  }
}

void ModbusTcpSlave::setup() {
  // Create server socket
  this->server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
  if (this->server_socket_ < 0) {
    ESP_LOGE(TAG, "Failed to create server socket");
    return;
  }

  // Set socket options
  int reuse = 1;
  setsockopt(this->server_socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  // Bind socket
  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(this->port_);

  if (bind(this->server_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    ESP_LOGE(TAG, "Failed to bind server socket to port %u", this->port_);
    close(this->server_socket_);
    this->server_socket_ = -1;
    return;
  }

  // Listen
  if (listen(this->server_socket_, 1) < 0) {
    ESP_LOGE(TAG, "Failed to listen on server socket");
    close(this->server_socket_);
    this->server_socket_ = -1;
    return;
  }

  // Set socket to non-blocking
  int flags = fcntl(this->server_socket_, F_GETFL, 0);
  fcntl(this->server_socket_, F_SETFL, flags | O_NONBLOCK);

  ESP_LOGI(TAG, "Modbus TCP Server listening on port %u", this->port_);
}

void ModbusTcpSlave::dump_config() {
  ESP_LOGCONFIG(TAG, "Modbus TCP Slave:");
  ESP_LOGCONFIG(TAG, "  Port: %u", this->port_);
  ESP_LOGCONFIG(TAG, "  Unit ID: %u", this->unit_id_);
  ESP_LOGCONFIG(TAG, "  Register definitions: %zu", this->server_registers_.size());
  ESP_LOGCONFIG(TAG, "  Courtesy response: %s", YESNO(this->server_courtesy_response_.enabled));
}

void ModbusTcpSlave::loop() {
  // Try to accept new connection
  if (this->client_socket_ < 0 && this->server_socket_ >= 0) {
    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    int new_socket = accept(this->server_socket_, (struct sockaddr *)&client_addr, &client_addr_len);
    if (new_socket >= 0) {
      this->client_socket_ = new_socket;
      ESP_LOGI(TAG, "Client connected");
      this->rx_buffer_.clear();
    }
  }

  // Read from client
  if (this->client_socket_ >= 0) {
    uint8_t buffer[256];
    ssize_t n = recv(this->client_socket_, buffer, sizeof(buffer), MSG_DONTWAIT);
    if (n > 0) {
      for (ssize_t i = 0; i < n; i++) {
        this->rx_buffer_.push_back(buffer[i]);
      }
      this->process_buffer_();
    } else if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
      close(this->client_socket_);
      this->client_socket_ = -1;
      ESP_LOGI(TAG, "Client disconnected");
    }
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

  if ((function_code == FC_READ_COILS || function_code == FC_READ_DISCRETE_INPUTS) && frame.size() >= 6) {
    const uint16_t start_address = (uint16_t(frame[2]) << 8) | frame[3];
    const uint16_t count = (uint16_t(frame[4]) << 8) | frame[5];
    if (count == 0) {
      this->send_exception_(transaction_id, function_code, modbus::ModbusExceptionCode::ILLEGAL_DATA_VALUE);
      return;
    }

    std::vector<uint8_t> bits;
    if (!this->fill_bit_response_(start_address, count, bits)) {
      this->send_exception_(transaction_id, function_code, modbus::ModbusExceptionCode::ILLEGAL_DATA_ADDRESS);
      return;
    }

    std::vector<uint8_t> pdu;
    pdu.reserve(2 + bits.size());
    pdu.push_back(function_code);
    pdu.push_back(static_cast<uint8_t>(bits.size()));
    pdu.insert(pdu.end(), bits.begin(), bits.end());
    this->send_response_(transaction_id, pdu);
    return;
  }

  if ((function_code == FC_READ_HOLDING_REGISTERS || function_code == FC_READ_INPUT_REGISTERS) && frame.size() >= 6) {
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

  if (function_code == FC_WRITE_SINGLE_COIL && frame.size() >= 6) {
    const uint16_t address = (uint16_t(frame[2]) << 8) | frame[3];
    const uint16_t value = (uint16_t(frame[4]) << 8) | frame[5];
    if (value != 0x0000 && value != 0xFF00) {
      this->send_exception_(transaction_id, function_code, modbus::ModbusExceptionCode::ILLEGAL_DATA_VALUE);
      return;
    }
    if (!this->write_single_coil_(address, value == 0xFF00)) {
      this->send_exception_(transaction_id, function_code, modbus::ModbusExceptionCode::ILLEGAL_DATA_ADDRESS);
      return;
    }

    std::vector<uint8_t> pdu(frame.begin() + 1, frame.begin() + 6);
    this->send_response_(transaction_id, pdu);
    return;
  }

  if (function_code == FC_WRITE_SINGLE_REGISTER && frame.size() >= 6) {
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

  if (function_code == FC_WRITE_MULTIPLE_COILS && frame.size() >= 7) {
    const uint16_t address = (uint16_t(frame[2]) << 8) | frame[3];
    const uint16_t count = (uint16_t(frame[4]) << 8) | frame[5];
    const uint8_t byte_count = frame[6];
    const size_t expected_byte_count = required_bytes_for_bits_(count);
    if (count == 0 || byte_count != expected_byte_count || frame.size() < static_cast<size_t>(7 + byte_count)) {
      this->send_exception_(transaction_id, function_code, modbus::ModbusExceptionCode::ILLEGAL_DATA_VALUE);
      return;
    }
    std::vector<uint8_t> payload(frame.begin() + 7, frame.begin() + 7 + byte_count);
    if (!this->write_multiple_coils_(address, count, payload)) {
      this->send_exception_(transaction_id, function_code, modbus::ModbusExceptionCode::ILLEGAL_DATA_ADDRESS);
      return;
    }

    std::vector<uint8_t> pdu{function_code, frame[2], frame[3], frame[4], frame[5]};
    this->send_response_(transaction_id, pdu);
    return;
  }

  if (function_code == FC_WRITE_MULTIPLE_REGISTERS && frame.size() >= 7) {
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
  if (this->client_socket_ < 0) {
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
  send(this->client_socket_, frame.data(), frame.size(), 0);
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
      number_to_payload_(payload, server_register->read_lambda(), server_register->value_type);
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

bool ModbusTcpSlave::fill_bit_response_(uint16_t start_address, uint16_t count, std::vector<uint8_t> &bits) const {
  bits.assign(required_bytes_for_bits_(count), 0);
  for (uint16_t offset = 0; offset < count; offset++) {
    auto *server_register = this->find_register_(start_address + offset);
    if (server_register == nullptr || server_register->register_count != 1 || !server_register->read_lambda) {
      return false;
    }
    if (server_register->read_lambda() != 0) {
      bits[offset / BITS_PER_BYTE] |= static_cast<uint8_t>(1U << (offset % BITS_PER_BYTE));
    }
  }
  return true;
}

bool ModbusTcpSlave::write_single_coil_(uint16_t address, bool value) {
  auto *server_register = this->find_register_(address);
  if (server_register == nullptr || server_register->register_count != 1 || !server_register->write_lambda) {
    return false;
  }
  return server_register->write_lambda(value ? 1 : 0);
}

bool ModbusTcpSlave::write_multiple_coils_(uint16_t address, uint16_t count, const std::vector<uint8_t> &payload) {
  std::vector<ServerRegister *> registers;
  registers.reserve(count);
  for (uint16_t offset = 0; offset < count; offset++) {
    auto *server_register = this->find_register_(address + offset);
    if (server_register == nullptr || server_register->register_count != 1 || !server_register->write_lambda) {
      return false;
    }
    registers.push_back(server_register);
  }

  for (uint16_t offset = 0; offset < count; offset++) {
    const bool value = (payload[offset / BITS_PER_BYTE] & (1U << (offset % BITS_PER_BYTE))) != 0;
    if (!registers[offset]->write_lambda(value ? 1 : 0)) {
      return false;
    }
  }
  return true;
}

bool ModbusTcpSlave::write_single_register_(uint16_t address, uint16_t value) {
  auto *server_register = this->find_register_(address);
  if (server_register == nullptr || server_register->register_count != 1 || !server_register->write_lambda) {
    return false;
  }

  std::vector<uint8_t> payload{static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)};
  return server_register->write_lambda(payload_to_number_(payload, server_register->value_type));
}

bool ModbusTcpSlave::write_multiple_registers_(uint16_t address, uint16_t count, const std::vector<uint8_t> &payload) {
  auto *server_register = this->find_register_(address);
  if (server_register == nullptr || server_register->register_count != count || !server_register->write_lambda) {
    return false;
  }
  return server_register->write_lambda(payload_to_number_(payload, server_register->value_type));
}

}  // namespace esphome::modbus_tcp_slave
