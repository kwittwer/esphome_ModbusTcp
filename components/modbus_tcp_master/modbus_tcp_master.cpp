#include "modbus_tcp_master.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome::modbus_tcp_master {

static const char *const TAG = "modbus_tcp_master";

void ModbusTcpMaster::dump_config() {
  ESP_LOGCONFIG(TAG, "Modbus TCP Master:");
  ESP_LOGCONFIG(TAG, "  Host: %s", this->host_.c_str());
  ESP_LOGCONFIG(TAG, "  Port: %u", this->port_);
  ESP_LOGCONFIG(TAG, "  Unit ID: %u", this->unit_id_);
  ESP_LOGCONFIG(TAG, "  Timeout: %u ms", this->timeout_ms_);
  LOG_UPDATE_INTERVAL(this);
  ESP_LOGCONFIG(TAG, "  Register sensors: %zu", this->sensors_.size());
}

bool ModbusTcpMaster::ensure_connected_() {
  if (this->client_.connected()) {
    return true;
  }

  this->client_.stop();
  this->client_.setTimeout(this->timeout_ms_);
  ESP_LOGI(TAG, "Connecting to %s:%u", this->host_.c_str(), this->port_);
  if (!this->client_.connect(this->host_.c_str(), this->port_)) {
    ESP_LOGW(TAG, "Connection to %s:%u failed", this->host_.c_str(), this->port_);
    return false;
  }

  ESP_LOGI(TAG, "Connected to %s:%u", this->host_.c_str(), this->port_);
  return true;
}

bool ModbusTcpMaster::read_exact_(uint8_t *buffer, size_t length) {
  size_t received = this->client_.readBytes(buffer, length);
  if (received != length) {
    ESP_LOGW(TAG, "Incomplete TCP response (%u/%u bytes)", static_cast<unsigned>(received),
             static_cast<unsigned>(length));
    this->client_.stop();
    return false;
  }
  return true;
}

bool ModbusTcpMaster::read_registers_(ModbusTcpSensor *item, std::vector<uint8_t> &payload) {
  if (!this->ensure_connected_()) {
    return false;
  }

  const uint16_t transaction_id = ++this->transaction_id_;
  const uint8_t function =
      item->register_type_ == modbus::ModbusRegisterType::READ ? modbus::ModbusFunctionCode::READ_INPUT_REGISTERS
                                                               : modbus::ModbusFunctionCode::READ_HOLDING_REGISTERS;

  uint8_t request[12] = {
      static_cast<uint8_t>(transaction_id >> 8),
      static_cast<uint8_t>(transaction_id & 0xFF),
      0x00,
      0x00,
      0x00,
      0x06,
      this->unit_id_,
      function,
      static_cast<uint8_t>(item->address_ >> 8),
      static_cast<uint8_t>(item->address_ & 0xFF),
      static_cast<uint8_t>(item->register_count_ >> 8),
      static_cast<uint8_t>(item->register_count_ & 0xFF),
  };

  if (this->client_.write(request, sizeof(request)) != sizeof(request)) {
    ESP_LOGW(TAG, "Failed to send request to %s:%u", this->host_.c_str(), this->port_);
    this->client_.stop();
    return false;
  }
  this->client_.flush();

  uint8_t header[7];
  if (!this->read_exact_(header, sizeof(header))) {
    return false;
  }

  const uint16_t response_transaction = (uint16_t(header[0]) << 8) | header[1];
  const uint16_t protocol_id = (uint16_t(header[2]) << 8) | header[3];
  const uint16_t length = (uint16_t(header[4]) << 8) | header[5];
  const uint8_t unit_id = header[6];

  if (response_transaction != transaction_id || protocol_id != 0 || unit_id != this->unit_id_ || length < 2) {
    ESP_LOGW(TAG, "Invalid Modbus TCP header received");
    this->client_.stop();
    return false;
  }

  std::vector<uint8_t> pdu(length - 1, 0);
  if (!this->read_exact_(pdu.data(), pdu.size())) {
    return false;
  }

  if ((pdu[0] & 0x80U) != 0U) {
    ESP_LOGW(TAG, "Device returned Modbus exception 0x%02X", pdu.size() > 1 ? pdu[1] : 0);
    return false;
  }

  if (pdu[0] != function || pdu.size() < 2) {
    ESP_LOGW(TAG, "Unexpected function code in response");
    return false;
  }

  const uint8_t byte_count = pdu[1];
  if (byte_count + 2 != pdu.size()) {
    ESP_LOGW(TAG, "Unexpected payload length in response");
    return false;
  }

  payload.assign(pdu.begin() + 2, pdu.end());
  return true;
}

void ModbusTcpMaster::update() {
  if (this->sensors_.empty()) {
    return;
  }

  bool ok = true;
  for (auto *item : this->sensors_) {
    std::vector<uint8_t> payload;
    if (!this->read_registers_(item, payload)) {
      ok = false;
      continue;
    }

    const int64_t raw = modbus::helpers::payload_to_number(payload, item->value_type_, 0, 0xFFFFFFFF);
    float value = modbus::helpers::value_type_is_float(item->value_type_)
                      ? bit_cast<float>(static_cast<uint32_t>(raw))
                      : static_cast<float>(raw);
    item->publish_state(value * item->scale_ + item->offset_);
  }

  if (ok) {
    this->status_clear_warning();
  } else {
    this->status_set_warning();
  }
}

}  // namespace esphome::modbus_tcp_master
