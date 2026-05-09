#pragma once

#include "esphome/components/modbus/modbus_definitions.h"
#include "esphome/components/modbus/modbus_helpers.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

#include <string>
#include <vector>

#ifdef USE_ESP32
#include <WiFi.h>
#elif defined(USE_ESP8266)
#include <ESP8266WiFi.h>
#endif

namespace esphome::modbus_tcp_master {

class ModbusTcpSensor : public sensor::Sensor {
 public:
  void set_address(uint16_t address) { this->address_ = address; }
  void set_function_code(uint8_t function_code) { this->function_code_ = function_code; }
  void set_value_type(modbus::helpers::SensorValueType value_type) { this->value_type_ = value_type; }
  void set_register_count(uint8_t register_count) { this->register_count_ = register_count; }
  void set_scale(float scale) { this->scale_ = scale; }
  void set_offset(float offset) { this->offset_ = offset; }

 protected:
  friend class ModbusTcpMaster;

  uint16_t address_{0};
  uint8_t function_code_{0x03};
  modbus::helpers::SensorValueType value_type_{modbus::helpers::SensorValueType::U_WORD};
  uint8_t register_count_{1};
  float scale_{1.0f};
  float offset_{0.0f};
};

class ModbusTcpMaster : public PollingComponent {
 public:
  void dump_config() override;
  void update() override;

  void set_host(const std::string &host) { this->host_ = host; }
  void set_port(uint16_t port) { this->port_ = port; }
  void set_unit_id(uint8_t unit_id) { this->unit_id_ = unit_id; }
  void set_timeout(uint32_t timeout_ms) { this->timeout_ms_ = timeout_ms; }
  void add_sensor(ModbusTcpSensor *sensor) { this->sensors_.push_back(sensor); }

 protected:
  bool ensure_connected_();
  bool read_registers_(ModbusTcpSensor *item, std::vector<uint8_t> &payload);
  bool read_exact_(uint8_t *buffer, size_t length);

  std::string host_;
  uint16_t port_{502};
  uint8_t unit_id_{1};
  uint32_t timeout_ms_{5000};
  uint16_t transaction_id_{0};
  WiFiClient client_;
  std::vector<ModbusTcpSensor *> sensors_;
};

}  // namespace esphome::modbus_tcp_master
