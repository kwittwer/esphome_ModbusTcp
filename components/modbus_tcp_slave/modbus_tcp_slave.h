#pragma once

#include "esphome/components/modbus/modbus_definitions.h"
#include "esphome/components/modbus/modbus_helpers.h"
#include "esphome/core/component.h"

#include <functional>
#include <memory>
#include <type_traits>
#include <vector>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <cstring>

namespace esphome::modbus_tcp_slave {

using modbus::helpers::SensorValueType;

struct ServerCourtesyResponse {
  bool enabled{false};
  uint16_t register_last_address{0xFFFF};
  uint16_t register_value{0};
};

class ServerRegister {
  using ReadLambda = std::function<int64_t()>;
  using WriteLambda = std::function<bool(int64_t value)>;

 public:
  ServerRegister(uint16_t address, SensorValueType value_type, uint8_t register_count)
      : address(address), value_type(value_type), register_count(register_count) {}

  template<typename T> void set_read_lambda(const std::function<T(uint16_t address)> &user_read_lambda) {
    this->read_lambda = [this, user_read_lambda]() -> int64_t {
      T user_value = user_read_lambda(this->address);
      if constexpr (std::is_same_v<T, float>) {
        return bit_cast<uint32_t>(user_value);
      }
      return static_cast<int64_t>(user_value);
    };
  }

  template<typename T> void set_write_lambda(const std::function<bool(uint16_t address, const T value)> &user_write_lambda) {
    this->write_lambda = [this, user_write_lambda](int64_t number) {
      if constexpr (std::is_same_v<T, float>) {
        return user_write_lambda(this->address, bit_cast<float>(static_cast<uint32_t>(number)));
      }
      return user_write_lambda(this->address, static_cast<T>(number));
    };
  }

  uint16_t address{0};
  SensorValueType value_type{SensorValueType::RAW};
  uint8_t register_count{0};
  ReadLambda read_lambda;
  WriteLambda write_lambda;
};

class ModbusTcpSlave : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_port(uint16_t port) { this->port_ = port; }
  void set_unit_id(uint8_t unit_id) { this->unit_id_ = unit_id; }
  void add_server_register(ServerRegister *server_register) { this->server_registers_.push_back(server_register); }
  void set_server_courtesy_response(const ServerCourtesyResponse &server_courtesy_response) {
    this->server_courtesy_response_ = server_courtesy_response;
  }

 protected:
  void process_buffer_();
  void handle_request_(uint16_t transaction_id, const std::vector<uint8_t> &frame);
  void send_exception_(uint16_t transaction_id, uint8_t function_code, modbus::ModbusExceptionCode exception_code);
  void send_response_(uint16_t transaction_id, const std::vector<uint8_t> &pdu);
  ServerRegister *find_register_(uint16_t address) const;
  bool fill_read_response_(uint16_t start_address, uint16_t count, std::vector<uint16_t> &words) const;
  bool fill_bit_response_(uint16_t start_address, uint16_t count, std::vector<uint8_t> &bits) const;
  bool write_single_coil_(uint16_t address, bool value);
  bool write_multiple_coils_(uint16_t address, uint16_t count, const std::vector<uint8_t> &payload);
  bool write_single_register_(uint16_t address, uint16_t value);
  bool write_multiple_registers_(uint16_t address, uint16_t count, const std::vector<uint8_t> &payload);

  uint16_t port_{502};
  uint8_t unit_id_{1};
  int server_socket_{-1};
  int client_socket_{-1};
  std::vector<uint8_t> rx_buffer_;
  std::vector<ServerRegister *> server_registers_;
  ServerCourtesyResponse server_courtesy_response_{};
};

}  // namespace esphome::modbus_tcp_slave
