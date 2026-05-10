#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include <cstring>

namespace modbus {

enum class ModbusExceptionCode : uint8_t {
  ILLEGAL_FUNCTION = 0x01,
  ILLEGAL_DATA_ADDRESS = 0x02,
  ILLEGAL_DATA_VALUE = 0x03,
  DEVICE_FAILURE = 0x04,
  ACKNOWLEDGE = 0x05,
  DEVICE_BUSY = 0x06,
  NEGATIVE_ACKNOWLEDGE = 0x07,
  MEMORY_PARITY_ERROR = 0x08,
};

namespace helpers {

enum class SensorValueType {
  U_WORD,
  S_WORD,
  U_DWORD,
  S_DWORD,
  U_DWORD_R,
  S_DWORD_R,
  FP32,
  FP32_R,
  U_QWORD,
  S_QWORD,
  U_QWORD_R,
  S_QWORD_R,
  RAW,
};

inline bool value_type_is_float(SensorValueType type) {
  return type == SensorValueType::FP32 || type == SensorValueType::FP32_R;
}

template <typename T>
T get_data(const std::vector<uint8_t> &data, size_t offset) {
  if (data.size() < offset + sizeof(T)) return 0;
  T value = 0;
  for (size_t i = 0; i < sizeof(T); i++) {
    value = (value << 8) | data[offset + i];
  }
  return value;
}

}  // namespace helpers
}  // namespace modbus
