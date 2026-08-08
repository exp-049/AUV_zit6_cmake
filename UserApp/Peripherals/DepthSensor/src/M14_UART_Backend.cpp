#include "M14_UART_Backend.hpp"

#include "RosLogger.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace auv {
namespace peripheral {
namespace {

bool parseM14Field(const char *line, const char marker, float &value) {
  if (line == nullptr) {
    return false;
  }

  const char *field = std::strchr(line, marker);
  if (field == nullptr) {
    return false;
  }

  ++field;
  while (*field == ' ' || *field == '\t') {
    ++field;
  }
  if (*field != '=') {
    return false;
  }

  ++field;
  while (*field == ' ' || *field == '\t') {
    ++field;
  }

  char *end = nullptr;
  value = std::strtof(field, &end);
  return end != field && std::isfinite(value);
}

bool parseM14DataLine(const char *line, float &temperature, float &depth) {
  // Do not use sscanf("%f") here. The embedded newlib configuration does not
  // necessarily link floating-point scanf support, while strtof is available
  // in the same firmware configuration. Accept both T=1.0D=2.0 and
  // T = 1.0 D = 2.0, matching the older UART depth backend.
  return parseM14Field(line, 'T', temperature) &&
         parseM14Field(line, 'D', depth);
}

} // namespace

M14_UART_Backend::M14_UART_Backend(UartPortOps ops) : ops_(ops) {
  ROS_LOG_DEBUG("[M14] backend constructed, ops=%p", &ops_);
}

bool M14_UART_Backend::init() {
  line_buffer_[0] = '\0';
  line_length_ = 0U;
  frame_ready_ = false;
  connected_ = false;
  depth_ = 0.0f;
  temperature_ = 0.0f;

  const bool ready = ops_.poll != nullptr && ops_.startRx != nullptr;
  ROS_LOG_DEBUG("[M14] init: transport=%d", ready);
  return ready;
}

void M14_UART_Backend::poll() {
  if (ops_.poll != nullptr) {
    ops_.poll(ops_.ctx);
  }
}

bool M14_UART_Backend::read() {
  if (!frame_ready_) {
    return false;
  }

  frame_ready_ = false;
  if (cb_.onDepthReady != nullptr) {
    cb_.onDepthReady(cb_.ctx, depth_, temperature_);
  }
  return true;
}

void M14_UART_Backend::start() {
  if (ops_.startRx == nullptr) {
    ROS_LOG_DEBUG("[M14] start: startRx hook is null");
    return;
  }
  const bool ok = ops_.startRx(ops_.ctx);
  ROS_LOG_DEBUG("[M14] start RX: %d", ok);
}

void M14_UART_Backend::onRxByte(uint8_t byte) {
  if (byte == '\r' || byte == '\n') {
    if (line_length_ != 0U) {
      finishLine();
    }
    return;
  }

  if (line_length_ >= kLineBufferSize - 1U) {
    line_length_ = 0U;
  }
  line_buffer_[line_length_++] = static_cast<char>(byte);
  line_buffer_[line_length_] = '\0';
}

bool M14_UART_Backend::finishLine() {
  line_buffer_[line_length_] = '\0';

  float temperature = 0.0f;
  float depth = 0.0f;
  const bool valid = parseM14DataLine(line_buffer_, temperature, depth);
  line_length_ = 0U;
  line_buffer_[0] = '\0';
  if (!valid) {
    return false;
  }

  temperature_ = temperature;
  depth_ = depth;
  connected_ = true;
  frame_ready_ = true;
  return true;
}

bool M14_UART_Backend::sendCommand(const char *command) {
  if (command == nullptr || ops_.transmit == nullptr) {
    return false;
  }

  char packet[48] = {};
  const int length = std::snprintf(packet, sizeof(packet), "%s\r\n", command);
  if (length <= 0 || static_cast<size_t>(length) >= sizeof(packet)) {
    return false;
  }
  return ops_.transmit(ops_.ctx, reinterpret_cast<const uint8_t *>(packet),
                       static_cast<uint16_t>(length));
}

bool M14_UART_Backend::setFluidDensity(uint16_t density) {
  if (density > 5000U) {
    return false;
  }
  char command[16] = {};
  std::snprintf(command, sizeof(command), "!F%u", density);
  return sendCommand(command);
}

bool M14_UART_Backend::setDepthOffset(float offset_m) {
  char command[24] = {};
  std::snprintf(command, sizeof(command), "!D%.2f", static_cast<double>(offset_m));
  return sendCommand(command);
}

bool M14_UART_Backend::setTemperatureOffset(float offset_c) {
  char command[24] = {};
  std::snprintf(command, sizeof(command), "!T%.2f", static_cast<double>(offset_c));
  return sendCommand(command);
}

bool M14_UART_Backend::toggleParameterOutput() { return sendCommand("!!"); }

bool M14_UART_Backend::resetSensor() { return sendCommand("!R"); }

bool M14_UART_Backend::restoreFactorySettings() { return sendCommand("!r"); }

bool M14_UART_Backend::clearOffsets() { return sendCommand("!C"); }

} // namespace peripheral
} // namespace auv
