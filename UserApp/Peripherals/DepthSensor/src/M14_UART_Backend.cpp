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
  line_overflow_ = false;
  frame_ready_ = false;
  connected_ = false;
  depth_ = 0.0f;
  temperature_ = 0.0f;
  rx_byte_count_ = 0U;
  valid_frame_count_ = 0U;
  parser_error_count_ = 0U;
  last_frame_length_ = 0U;
  rx_preview_next_ = 0U;
  rx_preview_count_ = 0U;

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

bool M14_UART_Backend::serviceRxRecovery(bool no_valid_frame_timeout) {
  if (ops_.serviceRxRecovery == nullptr) {
    return false;
  }
  const bool recovered =
      ops_.serviceRxRecovery(ops_.ctx, no_valid_frame_timeout);
  if (recovered) {
    // Do not join text received before and after a UART/DMA restart into one
    // protocol frame. Preserve the last valid sample while dropping only the
    // partial line.
    line_length_ = 0U;
    line_buffer_[0] = '\0';
    line_overflow_ = false;
  }
  return recovered;
}

void M14_UART_Backend::onRxByte(uint8_t byte) {
  ++rx_byte_count_;
  rx_preview_[rx_preview_next_] = byte;
  rx_preview_next_ = static_cast<uint8_t>((rx_preview_next_ + 1U) % 16U);
  if (rx_preview_count_ < 16U) {
    ++rx_preview_count_;
  }

  if (byte == '\r' || byte == '\n') {
    if (!line_overflow_ && line_length_ != 0U) {
      finishLine();
    }
    line_length_ = 0U;
    line_overflow_ = false;
    return;
  }

  if (line_overflow_) {
    return;
  }
  if (line_length_ >= kLineBufferSize - 1U) {
    ++parser_error_count_;
    last_frame_length_ = static_cast<uint8_t>(line_length_);
    line_length_ = 0U;
    line_overflow_ = true;
    return;
  }
  line_buffer_[line_length_++] = static_cast<char>(byte);
  line_buffer_[line_length_] = '\0';
}

bool M14_UART_Backend::finishLine() {
  line_buffer_[line_length_] = '\0';
  last_frame_length_ = static_cast<uint8_t>(line_length_);

  float temperature = 0.0f;
  float depth = 0.0f;
  const bool valid = parseM14DataLine(line_buffer_, temperature, depth);
  line_length_ = 0U;
  line_buffer_[0] = '\0';
  if (!valid) {
    ++parser_error_count_;
    return false;
  }

  temperature_ = temperature;
  depth_ = depth;
  connected_ = true;
  frame_ready_ = true;
  ++valid_frame_count_;
  return true;
}

void M14_UART_Backend::getDiagnostics(DepthDiagnostics &out) const {
  out = {};
  if (ops_.getDiagnostics != nullptr) {
    ops_.getDiagnostics(ops_.ctx, out);
  }
  out.connected = connected_;
  out.rx_byte_count = rx_byte_count_;
  out.valid_frame_count = valid_frame_count_;
  out.data_frame_count = valid_frame_count_;
  out.parser_error_count = parser_error_count_;
  out.last_frame_length = last_frame_length_;
  out.rx_preview_count = rx_preview_count_;
  for (uint8_t i = 0U; i < rx_preview_count_; ++i) {
    const uint8_t index = static_cast<uint8_t>(
        (rx_preview_next_ + 16U - rx_preview_count_ + i) % 16U);
    out.rx_preview[i] = rx_preview_[index];
  }
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
