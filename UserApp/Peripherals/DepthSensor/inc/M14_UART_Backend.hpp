#pragma once

#include "UART_DepthBackend.hpp"

#include <stdint.h>

namespace auv {
namespace peripheral {

/**
 * M14 metal-diaphragm depth sensor backend.
 *
 * The M14 wire protocol is intentionally kept inside this backend:
 *   data:    T=XX.XXD=XX.XX\r\n
 *   command: !Dxx.xx\r\n, !Txx.xx\r\n, !Fxxxx\r\n, ...
 */
class M14_UART_Backend final : public DepthBackend, public DepthUartRxSink {
public:
  explicit M14_UART_Backend(UartPortOps ops);

  bool init() override;
  void poll() override;
  bool read() override;
  void setCallback(DepthDataReadyCallback cb) override { cb_ = cb; }
  void start() override;
  bool isConnected() const override { return connected_; }
  float getDepth() const override { return depth_; }
  float getTemperature() const override { return temperature_; }

  /** Feed one byte from the UART DMA consumer. */
  void onRxByte(uint8_t byte) override;

  /** Send one M14 command; CRLF is appended automatically. */
  bool sendCommand(const char *command);
  bool setFluidDensity(uint16_t density);
  bool setDepthOffset(float offset_m);
  bool setTemperatureOffset(float offset_c);
  bool toggleParameterOutput();
  bool resetSensor();
  bool restoreFactorySettings();
  bool clearOffsets();

private:
  static constexpr uint16_t kLineBufferSize = 128U;

  bool finishLine();

  UartPortOps ops_;
  char line_buffer_[kLineBufferSize] = {};
  uint16_t line_length_ = 0U;
  bool frame_ready_ = false;
  bool connected_ = false;
  float depth_ = 0.0f;
  float temperature_ = 0.0f;
  DepthDataReadyCallback cb_{nullptr, nullptr};
};

} // namespace peripheral
} // namespace auv
