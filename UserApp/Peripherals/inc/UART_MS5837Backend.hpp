#pragma once

#include "MS5837_Driver.hpp"
#include "ms5837_protocol.h"
#include "pushrod_protocol.h"

#include <stdint.h>

namespace auv {
namespace peripheral {

/** Hardware hooks supplied by the UART4 porting layer. */
struct UART_MS5837PortOps {
  void *ctx;
  bool (*transmit)(void *ctx, const uint8_t *data, uint16_t length);
  void (*poll)(void *ctx);
  bool (*startRx)(void *ctx);
  uint32_t (*getTickMs)(void *ctx);
  uint32_t (*getRxRecoveryCount)(void *ctx);
  uint32_t (*getRxErrorCount)(void *ctx);
  uint32_t (*getLastRxError)(void *ctx);
  uint32_t (*getLastRxRecoveryReason)(void *ctx);
  uint32_t (*getRxEventCount)(void *ctx);
  uint32_t (*getDmaWritePos)(void *ctx);
};

/**
 * @brief Depth-board protocol-v1 UART backend.
 *
 * The backend consumes the byte stream produced by the UART porting layer,
 * uses the protocol parsers from UserApp/Protocol, and exposes depth in m.
 * A completed DATA frame is consumed by MS5837_Driver::Read(), while pushrod
 * acknowledgements are exposed through MS5837_Driver::readPushrodAck().
 */
class UART_MS5837Backend final : public DepthBackend {
public:
  explicit UART_MS5837Backend(UART_MS5837PortOps ops);

  bool init() override;
  void poll() override;
  bool read() override;
  void setCallback(DepthDataReadyCallback cb) override { cb_ = cb; }
  void start() override;

  bool isConnected() const override { return connected_; }
  float getDepth() const override { return depth_m_; }
  float getTemperature() const override { return temperature_c_; }
  bool sendPushrodTask(const pushrod_protocol_task_t &task) override;
  bool readPushrodAck(pushrod_protocol_ack_t *ack) override;
  bool isHandshakeAcknowledged() const override { return handshake_ack_; }
  uint32_t getRxRecoveryCount() const override {
    return ops_.getRxRecoveryCount != nullptr
               ? ops_.getRxRecoveryCount(ops_.ctx)
               : 0U;
  }
  uint32_t getRxErrorCount() const override {
    return ops_.getRxErrorCount != nullptr ? ops_.getRxErrorCount(ops_.ctx)
                                            : 0U;
  }
  uint32_t getLastRxError() const override {
    return ops_.getLastRxError != nullptr ? ops_.getLastRxError(ops_.ctx) : 0U;
  }
  uint32_t getLastRxRecoveryReason() const override {
    return ops_.getLastRxRecoveryReason != nullptr
               ? ops_.getLastRxRecoveryReason(ops_.ctx)
               : 0U;
  }
  uint32_t getRxEventCount() const override {
    return ops_.getRxEventCount != nullptr ? ops_.getRxEventCount(ops_.ctx)
                                            : 0U;
  }
  uint32_t getDmaWritePos() const override {
    return ops_.getDmaWritePos != nullptr ? ops_.getDmaWritePos(ops_.ctx) : 0U;
  }

  /** Feed one byte from the UART DMA consumer into the protocol parser. */
  void onRxByte(uint8_t byte);

  /** Drop a partial frame after the transport has been forcibly restarted. */
  void onRxReset() {
    ms5837_protocol_parser_init(&parser_);
    frame_ready_ = false;
    pushrod_ack_ready_ = false;
  }

private:
  static constexpr uint8_t kDataPayloadLength = 11U;

  void sendHandshake(uint32_t now_ms);

  UART_MS5837PortOps ops_;
  ms5837_protocol_parser_t parser_{};

  DepthDataReadyCallback cb_{nullptr, nullptr};
  float depth_m_ = 0.0f;
  float temperature_c_ = 0.0f;
  bool frame_ready_ = false;
  bool connected_ = false;
  bool handshake_attempted_ = false;
  bool handshake_ack_ = false;
  uint32_t last_handshake_ms_ = 0U;
  uint32_t last_handshake_ack_ms_ = 0U;
  uint32_t last_link_ms_ = 0U;
  uint32_t next_host_nonce_ = 0U;
  uint32_t expected_host_nonce_ = 0U;
  pushrod_protocol_ack_t pushrod_ack_{};
  bool pushrod_ack_ready_ = false;
};

} // namespace peripheral
} // namespace auv
