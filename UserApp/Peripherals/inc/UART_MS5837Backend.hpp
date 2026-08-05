#pragma once

#include "Depth_Sensor_Driver.hpp"
#include "Pushrod_Backend.hpp"

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
 * owns the self-calculation-board wire parser and pushrod codec, and exposes
 * depth in m.
 * A completed DATA frame is consumed by Depth_Sensor_Driver::Read(), while
 * pushrod acknowledgements are exposed through Pushrod_Driver::readAck().
 */
class UART_MS5837Backend final : public DepthBackend, public PushrodBackend {
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
  bool sendTask(const PushrodTask &task) override;
  bool readAck(PushrodAck *ack) override;
  bool isSupported() const override { return true; }
  bool isHandshakeAcknowledged() const { return handshake_ack_; }
  uint32_t getRxRecoveryCount() const {
    return ops_.getRxRecoveryCount != nullptr
               ? ops_.getRxRecoveryCount(ops_.ctx)
               : 0U;
  }
  uint32_t getRxErrorCount() const {
    return ops_.getRxErrorCount != nullptr ? ops_.getRxErrorCount(ops_.ctx)
                                            : 0U;
  }
  uint32_t getLastRxError() const {
    return ops_.getLastRxError != nullptr ? ops_.getLastRxError(ops_.ctx) : 0U;
  }
  uint32_t getLastRxRecoveryReason() const {
    return ops_.getLastRxRecoveryReason != nullptr
               ? ops_.getLastRxRecoveryReason(ops_.ctx)
               : 0U;
  }
  uint32_t getRxEventCount() const {
    return ops_.getRxEventCount != nullptr ? ops_.getRxEventCount(ops_.ctx)
                                            : 0U;
  }
  uint32_t getDmaWritePos() const {
    return ops_.getDmaWritePos != nullptr ? ops_.getDmaWritePos(ops_.ctx) : 0U;
  }

  void getDiagnostics(DepthDiagnostics &out) const override {
    out = {};
    out.connected = connected_;
    out.handshake_acknowledged = handshake_ack_;
    out.rx_recovery_count = getRxRecoveryCount();
    out.rx_error_count = getRxErrorCount();
    out.last_rx_error = getLastRxError();
    out.last_rx_recovery_reason = getLastRxRecoveryReason();
    out.rx_event_count = getRxEventCount();
    out.dma_write_pos = getDmaWritePos();
  }

  /** Feed one byte from the UART DMA consumer into the protocol parser. */
  void onRxByte(uint8_t byte);

  /** Drop a partial frame after the transport has been forcibly restarted. */
  void onRxReset() {
    parser_.reset();
    frame_ready_ = false;
    pushrod_ack_ready_ = false;
  }

private:
  static constexpr uint8_t kMaxPayloadLength = 32U;
  static constexpr uint8_t kMaxFrameSize = kMaxPayloadLength + 5U;
  static constexpr uint8_t kDataPayloadLength = 11U;

  /** Private wire frame; no protocol-frame type leaks into the public API. */
  struct Frame {
    uint8_t type = 0U;
    uint8_t length = 0U;
    uint8_t payload[kMaxPayloadLength] = {};
  };

  /** Streaming parser for the self calculation board's UART frames. */
  class FrameParser {
  public:
    void reset();
    int push(uint8_t byte, Frame &frame_out);

  private:
    enum State : uint8_t {
      kSof1,
      kSof2,
      kType,
      kLength,
      kPayload,
      kChecksum,
    };

    State state_ = kSof1;
    Frame frame_{};
    uint8_t payload_index_ = 0U;
    uint8_t checksum_ = 0U;
  };

  static uint8_t encodeFrame(uint8_t type, const uint8_t *payload,
                             uint8_t payload_length, uint8_t *output,
                             uint8_t output_size);
  static uint16_t pushrodCrc16Update(uint16_t crc, uint8_t data);
  static bool encodePushrodTask(const PushrodTask &task, uint8_t *output,
                                uint8_t output_size, uint8_t *length_out);
  static bool decodePushrodAck(const Frame &frame, PushrodAck &ack);

  void sendHandshake(uint32_t now_ms);

  UART_MS5837PortOps ops_;
  FrameParser parser_{};

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
  PushrodAck pushrod_ack_{};
  bool pushrod_ack_ready_ = false;
};

using Self_CalcBoard_Link = UART_MS5837Backend;

} // namespace peripheral
} // namespace auv
