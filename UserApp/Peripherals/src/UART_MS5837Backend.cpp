#include "UART_MS5837Backend.hpp"

#include "MS5837_LogConfig.hpp"

namespace auv {
namespace peripheral {

UART_MS5837Backend::UART_MS5837Backend(UART_MS5837PortOps ops) : ops_(ops) {
  ms5837_protocol_parser_init(&parser_);
  MS5837_LOG_DEBUG("[MS5837 UART] protocol backend constructed, ops=%p", &ops_);
}

bool UART_MS5837Backend::init() {
  ms5837_protocol_parser_init(&parser_);
  depth_m_ = 0.0f;
  temperature_c_ = 0.0f;
  frame_ready_ = false;
  connected_ = false;
  handshake_attempted_ = false;
  handshake_ack_ = false;
  last_handshake_ms_ = 0U;
  last_handshake_ack_ms_ = 0U;
  last_link_ms_ = 0U;
  next_host_nonce_ = 0U;
  expected_host_nonce_ = 0U;
  pushrod_ack_ = {};
  pushrod_ack_ready_ = false;

  const bool ready = ops_.transmit != nullptr && ops_.poll != nullptr &&
                     ops_.startRx != nullptr && ops_.getTickMs != nullptr;
  MS5837_LOG_DEBUG("[MS5837 UART] init: transport=%d", ready);
  return ready;
}

void UART_MS5837Backend::poll() {
  if (ops_.poll != nullptr) {
    ops_.poll(ops_.ctx);
  }

  if (ops_.getTickMs != nullptr) {
    const uint32_t now_ms = ops_.getTickMs(ops_.ctx);

    if (connected_ &&
        (uint32_t)(now_ms - last_link_ms_) >=
            MS5837_PROTOCOL_WATCHDOG_TIMEOUT_MS) {
      connected_ = false;
    }
    if (handshake_ack_ &&
        (uint32_t)(now_ms - last_handshake_ack_ms_) >=
            MS5837_PROTOCOL_WATCHDOG_TIMEOUT_MS) {
      handshake_ack_ = false;
    }

    if (!handshake_attempted_ ||
        (uint32_t)(now_ms - last_handshake_ms_) >=
            MS5837_PROTOCOL_HANDSHAKE_INTERVAL_MS) {
      sendHandshake(now_ms);
    }
  }
}

bool UART_MS5837Backend::read() {
  if (!frame_ready_) {
    return false;
  }

  frame_ready_ = false;
  if (cb_.onDepthReady != nullptr) {
    cb_.onDepthReady(cb_.ctx, depth_m_, temperature_c_);
  }
  return true;
}

bool UART_MS5837Backend::sendPushrodTask(
    const pushrod_protocol_task_t &task) {
  if (ops_.transmit == nullptr) {
    return false;
  }

  uint8_t packet[MS5837_PROTOCOL_MAX_FRAME_SIZE] = {};
  const uint8_t length = pushrod_protocol_encode_task(
      &task, packet, sizeof(packet));
  if (length == 0U) {
    return false;
  }
  return ops_.transmit(ops_.ctx, packet, length);
}

bool UART_MS5837Backend::readPushrodAck(pushrod_protocol_ack_t *ack) {
  if (ack == nullptr || !pushrod_ack_ready_) {
    return false;
  }
  *ack = pushrod_ack_;
  pushrod_ack_ready_ = false;
  return true;
}

void UART_MS5837Backend::onRxByte(uint8_t byte) {
  ms5837_protocol_frame_t frame{};
  const int result = ms5837_protocol_parser_push(&parser_, byte, &frame);
  if (result != 1) {
    return;
  }

  if (frame.type == PUSHROD_PROTOCOL_TYPE_ACK) {
    pushrod_protocol_ack_t ack{};
    if (pushrod_protocol_decode_ack(&frame, &ack) == 0) {
      pushrod_ack_ = ack;
      pushrod_ack_ready_ = true;
    }
    return;
  }

  // DATA is the only frame that carries the z-axis measurement. The V1
  // payload is little-endian: depth_cm, pressure_01mbar, temp_01c,
  // sample_seq, status. Pressure and sequence are intentionally ignored.
  if (frame.type != MS5837_PROTOCOL_TYPE_DATA ||
      frame.length != kDataPayloadLength) {
    if (frame.type == MS5837_PROTOCOL_TYPE_HANDSHAKE_ACK &&
        frame.length == 9U) {
      const uint32_t host_nonce =
          (uint32_t)frame.payload[0] |
          ((uint32_t)frame.payload[1] << 8) |
          ((uint32_t)frame.payload[2] << 16) |
          ((uint32_t)frame.payload[3] << 24);
      if (host_nonce == expected_host_nonce_) {
        handshake_ack_ = true;
        connected_ = true;
        last_handshake_ack_ms_ =
            ops_.getTickMs != nullptr ? ops_.getTickMs(ops_.ctx) : 0U;
        last_link_ms_ = last_handshake_ack_ms_;
      }
    }
    return;
  }

  const uint16_t depth_cm = (uint16_t)frame.payload[0] |
                            ((uint16_t)frame.payload[1] << 8);
  const int16_t temperature_01c =
      (int16_t)((uint16_t)frame.payload[6] |
                ((uint16_t)frame.payload[7] << 8));
  const uint8_t status = frame.payload[10];

  // A valid frame proves the UART link is alive. Only SENSOR_OK frames are
  // allowed to replace z, so a sensor fault cannot overwrite pos.z with 0.
  connected_ = true;
  last_link_ms_ =
      ops_.getTickMs != nullptr ? ops_.getTickMs(ops_.ctx) : last_link_ms_;
  if ((status & MS5837_PROTOCOL_STATUS_SENSOR_OK) == 0U) {
    return;
  }

  depth_m_ = (float)depth_cm * 0.01f;
  temperature_c_ = (float)temperature_01c * 0.1f;
  frame_ready_ = true;
}

void UART_MS5837Backend::start() {
  if (ops_.startRx == nullptr) {
    MS5837_LOG_DEBUG("[MS5837 UART] start: startRx hook is null");
    return;
  }

  const bool ok = ops_.startRx(ops_.ctx);
  MS5837_LOG_DEBUG("[MS5837 UART] start RX: %d", ok);

  if (ok && ops_.getTickMs != nullptr) {
    const uint32_t now_ms = ops_.getTickMs(ops_.ctx);
    next_host_nonce_ = 0x4D533700UL ^ now_ms;
    sendHandshake(now_ms);
  }
}

void UART_MS5837Backend::sendHandshake(uint32_t now_ms) {
  if (ops_.transmit == nullptr) {
    return;
  }

  uint8_t packet[MS5837_PROTOCOL_MAX_FRAME_SIZE] = {};
  const uint32_t host_nonce = next_host_nonce_++;
  const uint8_t length = ms5837_protocol_encode_handshake(
      host_nonce, packet, sizeof(packet));
  if (length == 0U) {
    return;
  }

  const bool sent = ops_.transmit(ops_.ctx, packet, length);
  handshake_attempted_ = true;
  last_handshake_ms_ = now_ms;
  if (sent) {
    expected_host_nonce_ = host_nonce;
    MS5837_LOG_DEBUG("[MS5837 UART] handshake sent nonce=0x%08lx",
                     (unsigned long)host_nonce);
  } else {
    MS5837_LOG_DEBUG("[MS5837 UART] handshake transmit failed");
  }
}

} // namespace peripheral
} // namespace auv
