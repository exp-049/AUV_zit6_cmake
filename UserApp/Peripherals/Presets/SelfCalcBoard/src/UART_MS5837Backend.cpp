#include "UART_MS5837Backend.hpp"

#include "FreeRTOS.h"
#include "MS5837_LogConfig.hpp"
#include "task.h"

#include <cstring>

namespace auv {
namespace peripheral {
namespace {

constexpr uint8_t kSof1 = 0xA5U;
constexpr uint8_t kSof2 = 0x5AU;
constexpr uint8_t kProtocolVersion = 0x01U;

constexpr uint8_t kDataType = 0x01U;
constexpr uint8_t kSetRateType = 0x10U;
constexpr uint8_t kHandshakeType = 0x20U;
constexpr uint8_t kPushrodTaskType = 0x30U;
constexpr uint8_t kRateAckType = 0x90U;
constexpr uint8_t kHandshakeAckType = 0xA0U;
constexpr uint8_t kPushrodAckType = 0xB0U;

constexpr uint8_t kSensorOk = 1U << 0;
constexpr uint32_t kHandshakeIntervalMs = 2000U;
constexpr uint32_t kWatchdogTimeoutMs = 5000U;

constexpr uint8_t kPushrodTaskPayloadLength = 12U;
constexpr uint8_t kPushrodAckPayloadLength = 7U;
constexpr int16_t kPushrodPowerMin = -1000;
constexpr int16_t kPushrodPowerMax = 1000;

uint16_t readU16(const uint8_t *data) {
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

uint32_t readU32(const uint8_t *data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

void putU16(uint8_t *data, uint16_t value) {
  data[0] = static_cast<uint8_t>(value);
  data[1] = static_cast<uint8_t>(value >> 8);
}

void putU32(uint8_t *data, uint32_t value) {
  data[0] = static_cast<uint8_t>(value);
  data[1] = static_cast<uint8_t>(value >> 8);
  data[2] = static_cast<uint8_t>(value >> 16);
  data[3] = static_cast<uint8_t>(value >> 24);
}

} // namespace

void UART_MS5837Backend::FrameParser::reset() {
  frame_ = {};
  candidate_size_ = 0U;
  expected_size_ = 0U;
}

int UART_MS5837Backend::FrameParser::push(uint8_t byte, Frame &frame_out) {
  if (candidate_size_ == 0U) {
    if (byte == kSof1) {
      candidate_size_ = 1U;
    }
    return 0;
  }

  if (candidate_size_ == 1U) {
    if (byte == kSof2) {
      candidate_size_ = 2U;
    } else if (byte == kSof1) {
      // The current byte may be the beginning of a new header.
      candidate_size_ = 1U;
    } else {
      reset();
    }
    return 0;
  }

  if (candidate_size_ == 2U) {
    frame_.type = byte;
    candidate_size_ = 3U;
    return 0;
  }

  if (candidate_size_ == 3U) {
    if (byte > kMaxPayloadLength) {
      reset();
      // Preserve a header that starts on the rejected length byte.
      if (byte == kSof1) {
        candidate_size_ = 1U;
      }
      return -1;
    }
    frame_.length = byte;
    expected_size_ = static_cast<uint8_t>(byte + 5U);
    candidate_size_ = 4U;
    return 0;
  }

  // Bytes [4, 4 + length) are payload; the final byte is the XOR checksum.
  if (candidate_size_ < static_cast<uint8_t>(4U + frame_.length)) {
    frame_.payload[candidate_size_ - 4U] = byte;
  }
  ++candidate_size_;

  if (candidate_size_ < expected_size_) {
    return 0;
  }

  uint8_t checksum = frame_.type ^ frame_.length;
  for (uint8_t i = 0U; i < frame_.length; ++i) {
    checksum ^= frame_.payload[i];
  }

  if (byte == checksum) {
    frame_out = frame_;
    reset();
    return 1;
  }

  // A bad candidate must not poison the stream. If the byte that failed the
  // checksum is itself SOF1, retain it as the beginning of the next frame;
  // otherwise the next SOF1 will be found normally.
  reset();
  if (byte == kSof1) {
    candidate_size_ = 1U;
  }
  return -1;
}

uint8_t UART_MS5837Backend::encodeFrame(uint8_t type, const uint8_t *payload,
                                         uint8_t payload_length,
                                         uint8_t *output,
                                         uint8_t output_size) {
  if (payload_length > kMaxPayloadLength || output == nullptr ||
      output_size < static_cast<uint8_t>(payload_length + 5U) ||
      (payload_length != 0U && payload == nullptr)) {
    return 0U;
  }

  output[0] = kSof1;
  output[1] = kSof2;
  output[2] = type;
  output[3] = payload_length;
  if (payload_length != 0U) {
    std::memcpy(&output[4], payload, payload_length);
  }

  uint8_t checksum = type ^ payload_length;
  for (uint8_t i = 0U; i < payload_length; ++i) {
    checksum ^= payload[i];
  }
  output[4U + payload_length] = checksum;
  return static_cast<uint8_t>(payload_length + 5U);
}

uint16_t UART_MS5837Backend::pushrodCrc16Update(uint16_t crc,
                                                 uint8_t data) {
  crc ^= static_cast<uint16_t>(data) << 8;
  for (uint8_t bit = 0U; bit < 8U; ++bit) {
    crc = (crc & 0x8000U) != 0U
              ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
              : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

bool UART_MS5837Backend::encodePushrodTask(const PushrodTask &task,
                                           uint8_t *output,
                                           uint8_t output_size,
                                           uint8_t *length_out) {
  if (task.power_x1000 < kPushrodPowerMin ||
      task.power_x1000 > kPushrodPowerMax || task.duration_ms == 0U ||
      output == nullptr || length_out == nullptr ||
      output_size < static_cast<uint8_t>(kPushrodTaskPayloadLength + 5U)) {
    return false;
  }

  uint8_t payload[kPushrodTaskPayloadLength] = {};
  putU32(&payload[0], task.task_id);
  putU16(&payload[4], static_cast<uint16_t>(task.power_x1000));
  putU32(&payload[6], task.duration_ms);

  uint16_t inner_crc = 0xFFFFU;
  inner_crc = pushrodCrc16Update(inner_crc, kPushrodTaskType);
  inner_crc = pushrodCrc16Update(inner_crc, kPushrodTaskPayloadLength);
  for (uint8_t i = 0U; i < 10U; ++i) {
    inner_crc = pushrodCrc16Update(inner_crc, payload[i]);
  }
  putU16(&payload[10], inner_crc);

  *length_out = encodeFrame(kPushrodTaskType, payload,
                            kPushrodTaskPayloadLength, output, output_size);
  return *length_out != 0U;
}

bool UART_MS5837Backend::decodePushrodAck(const Frame &frame,
                                          PushrodAck &ack) {
  if (frame.type != kPushrodAckType ||
      frame.length != kPushrodAckPayloadLength) {
    return false;
  }

  ack.task_id = readU32(&frame.payload[0]);
  ack.result = frame.payload[4];
  ack.queue_count = frame.payload[5];
  ack.ready = frame.payload[6];
  return true;
}

UART_MS5837Backend::UART_MS5837Backend(UART_MS5837PortOps ops) : ops_(ops) {
  parser_.reset();
  MS5837_LOG_DEBUG("[MS5837 UART] protocol backend constructed, ops=%p", &ops_);
}

bool UART_MS5837Backend::init() {
  parser_.reset();
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
  rx_byte_count_ = 0U;
  valid_frame_count_ = 0U;
  parser_error_count_ = 0U;
  data_frame_count_ = 0U;
  handshake_ack_count_ = 0U;
  pushrod_ack_count_ = 0U;
  sensor_not_ready_count_ = 0U;
  last_frame_type_ = 0U;
  last_frame_length_ = 0U;
  std::memset(rx_preview_, 0, sizeof(rx_preview_));
  rx_preview_next_ = 0U;
  rx_preview_count_ = 0U;
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
        static_cast<uint32_t>(now_ms - last_link_ms_) >=
            kWatchdogTimeoutMs) {
      connected_ = false;
    }
    if (handshake_ack_ &&
        static_cast<uint32_t>(now_ms - last_handshake_ack_ms_) >=
            kWatchdogTimeoutMs) {
      handshake_ack_ = false;
    }

    if (!handshake_attempted_ ||
        static_cast<uint32_t>(now_ms - last_handshake_ms_) >=
            kHandshakeIntervalMs) {
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

bool UART_MS5837Backend::sendTask(const PushrodTask &task) {
  if (ops_.transmit == nullptr) {
    return false;
  }

  uint8_t packet[kMaxFrameSize] = {};
  uint8_t length = 0U;
  if (!encodePushrodTask(task, packet, sizeof(packet), &length)) {
    return false;
  }
  return ops_.transmit(ops_.ctx, packet, length);
}

bool UART_MS5837Backend::readAck(PushrodAck *ack) {
  if (ack == nullptr) {
    return false;
  }

  taskENTER_CRITICAL();
  if (!pushrod_ack_ready_) {
    taskEXIT_CRITICAL();
    return false;
  }
  *ack = pushrod_ack_;
  pushrod_ack_ready_ = false;
  taskEXIT_CRITICAL();
  return true;
}

void UART_MS5837Backend::onRxByte(uint8_t byte) {
  recordRxByte(byte);
  Frame frame{};
  const int parse_result = parser_.push(byte, frame);
  if (parse_result < 0) {
    ++parser_error_count_;
  }
  if (parse_result != 1) {
    return;
  }

  ++valid_frame_count_;
  last_frame_type_ = frame.type;
  last_frame_length_ = frame.length;

  if (frame.type == kPushrodAckType) {
    PushrodAck ack{};
    if (decodePushrodAck(frame, ack)) {
      ++pushrod_ack_count_;
      taskENTER_CRITICAL();
      pushrod_ack_ = ack;
      pushrod_ack_ready_ = true;
      taskEXIT_CRITICAL();
    }
    return;
  }

  // DATA is the only frame that carries the z-axis measurement. Its
  // little-endian payload is depth_cm, pressure_01mbar, temp_01c, sample_seq,
  // status. Pressure and sequence are intentionally ignored here.
  if (frame.type != kDataType || frame.length != kDataPayloadLength) {
    if (frame.type == kHandshakeAckType && frame.length == 9U) {
      ++handshake_ack_count_;
      const uint32_t host_nonce = readU32(&frame.payload[0]);
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

  ++data_frame_count_;
  const uint16_t depth_cm = readU16(&frame.payload[0]);
  const int16_t temperature_01c =
      static_cast<int16_t>(readU16(&frame.payload[6]));
  const uint8_t status = frame.payload[10];

  connected_ = true;
  last_link_ms_ =
      ops_.getTickMs != nullptr ? ops_.getTickMs(ops_.ctx) : last_link_ms_;
  if ((status & kSensorOk) == 0U) {
    ++sensor_not_ready_count_;
    return;
  }

  depth_m_ = static_cast<float>(depth_cm) * 0.01f;
  temperature_c_ = static_cast<float>(temperature_01c) * 0.1f;
  frame_ready_ = true;
}

void UART_MS5837Backend::recordRxByte(uint8_t byte) {
  ++rx_byte_count_;
  rx_preview_[rx_preview_next_] = byte;
  rx_preview_next_ = static_cast<uint8_t>((rx_preview_next_ + 1U) % 16U);
  if (rx_preview_count_ < 16U) {
    ++rx_preview_count_;
  }
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

  uint8_t payload[4] = {};
  uint8_t packet[kMaxFrameSize] = {};
  const uint32_t host_nonce = next_host_nonce_++;
  putU32(payload, host_nonce);
  const uint8_t length =
      encodeFrame(kHandshakeType, payload, sizeof(payload), packet, sizeof(packet));
  if (length == 0U) {
    return;
  }

  const bool sent = ops_.transmit(ops_.ctx, packet, length);
  handshake_attempted_ = true;
  last_handshake_ms_ = now_ms;
  if (sent) {
    expected_host_nonce_ = host_nonce;
    MS5837_LOG_DEBUG("[MS5837 UART] handshake sent nonce=0x%08lx",
                     static_cast<unsigned long>(host_nonce));
  } else {
    MS5837_LOG_DEBUG("[MS5837 UART] handshake transmit failed");
  }
}

} // namespace peripheral
} // namespace auv
