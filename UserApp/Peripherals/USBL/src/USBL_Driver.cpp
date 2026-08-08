#include "USBL_Driver.hpp"

#include <cstring>

namespace auv {
namespace peripheral {

namespace {
constexpr uint16_t kChecksumOffset = 130;
constexpr uint16_t kTailOffset = 131;

int16_t readLE16(const uint8_t *data) {
  return static_cast<int16_t>(static_cast<uint16_t>(data[0]) |
                              (static_cast<uint16_t>(data[1]) << 8));
}

uint16_t readU16(const uint8_t *data) {
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

int32_t readLE32(const uint8_t *data) {
  return static_cast<int32_t>(static_cast<uint32_t>(data[0]) |
                              (static_cast<uint32_t>(data[1]) << 8) |
                              (static_cast<uint32_t>(data[2]) << 16) |
                              (static_cast<uint32_t>(data[3]) << 24));
}

float readLEFloat(const uint8_t *data) {
  float value = 0.0f;
  const uint32_t bits = static_cast<uint32_t>(readLE32(data));
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}
} // namespace

void USBL_Driver::init() {
  if (ops_.init != nullptr) {
    (void)ops_.init(ops_.ctx);
  }
  frame_len_ = 0;
  valid_frames_ = 0;
  invalid_frames_ = 0;
}

bool USBL_Driver::update(UsblState &state) {
  uint8_t temp_buf[256];
  const uint16_t len =
      (ops_.read != nullptr) ? ops_.read(ops_.ctx, temp_buf, sizeof(temp_buf))
                              : 0;
  bool has_new_frame = false;

  for (uint16_t i = 0; i < len; ++i) {
    if (parseByte(temp_buf[i])) {
      if (validateFrame()) {
        decodePacket(state);
        state_ = state;
        frame_len_ = 0;
        ++valid_frames_;
        has_new_frame = true;
      } else {
        ++invalid_frames_;
      }
    }
  }
  return has_new_frame;
}

uint16_t USBL_Driver::copyLastFrame(uint8_t *dst, uint16_t max_len) const {
  if (dst == nullptr || max_len == 0) {
    return 0;
  }
  const uint16_t count = (max_len < kTargetFrameSize) ? max_len
                                                       : kTargetFrameSize;
  std::memcpy(dst, packet_buf_, count);
  return count;
}

void USBL_Driver::getDiagnostics(UsblPortDiagnostics &out) const {
  out = {};
  out.valid_frames = valid_frames_;
  out.invalid_frames = invalid_frames_;
  if (ops_.getDiagnostics != nullptr) {
    UsblPortDiagnostics port;
    ops_.getDiagnostics(ops_.ctx, &port);
    out.events = port.events;
    out.invalid_events = port.invalid_events;
    out.write_pos = port.write_pos;
    out.dma_remaining = port.dma_remaining;
    out.dma_enabled = port.dma_enabled;
    out.uart_isr = port.uart_isr;
    std::memcpy(out.rx_preview, port.rx_preview, sizeof(out.rx_preview));
  }
}

uint8_t USBL_Driver::checkData(const uint8_t *data, uint16_t size) {
  uint8_t value = 0;
  for (uint16_t i = 0; i < size; ++i) {
    value ^= data[i];
  }
  return value;
}

bool USBL_Driver::parseByte(uint8_t byte) {
  if (frame_len_ == 0) {
    if (byte == kHeader1) {
      packet_buf_[frame_len_++] = byte;
    }
    return false;
  }

  if (frame_len_ == 1) {
    if (byte == kHeader2) {
      packet_buf_[frame_len_++] = byte;
    } else {
      frame_len_ = (byte == kHeader1) ? 1 : 0;
      if (frame_len_ == 1) {
        packet_buf_[0] = kHeader1;
      }
    }
    return false;
  }

  if (frame_len_ >= kMaxFrameSize) {
    frame_len_ = 0;
    return false;
  }

  packet_buf_[frame_len_++] = byte;
  return frame_len_ == kTargetFrameSize;
}

bool USBL_Driver::validateFrame() {
  if (frame_len_ != kTargetFrameSize || packet_buf_[0] != kHeader1 ||
      packet_buf_[1] != kHeader2 ||
      packet_buf_[kTailOffset] != kTail1 ||
      packet_buf_[kTailOffset + 1] != kTail2 ||
      checkData(packet_buf_, kChecksumOffset) != packet_buf_[kChecksumOffset]) {
    frame_len_ = 0;
    return false;
  }
  return true;
}

void USBL_Driver::decodePacket(UsblState &state) {
  state.roll = readLEFloat(packet_buf_ + 2);
  state.pitch = readLEFloat(packet_buf_ + 6);
  state.yaw = readLEFloat(packet_buf_ + 10);
  state.pressure = readLEFloat(packet_buf_ + 14);

  for (uint8_t i = 0; i < 4; ++i) {
    state.slant_range[i] = readLEFloat(packet_buf_ + 18 + i * 4);
  }

  state.latitude = readLE32(packet_buf_ + 38) * 1.0e-7f;
  state.longitude = readLE32(packet_buf_ + 42) * 1.0e-7f;

  for (uint8_t i = 0; i < 3; ++i) {
    state.time_diff[i] = readLEFloat(packet_buf_ + 46 + i * 4);
    state.passive_attitude[i] = readLE16(packet_buf_ + 59 + i * 2);
  }

  for (uint8_t i = 0; i < 4; ++i) {
    state.signal_strength[i] = readU16(packet_buf_ + 65 + i * 2);
  }
  std::memcpy(state.energy, packet_buf_ + 83, sizeof(state.energy));

  state.signal = readLEFloat(packet_buf_ + 91);
  state.gain = readLEFloat(packet_buf_ + 95);
  state.beacon_north = readLEFloat(packet_buf_ + 99);
  state.beacon_east = readLEFloat(packet_buf_ + 103);
  state.beacon_depth = readLEFloat(packet_buf_ + 107);

  state.sensor_status = packet_buf_[115];
  state.year = packet_buf_[116];
  state.month = packet_buf_[117];
  state.day = packet_buf_[118];
  state.hour = packet_buf_[119];
  state.minute = packet_buf_[120];
  state.second = readLEFloat(packet_buf_ + 121);
  state.nav_mode = packet_buf_[129];
  state.checksum = packet_buf_[130];
  state.timestamp =
      (ops_.getTickMs != nullptr) ? ops_.getTickMs(ops_.ctx) : 0U;
}

} // namespace peripheral
} // namespace auv
