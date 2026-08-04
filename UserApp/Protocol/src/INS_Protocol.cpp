#include "INS_Protocol.hpp"

#include <cstring>

namespace auv {
namespace protocol {
namespace ins {
namespace {

constexpr uint16_t kRoll = 2U;
constexpr uint16_t kPitch = 6U;
constexpr uint16_t kYaw = 10U;
constexpr uint16_t kRollRate = 14U;
constexpr uint16_t kPitchRate = 18U;
constexpr uint16_t kYawRate = 22U;
constexpr uint16_t kVx = 26U;
constexpr uint16_t kVy = 30U;
constexpr uint16_t kVz = 34U;
constexpr uint16_t kLat = 38U;
constexpr uint16_t kLon = 42U;
constexpr uint16_t kDepth = 46U;
constexpr uint16_t kNorthOffset = 99U;
constexpr uint16_t kEastOffset = 103U;
constexpr uint16_t kPressureDepth = 107U;
constexpr uint16_t kDvlAltitude = 111U;
constexpr uint16_t kSensorFlags = 115U;
constexpr uint16_t kNavMode = 129U;
constexpr uint16_t kChecksum = 130U;

int32_t readLE32(const uint8_t *data) {
  const uint32_t value = static_cast<uint32_t>(data[0]) |
                         (static_cast<uint32_t>(data[1]) << 8) |
                         (static_cast<uint32_t>(data[2]) << 16) |
                         (static_cast<uint32_t>(data[3]) << 24);
  return static_cast<int32_t>(value);
}

float readLEFloat(const uint8_t *data) {
  const uint32_t bits = static_cast<uint32_t>(data[0]) |
                        (static_cast<uint32_t>(data[1]) << 8) |
                        (static_cast<uint32_t>(data[2]) << 16) |
                        (static_cast<uint32_t>(data[3]) << 24);
  float value = 0.0f;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void writeBE32(uint8_t *data, int32_t value) {
  const uint32_t raw = static_cast<uint32_t>(value);
  data[0] = static_cast<uint8_t>(raw >> 24);
  data[1] = static_cast<uint8_t>(raw >> 16);
  data[2] = static_cast<uint8_t>(raw >> 8);
  data[3] = static_cast<uint8_t>(raw);
}

} // namespace

void Parser::reset() {
  working_length_ = 0U;
  has_last_frame_ = false;
}

int Parser::pushByte(uint8_t byte) {
  if (working_length_ == 0U) {
    if (byte == kHeader1) {
      working_frame_[working_length_++] = byte;
    }
    return 0;
  }

  if (working_length_ == 1U) {
    if (byte == kHeader2) {
      working_frame_[working_length_++] = byte;
    } else {
      working_length_ = 0U;
      if (byte == kHeader1) {
        working_frame_[working_length_++] = byte;
      }
    }
    return 0;
  }

  if (working_length_ >= kMaxFrameSize) {
    working_length_ = 0U;
    return 0;
  }

  working_frame_[working_length_++] = byte;
  if (working_length_ != kFrameSize) {
    return 0;
  }

  if (validateWorkingFrame()) {
    std::memcpy(last_frame_, working_frame_, kFrameSize);
    has_last_frame_ = true;
    working_length_ = 0U;
    return 1;
  }

  resynchronizeAfterInvalidFrame();
  return -1;
}

bool Parser::validateWorkingFrame() const {
  if (working_length_ != kFrameSize || working_frame_[0] != kHeader1 ||
      working_frame_[1] != kHeader2 || working_frame_[kFrameSize - 2U] != kTail1 ||
      working_frame_[kFrameSize - 1U] != kTail2) {
    return false;
  }

  uint8_t checksum = 0U;
  for (uint16_t i = 0U; i < kChecksum; ++i) {
    checksum ^= working_frame_[i];
  }
  return checksum == working_frame_[kChecksum];
}

void Parser::resynchronizeAfterInvalidFrame() {
  for (uint16_t search_pos = 2U; search_pos < working_length_; ++search_pos) {
    if (working_frame_[search_pos] == kHeader1) {
      const uint16_t remaining = working_length_ - search_pos;
      std::memmove(working_frame_, working_frame_ + search_pos, remaining);
      working_length_ = remaining;
      return;
    }
  }
  working_length_ = 0U;
}

bool Parser::decode(NavigationPacket &packet) const {
  if (!has_last_frame_) {
    return false;
  }

  packet.roll_deg = readLEFloat(last_frame_ + kRoll);
  packet.pitch_deg = readLEFloat(last_frame_ + kPitch);
  packet.yaw_deg = readLEFloat(last_frame_ + kYaw);
  packet.roll_rate = readLEFloat(last_frame_ + kRollRate);
  packet.pitch_rate = readLEFloat(last_frame_ + kPitchRate);
  packet.yaw_rate = readLEFloat(last_frame_ + kYawRate);
  packet.velocity_x = readLEFloat(last_frame_ + kVx);
  packet.velocity_y = readLEFloat(last_frame_ + kVy);
  packet.velocity_z = readLEFloat(last_frame_ + kVz);
  packet.latitude_e7 = readLE32(last_frame_ + kLat);
  packet.longitude_e7 = readLE32(last_frame_ + kLon);
  packet.combined_depth = readLEFloat(last_frame_ + kDepth);
  packet.north_offset = readLEFloat(last_frame_ + kNorthOffset);
  packet.east_offset = readLEFloat(last_frame_ + kEastOffset);
  packet.pressure_depth = readLEFloat(last_frame_ + kPressureDepth);
  packet.dvl_altitude = readLEFloat(last_frame_ + kDvlAltitude);
  packet.sensor_flags = last_frame_[kSensorFlags];
  packet.navigation_mode = last_frame_[kNavMode];
  return true;
}

uint16_t Parser::copyLastFrame(uint8_t *dst, uint16_t max_len) const {
  if (!has_last_frame_ || dst == nullptr || max_len == 0U) {
    return 0U;
  }
  const uint16_t count = max_len < kFrameSize ? max_len : kFrameSize;
  std::memcpy(dst, last_frame_, count);
  return count;
}

uint16_t Parser::encodeCommand(uint8_t command_id, const uint8_t *data,
                               uint8_t data_len, uint8_t *output,
                               uint16_t output_size) {
  constexpr uint8_t kPayloadSize = 8U;
  constexpr uint8_t kChecksumOffset = 11U;

  if (output == nullptr || output_size < commandFrameSize() ||
      (data_len > 0U && data == nullptr)) {
    return 0U;
  }

  std::memset(output, 0, commandFrameSize());
  output[0] = 0xFC;
  output[1] = 0xCF;
  output[2] = command_id;
  const uint8_t copy_len = data_len < kPayloadSize ? data_len : kPayloadSize;
  if (copy_len > 0U) {
    std::memcpy(output + 3U, data, copy_len);
  }

  uint8_t checksum = 0U;
  for (uint8_t i = 0U; i < kChecksumOffset; ++i) {
    checksum ^= output[i];
  }
  output[kChecksumOffset] = checksum;
  output[12] = 0xFD;
  output[13] = 0xDF;
  return commandFrameSize();
}

void Parser::encodeInitialPosition(double latitude, double longitude,
                                   uint8_t output[8]) {
  if (output == nullptr) {
    return;
  }
  writeBE32(output, static_cast<int32_t>(latitude * 1e7));
  writeBE32(output + 4U, static_cast<int32_t>(longitude * 1e7));
}

} // namespace ins
} // namespace protocol
} // namespace auv
