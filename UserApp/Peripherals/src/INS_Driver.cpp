#include "INS_Driver.hpp"
#include "AppContext.hpp"
#include "RosLogger.hpp"
#include "SoftWatchdog.hpp"
#include "SystemContext.hpp"
#include "main.h" // HAL_GetTick, HAL_Delay

#include <cstdint>
#include <cstring>

namespace auv {
namespace peripheral {
namespace {

constexpr uint8_t kHeader1 = 0xFAU;
constexpr uint8_t kHeader2 = 0xAFU;
constexpr uint8_t kTail1 = 0xFBU;
constexpr uint8_t kTail2 = 0xBFU;

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

void INS_Driver::ProtocolParser::reset() {
  working_length_ = 0U;
  has_last_frame_ = false;
}

int INS_Driver::ProtocolParser::pushByte(uint8_t byte) {
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

bool INS_Driver::ProtocolParser::validateWorkingFrame() const {
  if (working_length_ != kFrameSize || working_frame_[0] != kHeader1 ||
      working_frame_[1] != kHeader2 ||
      working_frame_[kFrameSize - 2U] != kTail1 ||
      working_frame_[kFrameSize - 1U] != kTail2) {
    return false;
  }

  uint8_t checksum = 0U;
  for (uint16_t i = 0U; i < kChecksum; ++i) {
    checksum ^= working_frame_[i];
  }
  return checksum == working_frame_[kChecksum];
}

void INS_Driver::ProtocolParser::resynchronizeAfterInvalidFrame() {
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

bool INS_Driver::ProtocolParser::decode(INS_Driver::ProtocolPacket &packet) const {
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

uint16_t INS_Driver::ProtocolParser::copyLastFrame(uint8_t *dst,
                                                    uint16_t max_len) const {
  if (!has_last_frame_ || dst == nullptr || max_len == 0U) {
    return 0U;
  }
  const uint16_t count = max_len < kFrameSize ? max_len : kFrameSize;
  std::memcpy(dst, last_frame_, count);
  return count;
}

uint16_t INS_Driver::ProtocolParser::encodeCommand(
    uint8_t command_id, const uint8_t *data, uint8_t data_len, uint8_t *output,
    uint16_t output_size) {
  constexpr uint8_t kPayloadSize = 8U;
  constexpr uint8_t kChecksumOffset = 11U;

  if (output == nullptr || output_size < commandFrameSize() ||
      (data_len > 0U && data == nullptr)) {
    return 0U;
  }

  std::memset(output, 0, commandFrameSize());
  output[0] = 0xFCU;
  output[1] = 0xCFU;
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
  output[12] = 0xFDU;
  output[13] = 0xDFU;
  return commandFrameSize();
}

void INS_Driver::ProtocolParser::encodeInitialPosition(double latitude,
                                                         double longitude,
                                                         uint8_t output[8]) {
  if (output == nullptr) {
    return;
  }
  writeBE32(output, static_cast<int32_t>(latitude * 1e7));
  writeBE32(output + 4U, static_cast<int32_t>(longitude * 1e7));
}

void INS_Driver::init() {
  for (int i = 0; i < 5; i++) {
    if (ops_.init && ops_.init(ops_.ctx))
      break;
    HAL_Delay(10);
  }
  protocol_parser_.reset();
  rx_total_bytes_ = 0;
  valid_frames_ = 0;
  invalid_frames_ = 0;
  resetPosition();
}

void INS_Driver::sendCommand(uint8_t cmd_id, uint8_t value) {
  uint8_t data[8] = {value, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendCommand(cmd_id, data, 8);
}

void INS_Driver::sendCommand(uint8_t cmd_id, const uint8_t *data,
                             uint8_t data_len) {
  uint8_t cmd[ProtocolParser::commandFrameSize()] = {};
  if (ops_.transmit == nullptr ||
      ProtocolParser::encodeCommand(cmd_id, data, data_len, cmd, sizeof(cmd)) ==
          0U) {
    return;
  }

  for (int i = 0; i < 3; i++) {
    ops_.transmit(ops_.ctx, cmd, sizeof(cmd));
    if (i < 2)
      HAL_Delay(10);
  }
}

void INS_Driver::resetPosition() { sendCommand(0x02, 0x00); }

void INS_Driver::setDvlPower(bool on) {
  // 手册：03 为 DVL 电源控制位, 01 为开, 00 为关
  sendCommand(0x03, on ? 0x01 : 0x00);
}

void INS_Driver::restart() {
  // 手册：04 为重启指令
  sendCommand(0x04, 0x00);
}

void INS_Driver::setInitialPosition(double lat, double lon) {
  uint8_t data[8];
  ProtocolParser::encodeInitialPosition(lat, lon, data);
  sendCommand(0x20, data, 8);
}

bool INS_Driver::isDataFresh() const {
  bool fresh = (HAL_GetTick() - last_update_ms_ < 200);
  if (!fresh) {
    static uint32_t last_warn_ms = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_warn_ms >= 2000) {
      last_warn_ms = now;
      ROS_LOG_WARN("INS data timeout! last update was %lu ms ago",
                   (unsigned long)(now - last_update_ms_));
    }
  }
  return fresh;
}

bool INS_Driver::update(auv::motion::NavState &state) {
  uint8_t temp_buf[256];
  uint16_t len = ops_.read(ops_.ctx, temp_buf, sizeof(temp_buf));
  rx_total_bytes_ += len;
  bool has_new_frame = false;

  for (uint16_t i = 0; i < len; i++) {
    const int parse_result = protocol_parser_.pushByte(temp_buf[i]);
    if (parse_result == 1) {
      ProtocolPacket packet;
      if (protocol_parser_.decode(packet)) {
        decodePacket(packet, state);
        last_update_ms_ = HAL_GetTick();
        ++valid_frames_;
        has_new_frame = true;
      }
    } else if (parse_result < 0) {
      ++invalid_frames_;
    }
  }
  return has_new_frame;
}

uint16_t INS_Driver::copyLastFrame(uint8_t *dst, uint16_t max_len) const {
  return protocol_parser_.copyLastFrame(dst, max_len);
}

void INS_Driver::getDiagnostics(InsPortDiagnostics &out) const {
  out = {};
  out.total_bytes = rx_total_bytes_;
  out.valid_frames = valid_frames_;
  out.invalid_frames = invalid_frames_;
  if (ops_.getDiagnostics != nullptr) {
    InsPortDiagnostics port;
    ops_.getDiagnostics(ops_.ctx, &port);
    out.read_events = port.read_events;
    out.total_bytes = port.total_bytes;
    out.write_pos = port.write_pos;
    out.dma_remaining = port.dma_remaining;
    out.dma_enabled = port.dma_enabled;
    out.uart_isr = port.uart_isr;
    std::memcpy(out.rx_preview, port.rx_preview, sizeof(out.rx_preview));
  }
}

void INS_Driver::decodePacket(const ProtocolPacket &packet,
                              auv::motion::NavState &s) {
  // 1. 姿态 → pos_world[ROLL=3, PITCH=4, YAW=5]
  s.pos_world[3] = packet.roll_deg;
  s.pos_world[4] = packet.pitch_deg;
  s.pos_world[5] = packet.yaw_deg;

  // 2. 角速度 → vel_body[ROLL=3, PITCH=4, YAW=5]
  s.vel_body[3] = packet.roll_rate;
  s.vel_body[4] = packet.pitch_rate;
  s.vel_body[5] = packet.yaw_rate;

  // 3. 机体系线速度
  s.vel_body[0] = packet.velocity_x;
  s.vel_body[1] = packet.velocity_y;
  s.vel_body[2] = packet.velocity_z;

  // 4. 经纬度 (int32, 1e7 缩放)
  {
    auto ns = auv::system::system_context.nav_status_.get();
    ns.lat = packet.latitude_e7 * 1e-7;
    ns.lon = packet.longitude_e7 * 1e-7;
    auv::system::system_context.nav_status_.set(ns);
  }

  // 5. 深度
  s.pos_world[2] = packet.combined_depth;

  // 6. 位置增量
  s.pos_world[0] = packet.north_offset;
  s.pos_world[1] = packet.east_offset;

  // 7. 压力计深度
  manometer_z_ = packet.pressure_depth;

  // 8. 传感器状态 & 导航模式
  {
    auto ns = auv::system::system_context.nav_status_.get();
    ns.imu_state = packet.navigation_mode;
    ns.dvl_state = (packet.sensor_flags & 0x02) ? 1 : 0;
    ns.timestamp = HAL_GetTick();
    auv::system::system_context.nav_status_.set(ns);
  }

  auv::system::g_app_ctx.watchdog->feed(
      auv::component::SoftWatchdog::Component::INS);

  const float kDeg2Rad = 0.0174532925f;
  s.pos_world[3] *= kDeg2Rad;
  s.pos_world[4] *= kDeg2Rad;
  s.pos_world[5] *= kDeg2Rad;
  s.vel_body[3] *= kDeg2Rad;
  s.vel_body[4] *= kDeg2Rad;
  s.vel_body[5] *= kDeg2Rad;

  state_ = s;
}

} // namespace peripheral
} // namespace auv
