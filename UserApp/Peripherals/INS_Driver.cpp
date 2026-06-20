#include "INS_Driver.hpp"
#include "RosLogger.hpp"
#include "SoftWatchdog.hpp"
#include "SystemContext.hpp"
#include <cmath>
#include <cstdint>

namespace auv {
namespace device {

void INS_Driver::init() {
  // 尝试启动 DMA 接收，如果失败则重试（防止上电初期串口噪声导致 ORE 锁死）
  for (int i = 0; i < 5; i++) {
    if (rx_port_.startReceive())
      break;
    HAL_Delay(10);
  }
  resetPosition();
}

void INS_Driver::sendCommand(uint8_t cmd_id, uint8_t value) {
  uint8_t data[8] = {value, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendCommand(cmd_id, data, 8);
}

void INS_Driver::sendCommand(uint8_t cmd_id, const uint8_t *data,
                             uint8_t data_len) {
  uint8_t cmd[14] = {0xFC, 0xCF, cmd_id};

  // 拷贝负载数据（最多8字节）
  uint8_t copy_len = (data_len > 8) ? 8 : data_len;
  if (data != nullptr && copy_len > 0) {
    memcpy(&cmd[3], data, copy_len);
  }

  // 填充剩余字节为 0
  for (uint8_t i = 3 + copy_len; i < 11; ++i) {
    cmd[i] = 0x00;
  }

  // 帧尾
  cmd[12] = 0xFD;
  cmd[13] = 0xDF;

  // 计算 XOR 校验和 (从 byte 0 到 byte 10)
  uint8_t v = 0;
  for (uint8_t i = 0; i < 11; ++i) {
    v ^= cmd[i];
  }
  cmd[11] = v;

  // 发送到 tx_uart_ (尝试发送3次以确保可靠性)
  for (int i = 0; i < 3; i++) {
    HAL_UART_Transmit(tx_uart_, cmd, 14, 50);
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
  // 手册：纬度/经度 * 10^7, 强制转 int, 高位在前 (Big Endian)
  int32_t lat_fixed = (int32_t)(lat * 1e7);
  int32_t lon_fixed = (int32_t)(lon * 1e7);

  uint8_t data[8];
  // 纬度 (Big Endian)
  data[0] = (uint8_t)((lat_fixed >> 24) & 0xFF);
  data[1] = (uint8_t)((lat_fixed >> 16) & 0xFF);
  data[2] = (uint8_t)((lat_fixed >> 8) & 0xFF);
  data[3] = (uint8_t)(lat_fixed & 0xFF);

  // 经度 (Big Endian)
  data[4] = (uint8_t)((lon_fixed >> 24) & 0xFF);
  data[5] = (uint8_t)((lon_fixed >> 16) & 0xFF);
  data[6] = (uint8_t)((lon_fixed >> 8) & 0xFF);
  data[7] = (uint8_t)(lon_fixed & 0xFF);

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
  uint16_t len = rx_port_.read(temp_buf, 256);
  bool has_new_frame = false;

  for (int i = 0; i < len; i++) {
    if (parseByte(temp_buf[i])) {
      if (validateFrame()) {
        decodePacket(state);
        last_update_ms_ = HAL_GetTick(); // 刷新时间戳
        has_new_frame = true;
      }
    }
  }
  return has_new_frame;
}

namespace ProtoOff {
constexpr int kRoll = 2;
constexpr int kPitch = 6;
constexpr int kYaw = 10;
constexpr int kRollRate = 14;
constexpr int kPitchRate = 18;
constexpr int kYawRate = 22;
constexpr int kVx = 26;
constexpr int kVy = 30;
constexpr int kVz = 34;
constexpr int kLat = 38;
constexpr int kLon = 42;
constexpr int kDepth = 46;
constexpr int kNorthOffset = 99;
constexpr int kEastOffset = 103;
constexpr int kManometerDepth = 107;
constexpr int kSensorFlags = 115;
constexpr int kNavMode = 129;
constexpr int kChecksum = 130;
constexpr int kTail1 = 131;
constexpr int kTail2 = 132;
constexpr int kFrameSize = 133;
} // namespace ProtoOff

bool INS_Driver::parseByte(uint8_t b) {
  // 状态 0：等待帧头 0xFA
  if (frame_len_ == 0) {
    if (b == 0xFA) {
      packet_buf_[frame_len_++] = b;
    }
    // 非 0xFA 直接丢弃（不上涨 frame_len_）
    return false;
  }

  // 状态 1：等待第二字节 0xAF
  if (frame_len_ == 1) {
    if (b == 0xAF) {
      packet_buf_[frame_len_++] = b;
      return false;
    }
    // 不是 0xAF：滑动窗口退回
    // 如果当前 b 恰好是 0xFA，则以它为新的帧头起始
    frame_len_ = 0;
    if (b == 0xFA) {
      packet_buf_[frame_len_++] = b;
    }
    return false;
  }

  // 状态 2+：收集数据负载
  if (frame_len_ < kMaxFrameSize) {
    packet_buf_[frame_len_++] = b;
  } else {
    // 缓冲区溢出，复位
    frame_len_ = 0;
    return false;
  }

  // UNAV-IP 标准帧长
  return frame_len_ == ProtoOff::kFrameSize;
}

bool INS_Driver::validateFrame() {
  using namespace ProtoOff;
  if (frame_len_ != kFrameSize) {
    frame_len_ = 0;
    return false;
  }

  // 异或校验位在 kChecksum，校验范围 0 ～ kChecksum-1
  uint8_t v = 0;
  for (int i = 0; i < kChecksum; i++) {
    v ^= packet_buf_[i];
  }

  if (v == packet_buf_[kChecksum] && packet_buf_[kTail1] == 0xFB &&
      packet_buf_[kTail2] == 0xBF) {
    return true;
  }

  // 校验失败：滑动窗口重同步
  // 从第 2 字节开始搜索 0xFA，避免丢弃下一帧的同步头
  uint16_t search_pos = 2; // 跳过已确认的 0xFA 0xAF
  while (search_pos < frame_len_) {
    if (packet_buf_[search_pos] == 0xFA) {
      // 发现潜在的新帧头，将后续字节移到缓冲区起始
      uint16_t remaining = frame_len_ - search_pos;
      memmove(packet_buf_, packet_buf_ + search_pos, remaining);
      frame_len_ = remaining;
      return false;
    }
    search_pos++;
  }

  // 未找到新的 0xFA，完全复位
  frame_len_ = 0;
  return false;
}

// ============================================================================
// 字节序安全读取工具（noexcept 消除异常处理桩，利于 STM32 编译器内联）
// ============================================================================

/// 从小端字节序读取 int32_t
static inline int32_t readLE32(const uint8_t *buf) noexcept {
  return (int32_t)buf[0] | ((int32_t)buf[1] << 8) | ((int32_t)buf[2] << 16) |
         ((int32_t)buf[3] << 24);
}

/// 从小端字节序读取 float
static inline float readLEFloat(const uint8_t *buf) noexcept {
  int32_t bits = readLE32(buf);
  float val;
  memcpy(&val, &bits, sizeof(val));
  return val;
}

/// 从大端字节序读取 int32_t（备用于命令响应帧解析）
static inline int32_t readBE32(const uint8_t *buf) noexcept {
  return ((int32_t)buf[0] << 24) | ((int32_t)buf[1] << 16) |
         ((int32_t)buf[2] << 8) | (int32_t)buf[3];
}

/// 从大端字节序读取 float（备用于命令响应帧解析）
static inline float readBEFloat(const uint8_t *buf) noexcept {
  int32_t bits = readBE32(buf);
  float val;
  memcpy(&val, &bits, sizeof(val));
  return val;
}

void INS_Driver::decodePacket(auv::motion::NavState &s) {
  using namespace ProtoOff;

  // 1. 姿态 → pos_world[ROLL=3, PITCH=4, YAW=5]
  s.pos_world[3] = readLEFloat(packet_buf_ + kRoll);  // Roll (φ)
  s.pos_world[4] = readLEFloat(packet_buf_ + kPitch); // Pitch (θ)
  s.pos_world[5] = readLEFloat(packet_buf_ + kYaw);   // Yaw (ψ)

  // 2. 角速度 → vel_body[ROLL=3, PITCH=4, YAW=5]
  s.vel_body[3] = readLEFloat(packet_buf_ + kRollRate);  // p
  s.vel_body[4] = readLEFloat(packet_buf_ + kPitchRate); // q
  s.vel_body[5] = readLEFloat(packet_buf_ + kYawRate);   // r

  // 3. 机体系线速度
  s.vel_body[0] = readLEFloat(packet_buf_ + kVx);
  s.vel_body[1] = readLEFloat(packet_buf_ + kVy);
  s.vel_body[2] = readLEFloat(packet_buf_ + kVz);

  // 4. 经纬度 (int32, 1e7 缩放)
  int32_t lat_i = readLE32(packet_buf_ + kLat);
  int32_t lon_i = readLE32(packet_buf_ + kLon);
  auv::system::system_context.nav_status.lat = lat_i * 1e-7;
  auv::system::system_context.nav_status.lon = lon_i * 1e-7;

  // 5. 深度
  s.pos_world[2] = readLEFloat(packet_buf_ + kDepth);

  // 6. 位置增量
  s.pos_world[0] = readLEFloat(packet_buf_ + kNorthOffset); // X (North)
  s.pos_world[1] = readLEFloat(packet_buf_ + kEastOffset);  // Y (East)

  // 7. 压力计深度
  manometer_z_ = readLEFloat(packet_buf_ + kManometerDepth);

  // 8. 传感器状态 & 导航模式
  auv::system::system_context.nav_status.imu_state = packet_buf_[kNavMode];
  auv::system::system_context.nav_status.dvl_state =
      (packet_buf_[kSensorFlags] & 0x02) ? 1 : 0;
  auv::system::system_context.nav_status.timestamp = HAL_GetTick();

  auv::device::SoftWatchdog::getInstance().feed(
      auv::device::SoftWatchdog::Component::INS);

  // 转换单位 (Deg -> Rad)
  const float kDeg2Rad = 0.0174532925f;
  s.pos_world[3] *= kDeg2Rad; // Roll
  s.pos_world[4] *= kDeg2Rad; // Pitch
  s.pos_world[5] *= kDeg2Rad; // Yaw
  s.vel_body[3] *= kDeg2Rad;  // p (Roll rate)
  s.vel_body[4] *= kDeg2Rad;  // q (Pitch rate)
  s.vel_body[5] *= kDeg2Rad;  // r (Yaw rate)

  // 更新内部缓存（保存为弧度制状态）
  state_ = s;

  frame_len_ = 0; // 确保状态机复位，准备下一帧
}

} // namespace device
} // namespace auv
