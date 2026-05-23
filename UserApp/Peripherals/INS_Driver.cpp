#include "INS_Driver.hpp"
#include "SystemContext.hpp"
#include "SoftWatchdog.hpp"
#include <cmath>

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
  return (HAL_GetTick() - last_update_ms_ < 200);
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

bool INS_Driver::parseByte(uint8_t b) {
  if (frame_len_ == 0 && b != 0xFA)
    return false;
  if (frame_len_ == 1 && b != 0xAF) {
    frame_len_ = 0;
    return false;
  }

  if (frame_len_ < kMaxFrameSize) {
    packet_buf_[frame_len_++] = b;
  } else {
    frame_len_ = 0;
    return false;
  }

  // UNAV-IP 133字节标准帧 (根据截图确认)
  return frame_len_ == 133;
}

bool INS_Driver::validateFrame() {
  if (frame_len_ != 133) {
    frame_len_ = 0;
    return false;
  }

  // 异或校验位在 130，校验范围 0-129
  uint8_t v = 0;
  for (int i = 0; i < 130; i++) {
    v ^= packet_buf_[i];
  }

  if (v == packet_buf_[130]) {
    // 检查帧尾 0xFB 0xBF
    if (packet_buf_[131] == 0xFB && packet_buf_[132] == 0xBF) {
      return true;
    }
  }

  frame_len_ = 0;
  return false;
}

void INS_Driver::decodePacket(auv::motion::NavState &s) {
  // 严格按照截图偏移量解析

  // 1. 姿态 (Offset 2, 6, 10)
  memcpy(&s.roll, packet_buf_ + 2, 4);
  memcpy(&s.pitch, packet_buf_ + 6, 4);
  memcpy(&s.pos_world[3], packet_buf_ + 10, 4);

  // 2. 角速度 (Offset 14, 18, 22)
  memcpy(&s.vroll, packet_buf_ + 14, 4);
  memcpy(&s.vpitch, packet_buf_ + 18, 4);
  memcpy(&s.vel_body[3], packet_buf_ + 22, 4);

  // 3. 机体系线速度 (Offset 26, 30, 34) -> vx, vy, vz
  memcpy(&s.vel_body[0], packet_buf_ + 26, 4);
  memcpy(&s.vel_body[1], packet_buf_ + 30, 4);
  memcpy(&s.vel_body[2], packet_buf_ + 34, 4);

  // 4. 经纬度 (Offset 38, 42, int32, 1e7)
  int32_t lat_i, lon_i;
  memcpy(&lat_i, packet_buf_ + 38, 4);
  memcpy(&lon_i, packet_buf_ + 42, 4);
  auv::system::system_context.nav_status.lat = lat_i * 1e-7;
  auv::system::system_context.nav_status.lon = lon_i * 1e-7;

  // 5. 深度 (改为 Offset 46: 组合深度) -> z
  memcpy(&s.pos_world[2], packet_buf_ + 46, 4);

  // 6. 位置增量 (Offset 99, 103, float) -> x, y
  memcpy(&s.pos_world[0], packet_buf_ + 99, 4);
  memcpy(&s.pos_world[1], packet_buf_ + 103, 4);

  // 6.5 压力计深度 (Offset 107, float, m)
  float manometer_z = 0.0f;
  memcpy(&manometer_z, packet_buf_ + 107, 4);

  // 7. 模式与状态 (Offset 129 模式, Offset 115 状态)
  auv::system::system_context.nav_status.imu_state = packet_buf_[129];
  auv::system::system_context.nav_status.dvl_state = (packet_buf_[115] & 0x02) ? 1 : 0;
  auv::system::system_context.nav_status.timestamp = HAL_GetTick();

  auv::device::SoftWatchdog::getInstance().feed(
      auv::device::SoftWatchdog::Component::INS);

  // 转换单位 (Deg -> Rad)
  const float kDeg2Rad = 0.0174532925f;
  s.roll *= kDeg2Rad;
  s.pitch *= kDeg2Rad;
  s.pos_world[3] *= kDeg2Rad;
  s.vroll *= kDeg2Rad;
  s.vpitch *= kDeg2Rad;
  s.vel_body[3] *= kDeg2Rad;

  // 更新内部缓存（保存为弧度制状态）
  state_ = s;

  manometer_z_ = manometer_z;

  frame_len_ = 0;
}

} // namespace device
} // namespace auv
