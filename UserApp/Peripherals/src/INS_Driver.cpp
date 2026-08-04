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
  uint8_t cmd[auv::protocol::ins::Parser::commandFrameSize()] = {};
  if (ops_.transmit == nullptr ||
      auv::protocol::ins::Parser::encodeCommand(
          cmd_id, data, data_len, cmd, sizeof(cmd)) == 0U) {
    return;
  }

  // 发送（尝试 3 次）
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
  auv::protocol::ins::Parser::encodeInitialPosition(lat, lon, data);
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
      auv::protocol::ins::NavigationPacket packet;
      if (protocol_parser_.decode(packet)) {
        decodePacket(packet, state);
        last_update_ms_ = HAL_GetTick(); // 刷新时间戳
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

void INS_Driver::decodePacket(
    const auv::protocol::ins::NavigationPacket &packet,
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
  s.pos_world[0] = packet.north_offset; // X (North)
  s.pos_world[1] = packet.east_offset; // Y (East)

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
}

} // namespace peripheral
} // namespace auv
