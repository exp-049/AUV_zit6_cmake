/**
 * @file INS_Driver.hpp
 * @brief NAV-300 集成惯导驱动类
 */

#ifndef __INS_DRIVER_HPP
#define __INS_DRIVER_HPP

#include "MotionContext.hpp"
#include <cstdint>
#include <cstring>

namespace auv {
namespace peripheral {

/**
 * @struct InsPortOps
 * @brief INS 惯导硬件操作接口（函数指针表）
 */
struct InsPortOps {
  void *ctx;
  bool (*init)(void *ctx);
  uint16_t (*read)(void *ctx, uint8_t *buf, uint16_t max_len);
  bool (*transmit)(void *ctx, const uint8_t *data, uint16_t len);
};

/**
 * @class INS_Driver
 * @brief 惯导驱动实现类
 */
class INS_Driver {
public:
  INS_Driver(InsPortOps ops) : ops_(ops) {}

  void init();
  bool update(auv::motion::NavState &state);
  auv::motion::NavState getNavState() const { return state_; }
  float getManometerZ() const { return manometer_z_; }
  bool isDataFresh() const;

  void sendCommand(uint8_t cmd_id, uint8_t value);
  void sendCommand(uint8_t cmd_id, const uint8_t *data, uint8_t data_len);
  void resetPosition();
  void setDvlPower(bool on);
  void restart();
  void setInitialPosition(double lat, double lon);

private:
  InsPortOps ops_;
  uint32_t rx_total_bytes_ = 0; ///< 累计接收字节数（调试统计）
  uint32_t last_update_ms_ = 0; ///< 上次收到有效包的时间

  static constexpr uint16_t kMaxFrameSize = 256;
  static constexpr uint16_t kMinFrameSize = 133;
  /// 帧解析临时缓冲区（4 字节对齐，确保 memmove/memcpy 使用 32 位总线）
  __attribute__((aligned(4))) uint8_t packet_buf_[kMaxFrameSize] = {0};
  uint16_t frame_len_ = 0; ///< 当前解析长度

  auv::motion::NavState state_{};      ///< 缓存的最新有效位姿状态
  auv::motion::NavState prev_state_{}; ///< 上一帧状态，用于速度差分估计
  bool has_prev_state_ = false;

  // 压力计深度 (m)
  float manometer_z_ = 0.0f;

  // 内部私有方法
  uint8_t checkData(const uint8_t *data, uint8_t size);
  bool parseByte(uint8_t b);
  bool validateFrame();
  void decodePacket(auv::motion::NavState &state);
};

} // namespace peripheral
} // namespace auv

#endif
