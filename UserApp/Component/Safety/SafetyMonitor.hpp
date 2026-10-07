#ifndef __SAFETY_MONITOR_HPP
#define __SAFETY_MONITOR_HPP

#include "AppContext.hpp"
#include <stdint.h>

namespace auv {
namespace component {

/**
 * @class SafetyMonitor
 * @brief 独立的安全监控模块
 *
 * 职责：
 * - 心跳超时检测 → 自动上锁 (Disarm)
 * - 显式原点已设置 + 心跳计数/持续时间达标 + 导航就绪 → 解锁 (Arm)
 * - 状态事件日志
 *
 * ARM/DISARM 不修改本次上电的 odom 原点。
 */
class SafetyMonitor {
public:
  SafetyMonitor(auv::system::AppContext *ctx) : ctx_(ctx) {}

  /**
   * @brief 每控制周期调用一次，执行解锁/上锁状态机
   * @param now_ms 当前系统毫秒时间 (HAL_GetTick())
   */
  void check(uint32_t now_ms);

private:
  auv::system::AppContext *ctx_;

  // ---------- 时间常量 ----------
  // BasicMotion 默认 15Hz；保持现有 1 秒超时机制（兼容 2Hz 心跳）。
  static constexpr uint32_t kArmedHeartbeatTimeoutMs = 1000;
  static constexpr uint32_t kDisarmedHeartbeatTimeoutMs = 1000;
  static constexpr uint32_t kArmMinDurationMs = 1000;
  static constexpr uint32_t kArmMinHeartbeatCount = 10;
  static constexpr uint32_t kRemoteModeHeartbeatData = 3;

  // ---------- 状态节流 ----------
  uint32_t last_warn_denied_ms_ = 0;

  // ---------- 内部逻辑 ----------
  bool isArmingConditionsMet(uint32_t now_ms, uint32_t arm_start_ms,
                             uint32_t hbt_count) const;
  void executeArm(uint32_t now_ms);
  void forceDisarmWithNeutralLevel(const char *reason);
  void setControlLevelNone();
};

} // namespace component
} // namespace auv

#endif // __SAFETY_MONITOR_HPP
