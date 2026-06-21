#ifndef __SAFETY_MONITOR_HPP
#define __SAFETY_MONITOR_HPP

#include <stdint.h>

namespace auv {
namespace system {

/**
 * @class SafetyMonitor
 * @brief 独立的安全监控模块
 *
 * 职责：
 * - 心跳超时检测 → 自动上锁 (Disarm)
 * - 心跳计数达标 + 导航就绪 → 自动解锁 (Arm)
 * - 状态事件日志
 *
 * 与原 ControlTask::handleArmState() 行为完全一致，
 * 只是抽离为独立模块，便于测试和职责分离。
 */
class SafetyMonitor {
public:
  SafetyMonitor() = default;

  /**
   * @brief 每控制周期调用一次，执行解锁/上锁状态机
   * @param now_ms 当前系统毫秒时间 (HAL_GetTick())
   */
  void check(uint32_t now_ms);

private:
  // ---------- 时间常量 ----------
  static constexpr uint32_t kArmedHeartbeatTimeoutMs = 500;
  static constexpr uint32_t kDisarmedHeartbeatTimeoutMs = 1000;
  static constexpr uint32_t kArmMinDurationMs = 1000;
  static constexpr uint32_t kArmMinHeartbeatCount = 10;
  static constexpr uint32_t kRemoteModeHeartbeatData = 3;

  // ---------- 状态节流 ----------
  uint32_t last_warn_denied_ms_ = 0;

  // ---------- 内部逻辑 ----------
  bool isArmingConditionsMet(uint32_t now_ms, uint32_t arm_start_ms,
                             uint32_t hbt_count) const;
  void executeArm();
  void forceDisarmWithNeutralLevel(const char *reason);
  void setControlLevelNone();
};

} // namespace system
} // namespace auv

#endif // __SAFETY_MONITOR_HPP
