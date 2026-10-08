#include "SafetyMonitor.hpp"
#include "FreeRTOS.h"
#include "MotionContext.hpp"
#include "RosLogger.hpp"
#include "SystemConfig.hpp"
#include "SystemContext.hpp"
#include "ChassisManager.hpp"
#include "task.h"
#include "main.h"

namespace auv {
namespace component {

// ============================================================================
// 主入口：每控制周期调用一次
// ============================================================================

void SafetyMonitor::check(uint32_t now_ms) {
  // Use the live tick: the monitor task can run late behind higher priority
  // work, and a caller's captured tick may no longer describe a fresh lease.
  now_ms = HAL_GetTick();
  // 1. 原子读取系统上下文快照
  auto a = auv::system::system_context.arm_state_.get();

  // 分支 A：已解锁 → 检查心跳超时
  if (a.is_armed) {
    if (now_ms - a.last_heartbeat_ms > kArmedHeartbeatTimeoutMs) {
      forceDisarmWithNeutralLevel("Heartbeat timeout");
    }
    return;
  }

  // 分支 B：未解锁 → 确保控制层级归零
  if (ctx_->chassis->getControlLevel() != auv::motion::ControlLevel::NONE) {
    setControlLevelNone();
  }

  // Expired heartbeats cannot trigger a new ARM even if count/duration qualify.
  if (now_ms - a.last_heartbeat_ms > kDisarmedHeartbeatTimeoutMs) {
    auv::system::system_context.resetArmingQualificationIfUnchanged(a);
    return;
  }

  // 分支 C：检查解锁条件是否满足
  if (isArmingConditionsMet(now_ms, a.start_ms, a.heartbeat_count)) {
    const bool direct_thrust_mode =
        a.last_heartbeat_data == kRemoteModeHeartbeatData;
    const bool origin_ok =
        auv::motion::motion_context.getOdomSnapshot().origin_initialized;
    const bool nav_ok = auv::system::system_context.getNavigationValid();
    const bool navigation_arm_mode =
        a.last_heartbeat_data == 1 && origin_ok && nav_ok;
    const bool can_arm_flag = direct_thrust_mode || navigation_arm_mode;

    if (can_arm_flag) {
      executeArm(now_ms);
    } else {
      if (a.last_heartbeat_data == 1 && (!origin_ok || !nav_ok)) {
        if (now_ms - last_warn_denied_ms_ > 2000) {
          last_warn_denied_ms_ = now_ms;
          ROS_LOG_WARN("Arm denied - origin not set or navigation invalid");
        }
      }
      auv::system::system_context.resetArmingQualificationIfUnchanged(a);
    }
  }


}

bool SafetyMonitor::isArmingConditionsMet(uint32_t now_ms,
                                          uint32_t arm_start_ms,
                                          uint32_t hbt_count) const {
  return (hbt_count >= kArmMinHeartbeatCount &&
          (now_ms - arm_start_ms >= kArmMinDurationMs));
}

// ============================================================================
// 执行解锁
// ============================================================================

void SafetyMonitor::executeArm(uint32_t now_ms) {
  taskENTER_CRITICAL();
  // Revalidate the latest qualification under the SAME lock as setorigin.
  // A service may have reset the counters after check() read its initial copy.
  auto &a = auv::system::system_context.arm_state_.unsafe();
  const bool direct_thrust_mode =
      a.last_heartbeat_data == kRemoteModeHeartbeatData;
  const bool origin_ok =
      auv::motion::motion_context.getOdomSnapshot().origin_initialized;
  const bool navigation_arm_mode =
      a.last_heartbeat_data == 1 && origin_ok &&
      auv::system::system_context.getNavigationValid();
  const bool arm_mode_ok = direct_thrust_mode || navigation_arm_mode;
  const uint32_t current_ms = HAL_GetTick();
  if (!arm_mode_ok ||
      !isArmingConditionsMet(current_ms, a.start_ms, a.heartbeat_count) ||
      current_ms - a.last_heartbeat_ms > kDisarmedHeartbeatTimeoutMs) {
    taskEXIT_CRITICAL();
    return;
  }

  auv::motion::motion_context.current_setpoint_.set(
      auv::motion::TargetSetpoint{});
  a.is_armed = true;

  taskEXIT_CRITICAL();

  ROS_LOG_INFO("System ARMED");
}

// ============================================================================
// 强制上锁
// ============================================================================

void SafetyMonitor::forceDisarmWithNeutralLevel(const char *reason) {
  bool disarmed = false;
  taskENTER_CRITICAL();
  auto &a = auv::system::system_context.arm_state_.unsafe();
  if (a.is_armed &&
      HAL_GetTick() - a.last_heartbeat_ms > kArmedHeartbeatTimeoutMs) {
    a.is_armed = false;
    a.heartbeat_count = 0;
    a.start_ms = 0;
    disarmed = true;
  }
  taskEXIT_CRITICAL();
  if (!disarmed)
    return;
  // Disarming retains this boot's origin and generation.

  setControlLevelNone();

  ROS_LOG_INFO("System DISARMED - %s", reason);
}

// ============================================================================
// 控制层级复位
// ============================================================================

void SafetyMonitor::setControlLevelNone() {
  ctx_->chassis->setControlLevel(auv::motion::ControlLevel::NONE);
}

} // namespace component
} // namespace auv
