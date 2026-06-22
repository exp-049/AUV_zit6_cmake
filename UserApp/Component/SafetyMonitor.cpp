#include "SafetyMonitor.hpp"
#include "FreeRTOS.h"
#include "MotionContext.hpp"
#include "RosLogger.hpp"
#include "SystemConfig.hpp"
#include "SystemContext.hpp"
#include "task.h"

namespace auv {
namespace component {

// ============================================================================
// 主入口：每控制周期调用一次
// ============================================================================

void SafetyMonitor::check(uint32_t now_ms) {
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

  // 分支 C：检查解锁条件是否满足
  if (isArmingConditionsMet(now_ms, a.start_ms, a.heartbeat_count)) {
    bool sim_mode = auv::config::sys_config.simulation.hitl_enabled ||
                    auv::config::sys_config.simulation.sitl_enabled;
    bool nav_ok = auv::system::system_context.getNavigationValid() || sim_mode;
    bool can_arm_flag = (a.last_heartbeat_data == kRemoteModeHeartbeatData) ||
                        (a.last_heartbeat_data == 1 && nav_ok);

    if (can_arm_flag) {
      executeArm();
    } else {
      if (a.last_heartbeat_data == 1 && !nav_ok) {
        if (now_ms - last_warn_denied_ms_ > 2000) {
          last_warn_denied_ms_ = now_ms;
          ROS_LOG_WARN("Arm denied - Navigation NOT valid");
        }
      }
      a.heartbeat_count = 0;
      auv::system::system_context.arm_state_.set(a);
    }
  }

  // 分支 D：长时间未收到心跳 → 清零计数
  if (now_ms - a.last_heartbeat_ms > kDisarmedHeartbeatTimeoutMs) {
    a.heartbeat_count = 0;
    auv::system::system_context.arm_state_.set(a);
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

void SafetyMonitor::executeArm() {
  auto nav_state = auv::motion::motion_context.nav_state_.get();

  taskENTER_CRITICAL();

  {
    auv::algorithm::math::Vector6f home_offset;
    home_offset << nav_state.pos_world[0], // X
        nav_state.pos_world[1],            // Y
        nav_state.pos_world[2],            // Z
        0.0f,                              // Roll 强制为 0
        0.0f,                              // Pitch 强制为 0
        nav_state.pos_world[5];            // Yaw 正常记录
    auv::motion::motion_context.setHomeOffset(home_offset);
  }

  auv::motion::motion_context.current_setpoint_.set(
      auv::motion::TargetSetpoint{});
  {
    auto a = auv::system::system_context.arm_state_.get();
    a.is_armed = true;
    auv::system::system_context.arm_state_.set(a);
  }

  taskEXIT_CRITICAL();

  ROS_LOG_INFO("System ARMED");
}

// ============================================================================
// 强制上锁
// ============================================================================

void SafetyMonitor::forceDisarmWithNeutralLevel(const char *reason) {
  {
    auto a = auv::system::system_context.arm_state_.get();
    a.is_armed = false;
    a.heartbeat_count = 0;
    auv::system::system_context.arm_state_.set(a);
  }
  auv::motion::motion_context.clearHomeOffset();

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
