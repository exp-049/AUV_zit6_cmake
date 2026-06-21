#include "SafetyMonitor.hpp"
#include "MotionContext.hpp"
#include "SystemContext.hpp"
#include "SystemConfig.hpp"
#include "ChassisManager.hpp"
#include "RosLogger.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace auv::system;

// ============================================================================
// 主入口：每控制周期调用一次
// ============================================================================

void SafetyMonitor::check(uint32_t now_ms) {
  // 1. 原子读取系统上下文快照
  taskENTER_CRITICAL();
  const bool armed = auv::system::system_context.is_system_armed;
  const uint32_t hbt_time = auv::system::system_context.last_arm_heartbeat_ms;
  const uint32_t hbt_count = auv::system::system_context.arm_heartbeat_count;
  const uint32_t arm_start = auv::system::system_context.arm_start_ms;
  const uint32_t hbt_data =
      auv::system::system_context.last_arm_heartbeat_data;
  taskEXIT_CRITICAL();

  // ================================================================
  // 分支 A：已解锁 → 检查心跳超时
  // ================================================================
  if (armed) {
    if (now_ms - hbt_time > kArmedHeartbeatTimeoutMs) {
      forceDisarmWithNeutralLevel("Heartbeat timeout");
    }
    return;
  }

  // ================================================================
  // 分支 B：未解锁 → 确保控制层级归零
  // ================================================================
  if (auv::control::chassis.getControlLevel() !=
      auv::motion::ControlLevel::NONE) {
    setControlLevelNone();
  }

  // ================================================================
  // 分支 C：检查解锁条件是否满足
  // ================================================================
  if (isArmingConditionsMet(now_ms, arm_start, hbt_count)) {
    // 检查心跳数据是否允许解锁
    bool sim_mode = auv::config::sys_config.simulation.hitl_enabled ||
                    auv::config::sys_config.simulation.sitl_enabled;
    bool nav_ok = auv::system::system_context.getNavigationValid() || sim_mode;
    bool can_arm_flag =
        (hbt_data == kRemoteModeHeartbeatData) ||
        (hbt_data == 1 && nav_ok);

    if (can_arm_flag) {
      executeArm();
    } else {
      // 心跳数据是 1 但导航未就绪，打印告警（限频 2s）
      if (hbt_data == 1 && !nav_ok) {
        if (now_ms - last_warn_denied_ms_ > 2000) {
          last_warn_denied_ms_ = now_ms;
          ROS_LOG_WARN("Arm denied - Navigation NOT valid");
        }
      }
      // 清空心跳计数，避免无限累积后突然解锁
      taskENTER_CRITICAL();
      auv::system::system_context.arm_heartbeat_count = 0;
      taskEXIT_CRITICAL();
    }
  }

  // ================================================================
  // 分支 D：长时间未收到心跳 → 清零计数
  // ================================================================
  if (now_ms - hbt_time > kDisarmedHeartbeatTimeoutMs) {
    taskENTER_CRITICAL();
    auv::system::system_context.arm_heartbeat_count = 0;
    taskEXIT_CRITICAL();
  }
}

// ============================================================================
// 解锁条件判断
// ============================================================================

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
  auto nav_state = auv::motion::motion_context.getNavState();

  taskENTER_CRITICAL();

  // 设置 HomeOffset：Roll/Pitch 强制为 0
  {
    auv::math::Vector6f home_offset;
    home_offset << nav_state.pos_world[0], // X
        nav_state.pos_world[1],            // Y
        nav_state.pos_world[2],            // Z
        0.0f,                              // Roll 强制为 0
        0.0f,                              // Pitch 强制为 0
        nav_state.pos_world[5];            // Yaw 正常记录
    auv::motion::motion_context.setHomeOffset(home_offset);
  }

  // 锁定控制器目标为当前点（即新坐标系的 0 点）
  auv::motion::motion_context.resetSetpoint();
  auv::system::system_context.is_system_armed = true;

  taskEXIT_CRITICAL();

  ROS_LOG_INFO("System ARMED");
}

// ============================================================================
// 强制上锁
// ============================================================================

void SafetyMonitor::forceDisarmWithNeutralLevel(const char *reason) {
  taskENTER_CRITICAL();
  auv::system::system_context.is_system_armed = false;
  auv::system::system_context.arm_heartbeat_count = 0;
  auv::motion::motion_context.clearHomeOffset();
  taskEXIT_CRITICAL();

  setControlLevelNone();

  ROS_LOG_INFO("System DISARMED - %s", reason);
}

// ============================================================================
// 控制层级复位
// ============================================================================

void SafetyMonitor::setControlLevelNone() {
  auv::control::chassis.setControlLevel(auv::motion::ControlLevel::NONE);
}
