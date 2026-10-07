#include "SystemContext.hpp"
#include "AppContext.hpp"
#include "SystemConfig.hpp"
#include "MotionContext.hpp"
#include "main.h"

namespace auv {
namespace system {

SystemContext system_context{};

bool SystemContext::getNavigationValid() const {
  const auto odom = auv::motion::motion_context.getOdomSnapshot();
  return odom.nav_valid && HAL_GetTick() - odom.nav_timestamp_ms <= 200U;
}

void SystemContext::resetArmingQualification() {
  taskENTER_CRITICAL();
  arm_state_.unsafe().heartbeat_count = 0;
  arm_state_.unsafe().start_ms = 0;
  taskEXIT_CRITICAL();
}

bool SystemContext::resetArmingQualificationIfUnchanged(
    const ArmState &expected) {
  taskENTER_CRITICAL();
  auto &current = arm_state_.unsafe();
  const bool unchanged = current.is_armed == expected.is_armed &&
      current.heartbeat_count == expected.heartbeat_count &&
      current.last_heartbeat_ms == expected.last_heartbeat_ms &&
      current.last_heartbeat_data == expected.last_heartbeat_data &&
      current.start_ms == expected.start_ms;
  if (unchanged && !current.is_armed) {
    current.heartbeat_count = 0;
    current.start_ms = 0;
  }
  taskEXIT_CRITICAL();
  return unchanged;
}

} // namespace system
} // namespace auv
