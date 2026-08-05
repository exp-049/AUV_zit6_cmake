#ifndef __SYSTEM_CONTEXT_HPP
#define __SYSTEM_CONTEXT_HPP

#include "LockedField.hpp"
#include <stdint.h>

// extern 对象声明需要完整类型
#include "ChassisManager.hpp"
#include "INS_Driver.hpp"
#include "MotionController_Driver.hpp"
#include "USBL_Driver.hpp"

namespace auv::peripheral {
class Depth_Sensor_Driver;
class Pushrod_Driver;
} // namespace auv::peripheral

// --- 底层驱动实例（通过 AppContext 访问，此处为定义提供 extern）---
namespace auv::peripheral {
extern INS_Driver ins_driver;
extern MotionController_Driver motor_driver;
extern Depth_Sensor_Driver *depth_sensor;
extern Pushrod_Driver *pushrod_driver;
extern USBL_Driver usbl_driver;
} // namespace auv::peripheral
namespace auv::component {
extern ChassisManager chassis;
}

namespace auv {
namespace system {

/**
 * @struct NavStatus
 * @brief 提取自原 NavState 的非实时状态标志和低频定位信息
 */
struct NavStatus {
  uint8_t imu_state = 0;  ///< 惯导模式
  uint8_t dvl_state = 0;  ///< DVL有效性标志
  double lat = 0.0;       ///< 纬度 (deg)
  double lon = 0.0;       ///< 经度 (deg)
  uint32_t timestamp = 0; ///< 系统毫秒时间戳
};

/**
 * @struct ArmState
 * @brief 解锁状态与心跳监测（多字段原子整体读取）
 */
struct ArmState {
  bool is_armed = false;
  uint32_t heartbeat_count = 0;
  uint32_t last_heartbeat_ms = 0;
  uint32_t last_heartbeat_data = 0;
  uint32_t start_ms = 0;
};

/**
 * @class SystemContext
 * @brief 系统/状态机上下文（安全解锁状态、心跳监测、规划器控制标志等）
 */
class SystemContext {
public:
  LockedField<ArmState> arm_state_{};
  LockedField<NavStatus> nav_status_{};

  // 规划器启用与状态变量
  bool is_planner_active = false;
  volatile bool planner_replan_flag = false;

  // 校验导航数据是否有效
  bool getNavigationValid() const;
};

extern SystemContext system_context;

} // namespace system
} // namespace auv

#endif // __SYSTEM_CONTEXT_HPP
