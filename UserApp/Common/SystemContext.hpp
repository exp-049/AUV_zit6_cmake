#ifndef __SYSTEM_CONTEXT_HPP
#define __SYSTEM_CONTEXT_HPP

#include "LockedField.hpp"
#include <stdint.h>

// 注:此处不再声明底层驱动 extern。全局驱动实例由 AppContext.cpp 定义并注入
// g_app_ctx(依赖注入容器);各处通过 g_app_ctx.<driver> 指针访问,不依赖
// SystemContext.hpp 暴露这些全局。此举消除了 INS_Driver.hpp → MotionContext.hpp
// → FreeRTOS 的传递平台依赖(与 AppContext.hpp 的前向声明风格一致)。

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
