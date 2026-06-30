#ifndef __MOTION_CONTEXT_HPP
#define __MOTION_CONTEXT_HPP

#include "FreeRTOS.h"
#include "LockedField.hpp"
#include "MathUtils.hpp"
#include "RosLogger.hpp"
#include "task.h"
#include <array>
#include <cmath>
#include <stdint.h>

/* queue.h 提供 QueueHandle_t，已在 FreeRTOS 源码树中 */
#include "queue.h"

namespace auv {
namespace motion {

/**
 * @enum ControlLevel
 * @brief 控制层级枚举
 */
enum class ControlLevel : uint8_t {
  NONE = 0,     ///< 待机/锁定模式
  POSITION = 1, ///< 位置闭环
  VELOCITY = 2, ///< 速度闭环
  ACTUATOR = 3  ///< 直接推力控制
};

/**
 * @struct NavState
 * @brief 6-DOF 导航状态结构体
 *
 * 索引约定（与 MathUtils::Axis 一致）：
 *   [0]=X, [1]=Y, [2]=Z, [3]=Roll, [4]=Pitch, [5]=Yaw
 *
 * 世界系坐标采用 NED (North-East-Down) 惯例。
 * 机体系坐标采用 FRD (Front-Right-Down) 惯例。
 */
struct NavState {
  std::array<float, 6> pos_world = {0.0f, 0.0f, 0.0f, 0.0f,
                                    0.0f, 0.0f}; ///< NED: [x, y, z, φ, θ, ψ]
  std::array<float, 6> vel_body = {0.0f, 0.0f, 0.0f, 0.0f,
                                   0.0f, 0.0f}; ///< FRD: [u, v, w, p, q, r]
};

/**
 * @struct TargetSetpoint
 * @brief 管理级联控制中各层级的目标设定值（6DOF）
 */
struct TargetSetpoint {
  std::array<float, 6> pos_world = {
      0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; ///< 世界系目标 [x, y, z, φ, θ, ψ]
  std::array<float, 6> vel_body = {
      0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; ///< 机体系速度 [u, v, w, p, q, r]
  std::array<float, 6> thrust_body = {
      0.0f, 0.0f, 0.0f,
      0.0f, 0.0f, 0.0f}; ///< 机体系推力 [Fx, Fy, Fz, Mk, Mm, Mn]
};

/**
 * @struct OffboardSetpoint
 * @brief 上位机原始指令结构体
 */
struct OffboardSetpoint {
  ControlLevel level;
  float data[4];
  uint32_t type_mask;
};

/**
 * @struct Constants
 * @brief 系统数学与控制常数
 */
struct HomeOffset {
  bool active = false;
  std::array<float, 6> offset = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
};

struct Constants {
  static constexpr float CONTROL_FREQ = 100.0f;     ///< 控制频率 (Hz)
  static constexpr uint32_t CONTROL_PERIOD_MS = 10; ///< 控制周期 (ms)
  static constexpr float DEG2RAD = 0.0174532925f;   ///< 角度转弧度
  static constexpr float RAD2DEG = 57.2957795f;     ///< 弧度转角度
};

class MotionContext {
public:
  static float wrapAngle(float angle);

  // 线程安全字段（LockedField::get/set 自动加临界区）
  LockedField<NavState> nav_state_{};
  LockedField<TargetSetpoint> current_setpoint_{};
  LockedField<float> last_dt_ms_{0.0f};
  LockedField<uint32_t> last_received_seq_{0};
  LockedField<std::array<float, 6>> last_output_forces_{};

  /** SITL 仿真导航数据队列（生产者：onSimNav，消费者：ControlTask）
   *  长度 3，xQueueSend 非阻塞入队，消费端 drain 到最新 */
  QueueHandle_t sitl_nav_queue = nullptr;

  LockedField<HomeOffset> home_offset_{};

  void setHomeOffset(const auv::algorithm::math::Vector6f &offset);
  void clearHomeOffset();
};

extern MotionContext motion_context;

} // namespace motion
} // namespace auv

#endif // __MOTION_CONTEXT_HPP
