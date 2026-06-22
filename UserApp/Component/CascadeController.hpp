#pragma once

#include "KinematicProfile.hpp"
#include "MotionContext.hpp"
#include "PID_Controller.hpp"
#include "SystemConfig.hpp"
#include <array>

namespace auv {
namespace component {

using auv::algorithm::control::KinematicProfile;
using auv::algorithm::control::PID_Controller;
using auv::algorithm::control::ProfileState;

/**
 * @class CascadeController
 * @brief 级联控制器
 */
class CascadeController {
public:
  CascadeController();
  CascadeController(const auv::config::ChassisConfig &cfg);

  /**
   * @brief 应用外部配置（可用于运行时切换参数）
   */
  void applyConfig(const auv::config::ChassisConfig &cfg);

  /**
   * @brief 获取当前控制层级
   */
  auv::motion::ControlLevel getControlLevel() const;

  /**
   * @brief 执行 100Hz 级联控制演进 (固定 dt=0.01s)
   *
   * dt 固定为 0.01s，信任 ControlTask 通过 vTaskDelayUntil 保证严格 10ms 周期。
   * 不使用 HAL_GetTick() 实时计算 dt，避免与硬编码值不一致。
   *
   * @return std::array<float, 6> 6-DOF 归一化力/力矩矢量
   */
  std::array<float, 6> update();

  /**
   * @brief 切换控制层级 (Bumpless Transition)
   * @param new_level 目标控制层级
   */
  void setControlLevel(auv::motion::ControlLevel new_level);

  /**
   * @brief 配置指定轴的 PID 参数
   */
  void configurePID(int axis, bool is_pos_ring, float kp, float ki, float kd,
                    float i_limit, float out_limit);

  /**
   * @brief 获取指定轴的 PID 配置
   */
  PID_Controller::Config getPIDConfig(int axis, bool is_pos_ring) const;

  /**
   * @brief 获取指定轴的运动学约束
   */
  void getProfileLimits(int axis, float &max_v, float &max_a) const;

  /**
   * @brief 配置指定轴的运动学约束
   */
  void configureProfile(int axis, float max_v, float max_a);

private:
  auv::motion::ControlLevel level_ = auv::motion::ControlLevel::NONE;

  std::array<KinematicProfile, 6> profiles_; ///< 6轴影子平滑器矩阵
  std::array<PID_Controller, 6> pos_pids_;   ///< 6个位置环 PID
  std::array<PID_Controller, 6> vel_pids_;   ///< 6个速度环 PID

  std::array<float, 6> last_output_forces_ = {0}; ///< 6轴输出快照
  float last_z_thrust_ = 0.0f; ///< 记录上周期的 Z 轴推力，用于 Trim Pre-loading
  std::array<float, 6> last_v_body_ = {
      0}; ///< 上周期机体系实际速度，用于计算加速度（用于 D 项）
  auv::config::ChassisConfig config_; ///< 当前应用的底盘参数
};

} // namespace component
} // namespace auv
