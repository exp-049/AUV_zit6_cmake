#include "ChassisManager.hpp"
#include "SystemConfig.hpp"
#include "main.h"
#include <algorithm>

namespace auv {
namespace control {

ChassisManager::ChassisManager() { applyConfig(auv::config::ChassisConfig()); }

ChassisManager::ChassisManager(const auv::config::ChassisConfig &cfg) {
  applyConfig(cfg);
}

void ChassisManager::applyConfig(const auv::config::ChassisConfig &cfg) {
  config_ = cfg;
  const auv::config::AxisConfig *axes[4] = {&cfg.x, &cfg.y, &cfg.z, &cfg.yaw};

  for (int i = 0; i < 4; i++) {
    const auto &axis_cfg = *axes[i];
    profiles_[i].setLimits(axis_cfg.max_v, axis_cfg.max_a);

    PID_Controller::Config pos_cfg;
    pos_cfg.kp = axis_cfg.pos_kp;
    pos_cfg.ki = axis_cfg.pos_ki;
    pos_cfg.kd = axis_cfg.pos_kd;
    pos_cfg.i_limit = axis_cfg.pos_i_limit;
    pos_cfg.output_limit = axis_cfg.pos_output_limit;
    pos_cfg.dt = 0.01f;            // 固定周期
    pos_pids_[i].setConfig(pos_cfg);

    PID_Controller::Config vel_cfg;
    vel_cfg.kp = axis_cfg.vel_kp;
    vel_cfg.ki = axis_cfg.vel_ki;
    vel_cfg.kd = axis_cfg.vel_kd;
    vel_cfg.i_limit = axis_cfg.vel_i_limit;
    vel_cfg.output_limit = axis_cfg.vel_output_limit;
    vel_cfg.dt = 0.01f;
    vel_pids_[i].setConfig(vel_cfg);
  }
}

auv::common::ControlLevel ChassisManager::getControlLevel() const {
  return level_;
}

PID_Controller::Config ChassisManager::getPIDConfig(int axis,
                                                    bool is_pos_ring) const {
  if (axis < 0 || axis >= 4)
    return {};
  return is_pos_ring ? pos_pids_[axis].getConfig()
                     : vel_pids_[axis].getConfig();
}

void ChassisManager::getProfileLimits(int axis, float &max_v,
                                      float &max_a) const {
  if (axis >= 0 && axis < 4) {
    max_v = profiles_[axis].getMaxV();
    max_a = profiles_[axis].getMaxA();
  }
}

void ChassisManager::configureProfile(int axis, float max_v, float max_a) {
  if (axis >= 0 && axis < 4) {
    float v = (max_v >= 0.0f) ? max_v : profiles_[axis].getMaxV();
    float a = (max_a >= 0.0f) ? max_a : profiles_[axis].getMaxA();
    profiles_[axis].setLimits(v, a);
  }
}

void ChassisManager::setActuatorForces(const float forces[4]) {
  for (int i = 0; i < 4; i++)
    target_forces_[i] = forces[i];
}

void ChassisManager::configurePID(int axis, bool is_pos_ring, float kp,
                                  float ki, float kd, float i_limit,
                                  float out_limit) {
  if (axis < 0 || axis >= 4)
    return;

  // 获取当前配置，用于增量修改
  PID_Controller::Config cfg = getPIDConfig(axis, is_pos_ring);

  if (kp >= 0.0f)
    cfg.kp = kp;
  if (ki >= 0.0f)
    cfg.ki = ki;
  if (kd >= 0.0f)
    cfg.kd = kd;
  if (i_limit >= 0.0f)
    cfg.i_limit = i_limit;
  if (out_limit >= 0.0f)
    cfg.output_limit = out_limit;

  cfg.dt = 0.01f; // 步长固定

  if (is_pos_ring)
    pos_pids_[axis].setConfig(cfg);
  else
    vel_pids_[axis].setConfig(cfg);
}

void ChassisManager::setControlLevel(auv::common::ControlLevel new_level,
                                     const float actual_p[4],
                                     const float actual_v[4]) {
  if (new_level == level_)
    return;
  // 切换到 POSITION：对齐影子状态并清除 PID 积分
  if (new_level == auv::common::ControlLevel::POSITION) {
    float actual_v_world[4];
    CoordinateManager::bodyToWorld(actual_p[3], actual_v[0], actual_v[1],
                                   actual_v_world[0], actual_v_world[1]);
    actual_v_world[2] = actual_v[2];
    actual_v_world[3] = actual_v[3];

    for (int i = 0; i < 4; i++) {
      profiles_[i].align(actual_p[i], actual_v_world[i]);
      pos_pids_[i].reset();
      vel_pids_[i].reset();
      target_forces_[i] = 0.0f; // 清除推力环残余，防止作为偏置叠加到位置环
    }
  }
  // 切换到 VELOCITY：对齐影子状态并清除速度环积分
  else if (new_level == auv::common::ControlLevel::VELOCITY) {
    float actual_v_world[4];
    CoordinateManager::bodyToWorld(actual_p[3], actual_v[0], actual_v[1],
                                   actual_v_world[0], actual_v_world[1]);
    actual_v_world[2] = actual_v[2];
    actual_v_world[3] = actual_v[3];

    for (int i = 0; i < 4; i++) {
      profiles_[i].align(actual_p[i], actual_v_world[i]);
      vel_pids_[i].reset();
      target_forces_[i] = 0.0f; // 清除推力环残余
    }
  }

  // 切换到 ACTUATOR：只改变层级，不清空 target_forces_
  else if (new_level == auv::common::ControlLevel::ACTUATOR) {
    // 保持 target_forces_ 不变，使外部直接写入的力能立即生效
  }

  // 切换到 NONE（安全停机）：清空目标推力
  else if (new_level == auv::common::ControlLevel::NONE) {
    target_forces_.fill(0.0f);
  }

  level_ = new_level;
}

std::array<float, 4> ChassisManager::update(const float actual_p[4],
                                            const float actual_v[4],
                                            const float target_p[4]) {
  std::array<float, 4> output_forces = {0};
  uint32_t now = HAL_GetTick();
  float dt = (last_update_tick_ == 0)
                 ? 0.01f
                 : (float)(now - last_update_tick_) / 1000.0f;
  last_update_tick_ = now;

  // 防止 dt 异常：限定在 [1ms, 100ms] 范围
  if (dt > 0.1f)
    dt = 0.1f;
  if (dt <= 0.0f)
    dt = 0.001f;

  if (level_ == auv::common::ControlLevel::NONE)
    return output_forces;

  // 计算世界系下的实际速度（用于位置环微分项）
  float actual_v_world_now[4];
  CoordinateManager::bodyToWorld(actual_p[3], actual_v[0], actual_v[1],
                                 actual_v_world_now[0], actual_v_world_now[1]);
  actual_v_world_now[2] = actual_v[2];
  actual_v_world_now[3] = actual_v[3];

  std::array<float, 4> v_target_world = {0};
  for (int i = 0; i < 4; i++) {
    if (level_ == auv::common::ControlLevel::POSITION) {
      ProfileState d;
      if (config_.planner_enabled) {
        if (i == 3) {
          d = profiles_[i].updateAngle(target_p[i], dt);
        } else {
          d = profiles_[i].update(target_p[i], dt);
        }
      } else {
        d.p = target_p[i];
        d.v = 0.0f;
        d.a = 0.0f;
      }
      
      // 修正：位置环的导数项应使用世界系下的速度误差 (v_ref_world -
      // v_actual_world)
      float actual_v_world_val = (i < 2) ? actual_v_world_now[i] : actual_v[i];
      float pos_derivative = d.v - actual_v_world_val;
      float pos_error = (i == 3) ? CoordinateManager::normalizeAngle(d.p - actual_p[i]) 
                                 : (d.p - actual_p[i]);
      v_target_world[i] =
          pos_pids_[i].compute(pos_error, dt, pos_derivative) + d.v;
    } else if (level_ == auv::common::ControlLevel::VELOCITY) {
      // 速度环规划器演进
      float target_v_world = 0.0f;
      if (target_v_is_body_) {
        // 如果是机体系目标，先转为世界系用于平滑器演进（保持位置参考点一致）
        float vx_w, vy_w;
        CoordinateManager::bodyToWorld(actual_p[3], target_p[0], target_p[1],
                                       vx_w, vy_w);
        target_v_world = (i == 0) ? vx_w : (i == 1 ? vy_w : target_p[i]);
      } else {
        target_v_world = target_p[i];
      }

      ProfileState d;
      if (config_.planner_enabled) {
        d = profiles_[i].updateVelocity(target_v_world, dt);
      } else {
        d.p = 0.0f; // Velocity mode doesn't use d.p
        d.v = target_v_world;
        d.a = 0.0f;
      }
      v_target_world[i] = d.v;
    } else {
      v_target_world[i] = 0.0f;
    }
  }

  // 获取当前机体系下的实际速度（用于速度环计算）
  float actual_v_body[4];
  for (int i = 0; i < 4; i++)
    actual_v_body[i] = actual_v[i];

  // 坐标系转换：将目标速度转换为机体系 (Body)
  float v_target_body[4];
  if (level_ == auv::common::ControlLevel::VELOCITY && target_v_is_body_) {
    // 如果本来就是机体系目标，直接使用
    for (int i = 0; i < 4; i++)
      v_target_body[i] = target_p[i];
  } else {
    // 否则从世界系转到机体系
    CoordinateManager::worldToBody(actual_p[3], v_target_world[0],
                                   v_target_world[1], v_target_body[0],
                                   v_target_body[1]);
    v_target_body[2] = v_target_world[2];
    v_target_body[3] = v_target_world[3];
  }

  for (int i = 0; i < 4; i++) {
    float f_base = 0.0f;
    if (level_ == auv::common::ControlLevel::POSITION ||
        level_ == auv::common::ControlLevel::VELOCITY) {
      // 获取当前轴的物理参数配置
      const auv::config::AxisConfig &axis_cfg =
          (i == 0) ? config_.x
                   : (i == 1 ? config_.y : (i == 2 ? config_.z : config_.yaw));

      // 使用机体系下的目标速度与机体系下的真实速度进行闭环
      float a_ref = config_.planner_enabled ? profiles_[i].getState().a : 0.0f;
      float a_actual = 0.0f;
      if (last_update_tick_ != 0 && dt > 0.0f) {
        a_actual = (actual_v_body[i] - last_v_body_[i]) / dt;
      }
      float vel_derivative = a_ref - a_actual;

      f_base = vel_pids_[i].compute(v_target_body[i] - actual_v_body[i], dt,
                                    vel_derivative);

      // 前馈补偿：F_ff = mass * a_ref + drag * v_ref
      float f_ff_accel = axis_cfg.mass * a_ref;
      float f_ff_drag = axis_cfg.drag * v_target_body[i];
      f_base += (f_ff_accel + f_ff_drag);
    }
    output_forces[i] = f_base + target_forces_[i];
  }

  for (int i = 0; i < 4; i++) {
    last_v_body_[i] = actual_v_body[i];
  }

  last_z_thrust_ = output_forces[2];
  last_output_forces_ = output_forces; // 更新快照

  return output_forces;
}

} // namespace control
} // namespace auv
