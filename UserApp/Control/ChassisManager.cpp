#include "ChassisManager.hpp"
#include "SystemConfig.hpp"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <algorithm>
#include <cmath>

// Empty anonymous namespace

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
    pos_cfg.dt = 0.01f; // 固定周期
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

auv::motion::ControlLevel ChassisManager::getControlLevel() const {
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

void ChassisManager::configurePID(int axis, bool is_pos_ring, float kp,
                                  float ki, float kd, float i_limit,
                                  float out_limit) {
  if (axis < 0 || axis >= 4)
    return;

  // 获取当前配置，用于增量修改
  PID_Controller::Config cfg = getPIDConfig(axis, is_pos_ring);

  if (kp >= 0.0f) cfg.kp = kp;
  if (ki >= 0.0f) cfg.ki = ki;
  if (kd >= 0.0f) cfg.kd = kd;
  if (i_limit >= 0.0f) cfg.i_limit = i_limit;
  if (out_limit >= 0.0f) cfg.output_limit = out_limit;
  cfg.dt = 0.01f; // 步长固定

  if (is_pos_ring)
    pos_pids_[axis].setConfig(cfg);
  else
    vel_pids_[axis].setConfig(cfg);
}

void ChassisManager::updateSetpoint(auv::motion::ControlLevel new_level, const float val[4], uint32_t mask, bool is_body, bool is_inc) {
  auto nav = auv::motion::motion_context.getNavState();
  auto sp = auv::motion::motion_context.getCurrentSetpoint();

  // 1. 模式切换对齐 (Bumpless Transition / Anti-Leakage)
  if (new_level != level_) {
    if (new_level == auv::motion::ControlLevel::POSITION) {
      for (int i = 0; i < 4; i++) {
        sp.pos_world[i] = nav.pos_world[i];
      }
    } else if (new_level == auv::motion::ControlLevel::VELOCITY) {
      for (int i = 0; i < 4; i++) {
        sp.vel_body[i] = nav.vel_body[i];
      }
    }
  }

  // 2. 执行坐标变换与目标值计算
  if (new_level == auv::motion::ControlLevel::POSITION) {
    float converted_val[4];
    if (is_body) {
      // 传入机体系位置设定（转换为世界系绝对或增量目标）
      auv::motion::motion_context.transformBodyToWorld(new_level, val,
                                                       converted_val, is_inc);
    }
    const float *target_val = is_body ? converted_val : val;

    for (int i = 0; i < 4; i++) {
      if (!(mask & (1 << i))) {
        if (is_inc) {
          sp.pos_world[i] += target_val[i];
        } else {
          sp.pos_world[i] = target_val[i];
        }
      }
    }
    sp.pos_world[3] = auv::motion::MotionContext::wrapAngle(sp.pos_world[3]);
  } else if (new_level == auv::motion::ControlLevel::VELOCITY) {
    float converted_val[4];
    if (!is_body) {
      // 传入世界系速度目标（转换为机体系绝对或增量目标）
      auv::motion::motion_context.transformWorldToBody(new_level, val,
                                                       converted_val, is_inc);
    }
    const float *target_val = is_body ? val : converted_val;

    for (int i = 0; i < 4; i++) {
      if (!(mask & (1 << i))) {
        if (is_inc) {
          sp.vel_body[i] += target_val[i];
        } else {
          sp.vel_body[i] = target_val[i];
        }
      }
    }
  } else if (new_level == auv::motion::ControlLevel::ACTUATOR) {
    float converted_val[4];
    if (!is_body) {
      // 传入世界系推力目标（转换为机体系绝对或增量目标）
      auv::motion::motion_context.transformWorldToBody(new_level, val,
                                                       converted_val, is_inc);
    }
    const float *target_val = is_body ? val : converted_val;

    for (int i = 0; i < 4; i++) {
      if (!(mask & (1 << i))) {
        if (is_inc) {
          sp.thrust_body[i] += target_val[i];
        } else {
          sp.thrust_body[i] = target_val[i];
        }
      }
    }
  }

  auv::motion::motion_context.updateSetpoint(sp);
  setControlLevel(new_level);
}

void ChassisManager::setControlLevel(auv::motion::ControlLevel new_level) {
  if (new_level == level_)
    return;
  auto nav = auv::motion::motion_context.getNavState();

  // 切换到 POSITION：对齐影子状态并清除 PID 积分
  if (new_level == auv::motion::ControlLevel::POSITION) {
    float actual_v_world[4];
    auv::motion::motion_context.transformBodyToWorld(
        auv::motion::ControlLevel::VELOCITY, nav.vel_body, actual_v_world, false);

    for (int i = 0; i < 4; i++) {
      profiles_[i].align(nav.pos_world[i], actual_v_world[i]);
      pos_pids_[i].reset_i();
      vel_pids_[i].reset_i();
    }
  }
  // 切换到 VELOCITY：对齐影子状态并清除速度环积分
  else if (new_level == auv::motion::ControlLevel::VELOCITY) {
    for (int i = 0; i < 4; i++) {
      // 速度环直接在机体系平滑对齐，位置对齐为0，速度为实际机体系速度
      profiles_[i].align(0.0f, nav.vel_body[i]);
      vel_pids_[i].reset_i();
    }
  }

  level_ = new_level;
}

std::array<float, 4> ChassisManager::update() {
  std::array<float, 4> output_forces = {0};
  uint32_t now = HAL_GetTick();
  float dt = (last_update_tick_ == 0)
                 ? 0.01f
                 : (float)(now - last_update_tick_) / 1000.0f;
  last_update_tick_ = now;

  // 防止 dt 异常：限定在 [1ms, 100ms] 范围
  dt = std::clamp(dt, 0.001f, 0.1f);

  if (level_ == auv::motion::ControlLevel::NONE)
    return output_forces;

  // 从全局上下文获取最新的反馈状态与设定值目标
  auto nav = auv::motion::motion_context.getNavState();
  auto target = auv::motion::motion_context.getCurrentSetpoint();

  const float *actual_p = nav.pos_world;
  const float *actual_v = nav.vel_body;

  // 计算世界系下的实际速度（用于位置环微分项）
  float actual_v_world_now[4];
  auv::motion::motion_context.transformBodyToWorld(
      auv::motion::ControlLevel::VELOCITY, actual_v, actual_v_world_now, false);

  float v_target_body[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  for (int i = 0; i < 4; i++) {
    if (level_ == auv::motion::ControlLevel::POSITION) {
      ProfileState profile_target;
      if (config_.planner_enabled) {
        if (i == 3) {
          float current = profiles_[i].getState().p;
          float target_yaw = current + auv::motion::MotionContext::wrapAngle(target.pos_world[i] - current);
          profile_target = profiles_[i].update(target_yaw, dt);
        } else {
          profile_target = profiles_[i].update(target.pos_world[i], dt);
        }
      } else {
        profile_target.p = target.pos_world[i];
        profile_target.v = 0.0f;
        profile_target.a = 0.0f;
      }

      // 位置环的导数项应使用世界系下的速度误差 (v_ref_world - v_actual_world)
      float actual_v_world_val = (i < 2) ? actual_v_world_now[i] : actual_v[i];
      float pos_derivative = profile_target.v - actual_v_world_val;
      float pos_error = profile_target.p - actual_p[i];
      if (i == 3)
        pos_error = auv::motion::MotionContext::wrapAngle(pos_error);
      float v_target_world_val = pos_pids_[i].compute(pos_error, dt, pos_derivative) + profile_target.v;

      if (i >= 2) {
        v_target_body[i] = v_target_world_val;
      }
      
      // 临时保存 X/Y 的世界系目标速度，并在算完 Y 后整体旋转至机体系
      static float temp_v_target_world[2];
      if (i < 2) {
        temp_v_target_world[i] = v_target_world_val;
      }
      if (i == 1) {
        float temp_v_w[4] = {temp_v_target_world[0], temp_v_target_world[1], 0.0f, 0.0f};
        float temp_v_b[4];
        auv::motion::motion_context.transformWorldToBody(
            auv::motion::ControlLevel::VELOCITY, temp_v_w, temp_v_b, false);
        v_target_body[0] = temp_v_b[0];
        v_target_body[1] = temp_v_b[1];
      }
    } else if (level_ == auv::motion::ControlLevel::VELOCITY) {
      // 速度环：直接跟踪机体系目标
      ProfileState d;
      if (config_.planner_enabled) {
        d = profiles_[i].updateVelocity(target.vel_body[i], dt);
      } else {
        d.p = 0.0f;
        d.v = target.vel_body[i];
        d.a = 0.0f;
      }
      v_target_body[i] = d.v;
    }
  }

  // 获取当前机体系下的实际速度（用于速度环计算）
  float actual_v_body[4];
  for (int i = 0; i < 4; i++)
    actual_v_body[i] = actual_v[i];

  for (int i = 0; i < 4; i++) {
    float f_base = 0.0f;
    if (level_ == auv::motion::ControlLevel::POSITION ||
        level_ == auv::motion::ControlLevel::VELOCITY) {
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
    output_forces[i] = f_base + target.thrust_body[i];
    // 强制截断到绝对物理极限 [-1.0, 1.0]
    output_forces[i] = std::max(-1.0f, std::min(1.0f, output_forces[i]));
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
