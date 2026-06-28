#include "CascadeController.hpp"
#include "FreeRTOS.h"
#include "MathUtils.hpp"
#include "SystemConfig.hpp"
#include "main.h"
#include "task.h"
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace auv {
namespace component {

CascadeController::CascadeController() {
  applyConfig(auv::config::ChassisConfig());
}

CascadeController::CascadeController(const auv::config::ChassisConfig &cfg) {
  applyConfig(cfg);
}

void CascadeController::applyConfig(const auv::config::ChassisConfig &cfg) {
  config_ = cfg;
  const auv::config::AxisConfig *axes[6] = {&cfg.x,    &cfg.y,     &cfg.z,
                                            &cfg.roll, &cfg.pitch, &cfg.yaw};

  for (int i = 0; i < 6; i++) {
    const auto &axis_cfg = *axes[i];
    profiles_[i].setLimits(axis_cfg.max_v, axis_cfg.max_a);

    PID_Controller::Config pos_cfg;
    pos_cfg.kp = axis_cfg.pos_kp;
    pos_cfg.ki = axis_cfg.pos_ki;
    pos_cfg.kd = axis_cfg.pos_kd;
    pos_cfg.i_limit = axis_cfg.pos_i_limit;
    pos_cfg.output_limit = axis_cfg.pos_output_limit;
    pos_cfg.dt = 0.01f;
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

auv::motion::ControlLevel CascadeController::getControlLevel() const {
  return level_;
}

PID_Controller::Config CascadeController::getPIDConfig(int axis,
                                                       bool is_pos_ring) const {
  if (axis < 0 || axis >= 6)
    return {};
  return is_pos_ring ? pos_pids_[axis].getConfig()
                     : vel_pids_[axis].getConfig();
}

void CascadeController::getProfileLimits(int axis, float &max_v,
                                         float &max_a) const {
  if (axis >= 0 && axis < 6) {
    max_v = profiles_[axis].getMaxV();
    max_a = profiles_[axis].getMaxA();
  }
}

void CascadeController::configureProfile(int axis, float max_v, float max_a) {
  if (axis >= 0 && axis < 6) {
    float v = (max_v >= 0.0f) ? max_v : profiles_[axis].getMaxV();
    float a = (max_a >= 0.0f) ? max_a : profiles_[axis].getMaxA();
    profiles_[axis].setLimits(v, a);
  }
}

void CascadeController::configurePID(int axis, bool is_pos_ring, float kp,
                                     float ki, float kd, float i_limit,
                                     float out_limit) {
  if (axis < 0 || axis >= 6)
    return;

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
  cfg.dt = 0.01f;

  if (is_pos_ring)
    pos_pids_[axis].setConfig(cfg);
  else
    vel_pids_[axis].setConfig(cfg);
}

void CascadeController::setControlLevel(auv::motion::ControlLevel new_level) {
  if (new_level == level_)
    return;
  auto nav = auv::motion::motion_context.nav_state_.get();

  // 切换到 POSITION：对齐影子状态并清除 PID 积分
  if (new_level == auv::motion::ControlLevel::POSITION) {
    float actual_v_world[6];
    {
      auto _n = auv::motion::motion_context.nav_state_.get();
      auv::algorithm::math::applyRotationToWorld(
          nav.vel_body.data(), actual_v_world, _n.pos_world[3], _n.pos_world[4],
          _n.pos_world[5]);
    }

    for (int i = 0; i < 6; i++) {
      profiles_[i].align(nav.pos_world[i], actual_v_world[i]);
      pos_pids_[i].reset_i();
      vel_pids_[i].reset_i();
    }
  }
  // 切换到 VELOCITY：对齐影子状态并清除速度环积分
  else if (new_level == auv::motion::ControlLevel::VELOCITY) {
    for (int i = 0; i < 6; i++) {
      profiles_[i].align(0.0f, nav.vel_body[i]);
      vel_pids_[i].reset_i();
    }
  }

  level_ = new_level;
}

std::array<float, 6> CascadeController::update() {
  std::array<float, 6> output_forces = {0};
  // dt = 0.01s (100Hz)，信任 ControlTask 的 vTaskDelayUntil 严格 10ms 周期
  constexpr float kDt = 0.01f;

  if (level_ == auv::motion::ControlLevel::NONE)
    return output_forces;

  auto nav = auv::motion::motion_context.nav_state_.get();
  auto target = auv::motion::motion_context.current_setpoint_.get();

  const float *actual_p_world = nav.pos_world.data();
  const float *actual_v_body = nav.vel_body.data();

  // 计算世界系下的实际速度（用于位置环微分项）
  float actual_v_world[6];
  {
    auto _n = auv::motion::motion_context.nav_state_.get();
    auv::algorithm::math::applyRotationToWorld(actual_v_body, actual_v_world,
                                               _n.pos_world[3], _n.pos_world[4],
                                               _n.pos_world[5]);
  }

  float v_target_body[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

  // POSITION → VELOCITY
  if (level_ == auv::motion::ControlLevel::POSITION) {
    float v_target_world[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    for (int i = 0; i < 6; i++) {
      ProfileState profile_target;
      if (config_.planner_enabled) {
        float raw_target = target.pos_world[i];
        if (i >= 3) {
          float current = profiles_[i].getState().p;
          raw_target = current + auv::motion::MotionContext::wrapAngle(
                                     raw_target - current);
        }
        profile_target = profiles_[i].updatePosition(raw_target, kDt);
      } else {
        profile_target.p = target.pos_world[i];
        profile_target.v = 0.0f;
        profile_target.a = 0.0f;
      }

      float pos_derivative = profile_target.v - actual_v_world[i];
      float pos_error = profile_target.p - actual_p_world[i];
      if (i >= 3)
        pos_error = auv::motion::MotionContext::wrapAngle(pos_error);
      v_target_world[i] = pos_pids_[i].compute(pos_error, kDt, pos_derivative) +
                          profile_target.v;
    }

    {
      auto _n = auv::motion::motion_context.nav_state_.get();
      auv::algorithm::math::applyRotationToBody(
          v_target_world, v_target_body, _n.pos_world[3], _n.pos_world[4],
          _n.pos_world[5]);
    }

  } else if (level_ == auv::motion::ControlLevel::VELOCITY) {
    for (int i = 0; i < 6; i++) {
      if (config_.planner_enabled) {
        v_target_body[i] =
            profiles_[i].updateVelocity(target.vel_body[i], kDt).v;
      } else {
        v_target_body[i] = target.vel_body[i];
      }
    }
  }

  // VELOCITY → ACTUATOR (thrust)
  for (int i = 0; i < 6; i++) {
    float f_base = 0.0f;
    if (level_ == auv::motion::ControlLevel::POSITION ||
        level_ == auv::motion::ControlLevel::VELOCITY) {
      const auv::config::AxisConfig *axes[6] = {&config_.x,     &config_.y,
                                                &config_.z,     &config_.roll,
                                                &config_.pitch, &config_.yaw};
      const auv::config::AxisConfig &axis_cfg = *axes[i];

      float a_ref = config_.planner_enabled ? profiles_[i].getState().a : 0.0f;
      float a_actual = (actual_v_body[i] - last_v_body_[i]) / kDt;
      float vel_derivative = a_ref - a_actual;

      f_base = vel_pids_[i].compute(v_target_body[i] - actual_v_body[i], kDt,
                                    vel_derivative);

      if (config_.planner_enabled) {
        float f_ff_accel = axis_cfg.mass * a_ref;
        float f_ff_drag = axis_cfg.drag * v_target_body[i];
        f_base += (f_ff_accel + f_ff_drag);
      }
    }
    output_forces[i] = f_base + target.thrust_body[i];
    output_forces[i] = std::max(-1.0f, std::min(1.0f, output_forces[i]));
  }

  for (int i = 0; i < 6; i++) {
    last_v_body_[i] = actual_v_body[i];
  }

  last_z_thrust_ = output_forces[2];
  last_output_forces_ = output_forces;

  return output_forces;
}

} // namespace component
} // namespace auv
