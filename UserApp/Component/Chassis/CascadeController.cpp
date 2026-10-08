#include "CascadeController.hpp"
#include "MathUtils.hpp"
#include "SystemConfig.hpp"
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

auv::motion::AxisControlLevels CascadeController::getAxisControlLevels() const {
  return axis_levels_;
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
  if (new_level == auv::motion::ControlLevel::MIXED)
    return;

  auv::motion::AxisControlLevels new_levels;
  new_levels.fill(new_level);
  setAxisControlLevels(new_levels);
}

void CascadeController::setAxisControlLevels(
    const auv::motion::AxisControlLevels &new_levels) {
  auto nav = auv::motion::motion_context.nav_state_.get();
  float actual_v_world[6];
  auv::algorithm::math::applyRotationToWorld(
      nav.vel_body.data(), actual_v_world, nav.pos_world[3], nav.pos_world[4],
      nav.pos_world[5]);

  for (int i = 0; i < 6; ++i) {
    const auto next = new_levels[i] == auv::motion::ControlLevel::MIXED
                          ? auv::motion::ControlLevel::NONE
                          : new_levels[i];
    if (next == axis_levels_[i])
      continue;

    // Each axis aligns only when its own mode changes. A masked axis therefore
    // keeps its controller state and continues following its existing target.
    if (next == auv::motion::ControlLevel::POSITION) {
      profiles_[i].align(nav.pos_world[i], actual_v_world[i]);
      pos_pids_[i].reset_i();
      vel_pids_[i].reset_i();
    } else if (next == auv::motion::ControlLevel::VELOCITY) {
      profiles_[i].align(0.0f, nav.vel_body[i]);
      vel_pids_[i].reset_i();
    }

    axis_levels_[i] = next;
  }

  level_ = axis_levels_[0];
  for (int i = 1; i < 6; ++i) {
    if (axis_levels_[i] != level_) {
      level_ = auv::motion::ControlLevel::MIXED;
      break;
    }
  }
}

std::array<float, 6> CascadeController::update() {
  std::array<float, 6> output_forces = {0};
  // dt = 0.01s (100Hz)，信任 ControlTask 的 vTaskDelayUntil 严格 10ms 周期
  constexpr float kDt = 0.01f;

  bool any_active_axis = false;
  for (const auto axis_level : axis_levels_)
    any_active_axis |= axis_level != auv::motion::ControlLevel::NONE;
  if (!any_active_axis)
    return output_forces;

  auto nav = auv::motion::motion_context.nav_state_.get();
  auto target = auv::motion::motion_context.current_setpoint_.get();

  const float *actual_p_world = nav.pos_world.data();
  const float *actual_v_body = nav.vel_body.data();

  // 计算世界系下的实际速度（用于位置环微分项）
  float actual_v_world[6];
  auv::algorithm::math::applyRotationToWorld(actual_v_body, actual_v_world,
                                             nav.pos_world[3], nav.pos_world[4],
                                             nav.pos_world[5]);

  float v_target_world_position[6] = {0.0f, 0.0f, 0.0f,
                                      0.0f, 0.0f, 0.0f};

  // Position loops operate in world coordinates. Only axes currently in
  // POSITION contribute to this world-frame velocity vector.
  for (int i = 0; i < 6; ++i) {
    if (axis_levels_[i] != auv::motion::ControlLevel::POSITION)
      continue;

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
    v_target_world_position[i] =
        pos_pids_[i].compute(pos_error, kDt, pos_derivative) +
        profile_target.v;
  }

  float v_target_body_position[6] = {0.0f, 0.0f, 0.0f,
                                     0.0f, 0.0f, 0.0f};
  auv::algorithm::math::applyRotationToBody(
      v_target_world_position, v_target_body_position, nav.pos_world[3],
      nav.pos_world[4], nav.pos_world[5]);

  float v_target_body_velocity[6] = {0.0f, 0.0f, 0.0f,
                                     0.0f, 0.0f, 0.0f};
  for (int i = 0; i < 6; ++i) {
    if (axis_levels_[i] != auv::motion::ControlLevel::VELOCITY)
      continue;
    if (config_.planner_enabled) {
      v_target_body_velocity[i] =
          profiles_[i].updateVelocity(target.vel_body[i], kDt).v;
    } else {
      v_target_body_velocity[i] = target.vel_body[i];
    }
  }

  // Select each body-axis target independently. Position-derived targets have
  // already been transformed from world coordinates to the body frame.
  for (int i = 0; i < 6; i++) {
    float f_base = 0.0f;
    const auto axis_level = axis_levels_[i];
    if (axis_level == auv::motion::ControlLevel::POSITION ||
        axis_level == auv::motion::ControlLevel::VELOCITY) {
      const auv::config::AxisConfig *axes[6] = {&config_.x,     &config_.y,
                                                &config_.z,     &config_.roll,
                                                &config_.pitch, &config_.yaw};
      const auv::config::AxisConfig &axis_cfg = *axes[i];
      const float v_target =
          axis_level == auv::motion::ControlLevel::POSITION
              ? v_target_body_position[i]
              : v_target_body_velocity[i];

      float a_ref = config_.planner_enabled ? profiles_[i].getState().a : 0.0f;
      float a_actual = (actual_v_body[i] - last_v_body_[i]) / kDt;
      float vel_derivative = a_ref - a_actual;

      f_base = vel_pids_[i].compute(v_target - actual_v_body[i], kDt,
                                    vel_derivative);

      if (config_.planner_enabled) {
        float f_ff_accel = axis_cfg.mass * a_ref;
        float f_ff_drag = axis_cfg.drag * v_target;
        f_base += (f_ff_accel + f_ff_drag);
      }
      // Keep the existing thrust trim additive for closed-loop axes.
      f_base += target.thrust_body[i];
    } else if (axis_level == auv::motion::ControlLevel::ACTUATOR) {
      f_base = target.thrust_body[i];
    }
    output_forces[i] = std::max(-1.0f, std::min(1.0f, f_base));
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
