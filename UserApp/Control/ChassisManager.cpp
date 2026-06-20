#include "ChassisManager.hpp"
#include "FreeRTOS.h"
#include "SystemConfig.hpp"
#include "main.h"
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
  if (axis < 0 || axis >= 6)
    return {};
  return is_pos_ring ? pos_pids_[axis].getConfig()
                     : vel_pids_[axis].getConfig();
}

void ChassisManager::getProfileLimits(int axis, float &max_v,
                                      float &max_a) const {
  if (axis >= 0 && axis < 6) {
    max_v = profiles_[axis].getMaxV();
    max_a = profiles_[axis].getMaxA();
  }
}

void ChassisManager::configureProfile(int axis, float max_v, float max_a) {
  if (axis >= 0 && axis < 6) {
    float v = (max_v >= 0.0f) ? max_v : profiles_[axis].getMaxV();
    float a = (max_a >= 0.0f) ? max_a : profiles_[axis].getMaxA();
    profiles_[axis].setLimits(v, a);
  }
}

void ChassisManager::configurePID(int axis, bool is_pos_ring, float kp,
                                  float ki, float kd, float i_limit,
                                  float out_limit) {
  if (axis < 0 || axis >= 6)
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

void ChassisManager::updateSetpoint(auv::motion::ControlLevel new_level,
                                    const float val[4], uint32_t mask,
                                    bool is_body, bool is_inc) {
  auto nav = auv::motion::motion_context.getNavState();
  auto sp = auv::motion::motion_context.getCurrentSetpoint();

  // 1. 模式切换对齐 (Bumpless Transition / Anti-Leakage)
  if (new_level != level_) {
    if (new_level == auv::motion::ControlLevel::POSITION) {
      for (int i = 0; i < 6; i++) {
        sp.pos_world[i] = nav.pos_world[i];
      }
    } else if (new_level == auv::motion::ControlLevel::VELOCITY) {
      for (int i = 0; i < 6; i++) {
        sp.vel_body[i] = nav.vel_body[i];
      }
    }
  }

  // 2. 执行坐标变换与目标值计算（6DOF 矩阵版本）
  // ⚠️ val[4] = [X, Y, Z, Yaw] 来自 AGX 4DOF 协议。6DOF 索引为：
  //    [0]=X, [1]=Y, [2]=Z, [3]=Roll, [4]=Pitch, [5]=Yaw
  // 无论 is_body 如何，统一转换为 6 元素 converted_val，然后循环 6 次写入。
  float converted_val[6] = {0};

  if (new_level == auv::motion::ControlLevel::POSITION) {
    if (is_body) {
      // 机体系位置指令 → 世界系：填充 val6, 变换, 可选加 nav 位姿
      float val6[6];
      Eigen::Map<auv::math::Vector6f>(val6) << val[0], val[1], val[2],
          0.0f, 0.0f, val[3];
      float world6[6];
      auv::motion::motion_context.transformBodyToWorld(val6, world6);
      auto nav = auv::motion::motion_context.getNavState();
      Eigen::Map<auv::math::Vector6f> cv(converted_val);
      if (is_inc) {
        cv = Eigen::Map<const auv::math::Vector6f>(world6);
      } else {
        cv.noalias() =
            Eigen::Map<const auv::math::Vector6f>(nav.pos_world.data()) +
            Eigen::Map<const auv::math::Vector6f>(world6);
      }
    } else {
      // 世界系位置指令：直接填充（默认 roll/pitch=0）
      Eigen::Map<auv::math::Vector6f>(converted_val) << val[0], val[1], val[2],
          0.0f, 0.0f, val[3];
    }
    // 写入 6DOF setpoint，用 mask 控制哪些轴被覆盖
    for (int i = 0; i < 6; i++) {
      if (!(mask & (1 << (i > 2 ? i + 2 : i)))) {  // mask 位: [X=0,Y=1,Z=2,Yaw=3]
        sp.pos_world[i] = converted_val[i];
      }
      if (i >= 3) {
        sp.pos_world[i] =
            auv::motion::MotionContext::wrapAngle(sp.pos_world[i]);
      }
    }
  } else if (new_level == auv::motion::ControlLevel::VELOCITY) {
    if (is_body) {
      // 机体系速度指令：4DOF → 6DOF，[3]=Yaw → [5]=YawRate
      Eigen::Map<auv::math::Vector6f>(converted_val) << val[0], val[1], val[2],
          0.0f, 0.0f, val[3];
    } else {
      // 世界系速度指令：变换到机体系
      float world6[6];
      Eigen::Map<auv::math::Vector6f>(world6) << val[0], val[1], val[2],
          0.0f, 0.0f, val[3];
      float body6[6];
      auv::motion::motion_context.transformWorldToBody(world6, body6);
      std::memcpy(converted_val, body6, 6 * sizeof(float));
    }
    for (int i = 0; i < 6; i++) {
      if (!(mask & (1 << (i > 2 ? i + 2 : i)))) {
        sp.vel_body[i] = converted_val[i];
      }
    }
  } else if (new_level == auv::motion::ControlLevel::ACTUATOR) {
    if (is_body) {
      Eigen::Map<auv::math::Vector6f>(converted_val) << val[0], val[1], val[2],
          0.0f, 0.0f, val[3];
    } else {
      float world6[6];
      Eigen::Map<auv::math::Vector6f>(world6) << val[0], val[1], val[2],
          0.0f, 0.0f, val[3];
      float body6[6];
      auv::motion::motion_context.transformWorldToBody(world6, body6);
      std::memcpy(converted_val, body6, 6 * sizeof(float));
    }
    for (int i = 0; i < 6; i++) {
      if (!(mask & (1 << (i > 2 ? i + 2 : i)))) {
        sp.thrust_body[i] = converted_val[i];
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
    float actual_v_world[6];
    auv::motion::motion_context.transformBodyToWorld(nav.vel_body.data(),
                                                     actual_v_world);

    for (int i = 0; i < 6; i++) {
      profiles_[i].align(nav.pos_world[i], actual_v_world[i]);
      pos_pids_[i].reset_i();
      vel_pids_[i].reset_i();
    }
  }
  // 切换到 VELOCITY：对齐影子状态并清除速度环积分
  else if (new_level == auv::motion::ControlLevel::VELOCITY) {
    for (int i = 0; i < 6; i++) {
      // 速度环直接在机体系平滑对齐，位置对齐为0，速度为实际机体系速度
      profiles_[i].align(0.0f, nav.vel_body[i]);
      vel_pids_[i].reset_i();
    }
  }

  level_ = new_level;
}

std::array<float, 6> ChassisManager::update() {
  std::array<float, 6> output_forces = {0};
  // 控制周期由 vTaskDelayUntil 严格锁定在 10ms，使用固定 dt 避免
  // HAL_GetTick ±1ms 截断噪声引入 10% 的微分项抖动
  constexpr float kDt = 0.01f;
  float dt = kDt;

  if (level_ == auv::motion::ControlLevel::NONE)
    return output_forces;

  // 从全局上下文获取最新的反馈状态与设定值目标
  auto nav = auv::motion::motion_context.getNavState();
  auto target = auv::motion::motion_context.getCurrentSetpoint();

  const float *actual_p_world = nav.pos_world.data();
  const float *actual_v_body = nav.vel_body.data();

  // 计算世界系下的实际速度（用于位置环微分项）
  float actual_v_world[6];
  auv::motion::motion_context.transformBodyToWorld(actual_v_body,
                                                   actual_v_world);

  float v_target_body[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  // pos->vel
  if (level_ == auv::motion::ControlLevel::POSITION) {
    float v_target_world[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    for (int i = 0; i < 4; i++) {
      ProfileState profile_target;
      if (config_.planner_enabled) {
        if (i == 3) {
          float current = profiles_[i].getState().p;
          float target_yaw =
              current +
              auv::motion::MotionContext::wrapAngle(
                  target.pos_world[i] -
                  current); // 启用规划器时，航向采用最短旋转路径目标值
          profile_target = profiles_[i].updatePosition(target_yaw, dt);
        } else {
          profile_target = profiles_[i].updatePosition(target.pos_world[i], dt);
        }
      } else {
        profile_target.p = target.pos_world[i];
        profile_target.v = 0.0f;
        profile_target.a = 0.0f;
      }

      // 位置环的导数项应使用世界系下的速度误差 (v_ref_world - v_actual_world)
      float pos_derivative = profile_target.v - actual_v_world[i];
      float pos_error = profile_target.p - actual_p_world[i];
      if (i == 3)
        pos_error = auv::motion::MotionContext::wrapAngle(pos_error);
      v_target_world[i] = pos_pids_[i].compute(pos_error, dt, pos_derivative) +
                          profile_target.v;
    }

    auv::motion::motion_context.transformWorldToBody(v_target_world,
                                                     v_target_body);

  } else if (level_ == auv::motion::ControlLevel::VELOCITY) {
    // 速度环：直接跟踪机体系目标
    for (int i = 0; i < 6; i++) {
      if (config_.planner_enabled) {
        v_target_body[i] =
            profiles_[i].updateVelocity(target.vel_body[i], dt).v;
      } else {
        v_target_body[i] = target.vel_body[i];
      }
    }
  }
  // vel->thr
  for (int i = 0; i < 6; i++) {
    float f_base = 0.0f;
    if (level_ == auv::motion::ControlLevel::POSITION ||
        level_ == auv::motion::ControlLevel::VELOCITY) {
      // 通过本地指针数组获取轴配置（避免 IIFE lambda + switch 开销）
      const auv::config::AxisConfig *axes[6] = {&config_.x, &config_.y,
          &config_.z, &config_.roll, &config_.pitch, &config_.yaw};
      const auv::config::AxisConfig &axis_cfg = *axes[i];

      // 使用机体系下的目标速度与机体系下的真实速度进行闭环
      float a_ref = config_.planner_enabled ? profiles_[i].getState().a : 0.0f;
      float a_actual = (actual_v_body[i] - last_v_body_[i]) / dt;
      float vel_derivative = a_ref - a_actual;

      f_base = vel_pids_[i].compute(v_target_body[i] - actual_v_body[i], dt,
                                    vel_derivative);

      if (config_.planner_enabled) {
        // 前馈补偿：F_ff = mass * a_ref + drag * v_ref
        float f_ff_accel = axis_cfg.mass * a_ref;
        float f_ff_drag = axis_cfg.drag * v_target_body[i];
        f_base += (f_ff_accel + f_ff_drag);
      }
    }
    output_forces[i] = f_base + target.thrust_body[i];
    // 强制截断到绝对物理极限 [-1.0, 1.0]
    output_forces[i] = std::max(-1.0f, std::min(1.0f, output_forces[i]));
  }

  for (int i = 0; i < 6; i++) {
    last_v_body_[i] = actual_v_body[i];
  }

  last_z_thrust_ = output_forces[2];
  last_output_forces_ = output_forces; // 更新快照

  return output_forces;
}

} // namespace control
} // namespace auv
