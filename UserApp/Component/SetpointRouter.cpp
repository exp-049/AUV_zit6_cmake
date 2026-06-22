#include "SetpointRouter.hpp"
#include "MathUtils.hpp"
#include <Eigen/Core>
#include <cstring>

namespace auv {
namespace component {

auv::motion::ControlLevel
SetpointRouter::route(auv::motion::ControlLevel current_level,
                      auv::motion::ControlLevel new_level, const float val[4],
                      uint32_t mask, bool is_body, bool is_inc) {
  auto nav = auv::motion::motion_context.nav_state_.get();
  auto sp = auv::motion::motion_context.current_setpoint_.get();

  // 1. 模式切换对齐 (Bumpless Transition / Anti-Leakage)
  if (new_level != current_level) {
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
  // val[4] = [X, Y, Z, Yaw] 来自 AGX 4DOF 协议。6DOF 索引为：
  //    [0]=X, [1]=Y, [2]=Z, [3]=Roll, [4]=Pitch, [5]=Yaw
  float converted_val[6] = {0};

  if (new_level == auv::motion::ControlLevel::POSITION) {
    if (is_body) {
      // 机体系位置指令 → 世界系：填充 val6, 变换, 可选加 nav 位姿
      float val6[6];
      Eigen::Map<auv::algorithm::math::Vector6f>(val6) << val[0], val[1],
          val[2], 0.0f, 0.0f, val[3];
      float world6[6];
      {
        auto _n = auv::motion::motion_context.nav_state_.get();
        auv::algorithm::math::applyRotationToWorld(
            val6, world6, _n.pos_world[3], _n.pos_world[4], _n.pos_world[5]);
      }
      auto nav = auv::motion::motion_context.nav_state_.get();
      Eigen::Map<auv::algorithm::math::Vector6f> cv(converted_val);
      if (is_inc) {
        cv = Eigen::Map<const auv::algorithm::math::Vector6f>(world6);
      } else {
        cv.noalias() = Eigen::Map<const auv::algorithm::math::Vector6f>(
                           nav.pos_world.data()) +
                       Eigen::Map<const auv::algorithm::math::Vector6f>(world6);
      }
    } else {
      // 世界系位置指令：直接填充（默认 roll/pitch=0）
      Eigen::Map<auv::algorithm::math::Vector6f>(converted_val) << val[0],
          val[1], val[2], 0.0f, 0.0f, val[3];
    }
    // 写入 6DOF setpoint，用 mask 控制哪些轴被覆盖
    for (int i = 0; i < 6; i++) {
      if (!(mask &
            (1 << (i > 2 ? i + 2 : i)))) { // mask 位: [X=0,Y=1,Z=2,Yaw=3]
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
      Eigen::Map<auv::algorithm::math::Vector6f>(converted_val) << val[0],
          val[1], val[2], 0.0f, 0.0f, val[3];
    } else {
      // 世界系速度指令：变换到机体系
      float world6[6];
      Eigen::Map<auv::algorithm::math::Vector6f>(world6) << val[0], val[1],
          val[2], 0.0f, 0.0f, val[3];
      float body6[6];
      {
        auto _n = auv::motion::motion_context.nav_state_.get();
        auv::algorithm::math::applyRotationToBody(
            world6, body6, _n.pos_world[3], _n.pos_world[4], _n.pos_world[5]);
      }
      std::memcpy(converted_val, body6, 6 * sizeof(float));
    }
    for (int i = 0; i < 6; i++) {
      if (!(mask & (1 << (i > 2 ? i + 2 : i)))) {
        sp.vel_body[i] = converted_val[i];
      }
    }
  } else if (new_level == auv::motion::ControlLevel::ACTUATOR) {
    if (is_body) {
      Eigen::Map<auv::algorithm::math::Vector6f>(converted_val) << val[0],
          val[1], val[2], 0.0f, 0.0f, val[3];
    } else {
      float world6[6];
      Eigen::Map<auv::algorithm::math::Vector6f>(world6) << val[0], val[1],
          val[2], 0.0f, 0.0f, val[3];
      float body6[6];
      {
        auto _n = auv::motion::motion_context.nav_state_.get();
        auv::algorithm::math::applyRotationToBody(
            world6, body6, _n.pos_world[3], _n.pos_world[4], _n.pos_world[5]);
      }
      std::memcpy(converted_val, body6, 6 * sizeof(float));
    }
    for (int i = 0; i < 6; i++) {
      if (!(mask & (1 << (i > 2 ? i + 2 : i)))) {
        sp.thrust_body[i] = converted_val[i];
      }
    }
  }

  auv::motion::motion_context.current_setpoint_.set(sp);
  return new_level;
}

} // namespace component
} // namespace auv
