#include "MotionContext.hpp"
#include "FreeRTOS.h"
#include "INS_Driver.hpp"
#include "SystemConfig.hpp"
#include "SystemContext.hpp"
#include "task.h"

namespace auv {
namespace motion {

MotionContext motion_context{};

float MotionContext::wrapAngle(float angle) {
  if (angle > auv::math::kPi || angle < -auv::math::kPi) {
    angle = std::fmod(angle + auv::math::kPi, auv::math::kTwoPi);
    if (angle < 0.0f)
      angle += auv::math::kTwoPi;
    angle -= auv::math::kPi;
  }
  return angle;
}

void MotionContext::transformBodyToWorld(const float body_in[6],
                                         float world_out[6]) const {
  using namespace auv::math;
  NavState nav = getNavState();
  applyRotationToWorld(body_in, world_out, nav.pos_world[ROLL],
                       nav.pos_world[PITCH], nav.pos_world[YAW]);
}

void MotionContext::transformWorldToBody(const float world_in[6],
                                         float body_out[6]) const {
  using namespace auv::math;
  NavState nav = getNavState();
  applyRotationToBody(world_in, body_out, nav.pos_world[ROLL],
                      nav.pos_world[PITCH], nav.pos_world[YAW]);
}

void MotionContext::setHomeOffset(const auv::math::Vector6f &offset) {
  // Eigen 向量 → 连续成员变量的拷贝（offset_x_ .. offset_yaw_ 声明连续）
  Eigen::Map<auv::math::Vector6f> dest(&offset_x_);
  dest = offset;
  use_offset_ = true;
  ROS_LOG_INFO("Home offset set: x=%.2f y=%.2f z=%.2f r=%.2f p=%.2f y=%.2f",
               offset[0], offset[1], offset[2], offset[3], offset[4],
               offset[5]);
}

void MotionContext::clearHomeOffset() { use_offset_ = false; }

} // namespace motion
} // namespace auv
