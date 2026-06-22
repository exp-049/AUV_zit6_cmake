#include "MotionContext.hpp"
#include "FreeRTOS.h"
#include "task.h"

namespace auv {
namespace motion {

MotionContext motion_context{};

float MotionContext::wrapAngle(float angle) {
  if (angle > auv::algorithm::math::kPi || angle < -auv::algorithm::math::kPi) {
    angle = std::fmod(angle + auv::algorithm::math::kPi,
                      auv::algorithm::math::kTwoPi);
    if (angle < 0.0f)
      angle += auv::algorithm::math::kTwoPi;
    angle -= auv::algorithm::math::kPi;
  }
  return angle;
}

void MotionContext::setHomeOffset(
    const auv::algorithm::math::Vector6f &offset) {
  HomeOffset h;
  h.active = true;
  Eigen::Map<auv::algorithm::math::Vector6f>(h.offset.data()) = offset;
  home_offset_.set(h);
  ROS_LOG_INFO("Home offset set: x=%.2f y=%.2f z=%.2f r=%.2f p=%.2f y=%.2f",
               offset[0], offset[1], offset[2], offset[3], offset[4],
               offset[5]);
}

void MotionContext::clearHomeOffset() {
  HomeOffset h;
  home_offset_.set(h);
}

} // namespace motion
} // namespace auv
