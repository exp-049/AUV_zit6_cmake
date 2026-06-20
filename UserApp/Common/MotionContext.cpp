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

void MotionContext::transformBodyToWorld(ControlLevel level,
                                         const float body_in[4],
                                         float world_out[4],
                                         bool is_inc) const {
  auto nav = getNavState();
  using auv::math::YAW;
  float cur_yaw = nav.pos_world[YAW];
  float cos_y = std::cos(cur_yaw);
  float sin_y = std::sin(cur_yaw);
  if (level == ControlLevel::POSITION) {
    float dx_w = body_in[0] * cos_y - body_in[1] * sin_y;
    float dy_w = body_in[0] * sin_y + body_in[1] * cos_y;
    if (is_inc) {
      world_out[0] = dx_w;
      world_out[1] = dy_w;
    } else {
      world_out[0] = nav.pos_world[0] + dx_w;
      world_out[1] = nav.pos_world[1] + dy_w;
    }
    world_out[2] = (is_inc ? 0.0f : nav.pos_world[2]) + body_in[2];
    world_out[3] = (is_inc ? 0.0f : cur_yaw) + body_in[3];
  } else {
    world_out[0] = body_in[0] * cos_y - body_in[1] * sin_y;
    world_out[1] = body_in[0] * sin_y + body_in[1] * cos_y;
    world_out[2] = body_in[2];
    world_out[3] = body_in[3];
  }
}

void MotionContext::transformWorldToBody(ControlLevel level,
                                         const float world_in[4],
                                         float body_out[4], bool is_inc) const {
  auto nav = getNavState();
  using auv::math::YAW;
  float cur_yaw = nav.pos_world[YAW];
  float cos_y = std::cos(cur_yaw);
  float sin_y = std::sin(cur_yaw);
  if (level == ControlLevel::POSITION) {
    float dx_w = world_in[0] - (is_inc ? 0.0f : nav.pos_world[0]);
    float dy_w = world_in[1] - (is_inc ? 0.0f : nav.pos_world[1]);
    body_out[0] = dx_w * cos_y + dy_w * sin_y;
    body_out[1] = -dx_w * sin_y + dy_w * cos_y;
    body_out[2] = world_in[2] - (is_inc ? 0.0f : nav.pos_world[2]);
    body_out[3] = world_in[3] - (is_inc ? 0.0f : cur_yaw);
  } else {
    body_out[0] = world_in[0] * cos_y + world_in[1] * sin_y;
    body_out[1] = -world_in[0] * sin_y + world_in[1] * cos_y;
    body_out[2] = world_in[2];
    body_out[3] = world_in[3];
  }
}

void MotionContext::transformBodyToWorld6(const float body_in[6],
                                          float world_out[6]) const {
  using namespace auv::math;
  NavState nav = getNavState();
  applyRotationToWorld(body_in, world_out, nav.pos_world[ROLL],
                       nav.pos_world[PITCH], nav.pos_world[YAW]);
}

void MotionContext::transformWorldToBody6(const float world_in[6],
                                          float body_out[6]) const {
  using namespace auv::math;
  NavState nav = getNavState();
  applyRotationToBody(world_in, body_out, nav.pos_world[ROLL],
                      nav.pos_world[PITCH], nav.pos_world[YAW]);
}

void MotionContext::setHomeOffset(float x, float y, float z, float roll,
                                  float pitch, float yaw) {
  use_offset_ = true;
  offset_x_ = x;
  offset_y_ = y;
  offset_z_ = z;
  offset_roll_ = roll;
  offset_pitch_ = pitch;
  offset_yaw_ = yaw;
  ROS_LOG_INFO("Home offset set: x=%.2f y=%.2f z=%.2f r=%.2f p=%.2f y=%.2f", x,
               y, z, roll, pitch, yaw);
}

void MotionContext::clearHomeOffset() { use_offset_ = false; }

} // namespace motion
} // namespace auv
