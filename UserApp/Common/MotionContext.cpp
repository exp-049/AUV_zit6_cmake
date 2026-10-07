#include "MotionContext.hpp"

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

void MotionContext::updateOdomLocked() {
  const auto &raw = raw_nav_snapshot_.unsafe();
  const auto &home = home_offset_.unsafe();
  auto &odom = odom_snapshot_.unsafe();
  odom.nav_state = raw.raw_nav;
  odom.nav_timestamp_ms = raw.nav_timestamp_ms;
  odom.nav_valid = raw.have_sample && raw.nav_valid;
  odom.origin_initialized = home.active;
  if (home.active) {
    const float dx = raw.raw_nav.pos_world[0] - home.offset[0];
    const float dy = raw.raw_nav.pos_world[1] - home.offset[1];
    const float c = std::cos(home.offset[5]), s = std::sin(home.offset[5]);
    odom.nav_state.pos_world[0] = c * dx + s * dy;
    odom.nav_state.pos_world[1] = -s * dx + c * dy;
    odom.nav_state.pos_world[2] -= home.offset[2];
    // Yaw-only frame transform leaves measured roll/pitch and body twist intact.
    odom.nav_state.pos_world[5] =
        wrapAngle(raw.raw_nav.pos_world[5] - home.offset[5]);
  }
  nav_state_.unsafe() = odom.nav_state;
}

void MotionContext::updateNavigationSnapshot(const NavState &raw,
                                              uint32_t sample_ms, bool valid) {
  for (size_t i = 0; i < 6; ++i)
    valid = valid && std::isfinite(raw.pos_world[i]) &&
            std::isfinite(raw.vel_body[i]);
  taskENTER_CRITICAL();
  auto &snapshot = raw_nav_snapshot_.unsafe();
  snapshot.raw_nav = raw;
  snapshot.nav_timestamp_ms = sample_ms;
  snapshot.nav_valid = valid;
  snapshot.have_sample = snapshot.have_sample || valid;
  updateOdomLocked();
  taskEXIT_CRITICAL();
}

bool MotionContext::trySetOrigin(uint32_t now_ms, uint32_t max_age_ms,
                                  OriginCommit &commit) {
  taskENTER_CRITICAL();
  const auto &raw = raw_nav_snapshot_.unsafe();
  if (!raw.have_sample || !raw.nav_valid ||
      now_ms - raw.nav_timestamp_ms > max_age_ms) {
    taskEXIT_CRITICAL();
    return false;
  }
  auto &home = home_offset_.unsafe();
  home.active = true;
  home.offset = {raw.raw_nav.pos_world[0], raw.raw_nav.pos_world[1],
                 raw.raw_nav.pos_world[2], 0.0f, 0.0f,
                 raw.raw_nav.pos_world[5]};
  auto &odom = odom_snapshot_.unsafe();
  ++odom.origin_generation;
  if (odom.origin_generation == 0) // zero is reserved for this boot's uninitialized state
    ++odom.origin_generation;
  updateOdomLocked();
  current_setpoint_.unsafe() = TargetSetpoint{};
  commit.origin_nav = home.offset;
  commit.nav_timestamp_ms = raw.nav_timestamp_ms;
  commit.origin_generation = odom.origin_generation;
  taskEXIT_CRITICAL();
  return true;
}

void MotionContext::setHomeOffset(
    const auv::algorithm::math::Vector6f &offset) {
  taskENTER_CRITICAL();
  auto &h = home_offset_.unsafe();
  h.active = true;
  Eigen::Map<auv::algorithm::math::Vector6f>(h.offset.data()) = offset;
  h.offset[3] = h.offset[4] = 0.0f;
  ++odom_snapshot_.unsafe().origin_generation;
  updateOdomLocked();
  taskEXIT_CRITICAL();
}

void MotionContext::clearHomeOffset() {
  taskENTER_CRITICAL();
  home_offset_.unsafe() = HomeOffset{};
  odom_snapshot_.unsafe().origin_generation = 0;
  updateOdomLocked();
  taskEXIT_CRITICAL();
}

} // namespace motion
} // namespace auv
