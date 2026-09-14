#include "ChassisManager.hpp"

namespace auv {
namespace component {

ChassisManager::ChassisManager() : controller_() {}

ChassisManager::ChassisManager(const auv::config::ChassisConfig &cfg)
    : controller_(cfg) {}

void ChassisManager::applyConfig(const auv::config::ChassisConfig &cfg) {
  controller_.applyConfig(cfg);
}

auv::motion::ControlLevel ChassisManager::getControlLevel() const {
  return controller_.getControlLevel();
}

std::array<float, 6> ChassisManager::update() { return controller_.update(); }

void ChassisManager::updateSetpoint(auv::motion::ControlLevel new_level,
                                    const float val[6], uint32_t mask,
                                    bool is_body, bool is_inc) {
  auto lv = router_.route(controller_.getControlLevel(), new_level, val, mask,
                          is_body, is_inc);
  controller_.setControlLevel(lv);
}

void ChassisManager::setControlLevel(auv::motion::ControlLevel new_level) {
  controller_.setControlLevel(new_level);
}

void ChassisManager::configurePID(int axis, bool is_pos_ring, float kp,
                                  float ki, float kd, float i_limit,
                                  float out_limit) {
  controller_.configurePID(axis, is_pos_ring, kp, ki, kd, i_limit, out_limit);
}

PID_Controller::Config ChassisManager::getPIDConfig(int axis,
                                                    bool is_pos_ring) const {
  return controller_.getPIDConfig(axis, is_pos_ring);
}

void ChassisManager::getProfileLimits(int axis, float &max_v,
                                      float &max_a) const {
  controller_.getProfileLimits(axis, max_v, max_a);
}

void ChassisManager::configureProfile(int axis, float max_v, float max_a) {
  controller_.configureProfile(axis, max_v, max_a);
}

} // namespace component
} // namespace auv
