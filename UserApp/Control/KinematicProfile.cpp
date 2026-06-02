#include "KinematicProfile.hpp"
#include <cmath>
#include <algorithm>

namespace auv {
namespace control {

void KinematicProfile::setLimits(float max_v, float max_a) {
    limits_.max_v = std::abs(max_v);
    limits_.max_a = std::abs(max_a);
}

ProfileState KinematicProfile::updateVelocity(float target_v, float dt) {
    if (dt <= 0.0f || limits_.max_a <= 0.0f) {
        state_.a = 0.0f;
        return state_;
    }

    float v_cmd = std::max(-limits_.max_v, std::min(limits_.max_v, target_v));
    float delta_v = v_cmd - state_.v;
    
    // 限制加速度
    state_.a = std::max(-limits_.max_a, std::min(limits_.max_a, delta_v / dt));
    
    state_.v += state_.a * dt;
    state_.p += state_.v * dt; // 位置依然累加，保证切回位置环时有参考点

    return state_;
}
ProfileState KinematicProfile::updatePosition(float target_p, float dt) {
    if (dt <= 0.0f || limits_.max_v <= 0.0f || limits_.max_a <= 0.0f) {
        state_.a = 0.0f;
        state_.v = 0.0f;
        return state_;
    }

    float delta_p = target_p - state_.p;

    float v_ideal = 0.0f;
    if (std::abs(delta_p) > 1e-4f) {
        v_ideal = std::sqrt(2.0f * limits_.max_a * std::abs(delta_p));
        if (delta_p < 0) v_ideal = -v_ideal;
    }

    float v_cmd = std::max(-limits_.max_v, std::min(limits_.max_v, v_ideal));

    float a_raw = (v_cmd - state_.v) / dt;
    state_.a = std::max(-limits_.max_a, std::min(limits_.max_a, a_raw));

    state_.v += state_.a * dt;
    state_.p += state_.v * dt;

    return state_;
}

void KinematicProfile::align(float actual_p, float actual_v) {
    state_.p = actual_p;
    state_.v = actual_v;
    state_.a = 0.0f;
}

const ProfileState& KinematicProfile::getState() const {
    return state_;
}

} // namespace control
} // namespace auv
