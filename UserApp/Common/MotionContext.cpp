#include "MotionContext.hpp"
#include "SystemContext.hpp"
#include "INS_Driver.hpp"
#include "SystemConfig.hpp"
#include "FreeRTOS.h"
#include "task.h"

namespace auv {
namespace motion {

MotionContext motion_context{};

void MotionContext::transformBodyToWorld(ControlLevel level, const float body_in[4], float world_out[4], bool is_inc) const {
    float cur_yaw = nav_state.pos_world[3];
    float cos_y = std::cos(cur_yaw);
    float sin_y = std::sin(cur_yaw);
    if (level == ControlLevel::POSITION) {
        float dx_w = body_in[0] * cos_y - body_in[1] * sin_y;
        float dy_w = body_in[0] * sin_y + body_in[1] * cos_y;
        if (is_inc) {
            world_out[0] = dx_w;
            world_out[1] = dy_w;
        } else {
            world_out[0] = nav_state.pos_world[0] + dx_w;
            world_out[1] = nav_state.pos_world[1] + dy_w;
        }
        world_out[2] = (is_inc ? 0.0f : nav_state.pos_world[2]) + body_in[2];
        world_out[3] = (is_inc ? 0.0f : cur_yaw) + body_in[3];
    } else {
        world_out[0] = body_in[0] * cos_y - body_in[1] * sin_y;
        world_out[1] = body_in[0] * sin_y + body_in[1] * cos_y;
        world_out[2] = body_in[2];
        world_out[3] = body_in[3];
    }
}

void MotionContext::transformWorldToBody(ControlLevel level, const float world_in[4], float body_out[4], bool is_inc) const {
    float cur_yaw = nav_state.pos_world[3];
    float cos_y = std::cos(cur_yaw);
    float sin_y = std::sin(cur_yaw);
    if (level == ControlLevel::POSITION) {
        float dx_w = world_in[0] - (is_inc ? 0.0f : nav_state.pos_world[0]);
        float dy_w = world_in[1] - (is_inc ? 0.0f : nav_state.pos_world[1]);
        body_out[0] = dx_w * cos_y + dy_w * sin_y;
        body_out[1] = -dx_w * sin_y + dy_w * cos_y;
        body_out[2] = world_in[2] - (is_inc ? 0.0f : nav_state.pos_world[2]);
        body_out[3] = world_in[3] - (is_inc ? 0.0f : cur_yaw);
    } else {
        body_out[0] = world_in[0] * cos_y + world_in[1] * sin_y;
        body_out[1] = -world_in[0] * sin_y + world_in[1] * cos_y;
        body_out[2] = world_in[2];
        body_out[3] = world_in[3];
    }
}

NavState MotionContext::getNavState() const {
    NavState nav;
    taskENTER_CRITICAL();
    nav = nav_state;
    taskEXIT_CRITICAL();
    return nav;
}

void MotionContext::setHomeOffset(float x, float y, float z, float yaw) {
    use_offset_ = true;
    offset_x_ = x;
    offset_y_ = y;
    offset_z_ = z;
    offset_yaw_ = yaw;
}

void MotionContext::clearHomeOffset() {
    use_offset_ = false;
}

float MotionContext::getMS5837Z() const {
    float z = 0.0f;
    taskENTER_CRITICAL();
    z = current_depth_z_;
    taskEXIT_CRITICAL();
    return z;
}

void MotionContext::setMS5837Z(float z) {
    taskENTER_CRITICAL();
    current_depth_z_ = z;
    taskEXIT_CRITICAL();
}

TargetSetpoint MotionContext::getCurrentSetpoint() const {
    TargetSetpoint sp;
    taskENTER_CRITICAL();
    sp = current_setpoint;
    taskEXIT_CRITICAL();
    return sp;
}

void MotionContext::setNavState(const NavState& state) {
    taskENTER_CRITICAL();
    nav_state = state;
    taskEXIT_CRITICAL();
}

void MotionContext::updateSetpoint(const TargetSetpoint& sp) {
    taskENTER_CRITICAL();
    current_setpoint = sp;
    taskEXIT_CRITICAL();
}

void MotionContext::resetSetpoint() {
    taskENTER_CRITICAL();
    for (int i = 0; i < 4; ++i) {
        current_setpoint.pos_world[i] = 0.0f;
        current_setpoint.vel_body[i] = 0.0f;
        current_setpoint.thrust_body[i] = 0.0f;
    }
    taskEXIT_CRITICAL();
}

RawSetpoint MotionContext::getRawSetpoint() const {
    RawSetpoint sp;
    taskENTER_CRITICAL();
    sp = raw_setpoint;
    taskEXIT_CRITICAL();
    return sp;
}

void MotionContext::setRawSetpoint(const RawSetpoint& sp) {
    taskENTER_CRITICAL();
    raw_setpoint = sp;
    taskEXIT_CRITICAL();
}

float MotionContext::getLastDtMs() const {
    float dt;
    taskENTER_CRITICAL();
    dt = last_dt_ms;
    taskEXIT_CRITICAL();
    return dt;
}

void MotionContext::setLastDtMs(float dt) {
    taskENTER_CRITICAL();
    last_dt_ms = dt;
    taskEXIT_CRITICAL();
}

uint32_t MotionContext::getLastReceivedSeq() const {
    uint32_t seq;
    taskENTER_CRITICAL();
    seq = last_received_seq;
    taskEXIT_CRITICAL();
    return seq;
}

void MotionContext::setLastReceivedSeq(uint32_t seq) {
    taskENTER_CRITICAL();
    last_received_seq = seq;
    taskEXIT_CRITICAL();
}

std::array<float, 4> MotionContext::getLastOutputForces() const {
    std::array<float, 4> forces;
    taskENTER_CRITICAL();
    for (int i = 0; i < 4; ++i) {
        forces[i] = last_output_forces[i];
    }
    taskEXIT_CRITICAL();
    return forces;
}

void MotionContext::setLastOutputForces(const std::array<float, 4>& forces) {
    taskENTER_CRITICAL();
    for (int i = 0; i < 4; ++i) {
        last_output_forces[i] = forces[i];
    }
    taskEXIT_CRITICAL();
}

} // namespace motion
} // namespace auv
