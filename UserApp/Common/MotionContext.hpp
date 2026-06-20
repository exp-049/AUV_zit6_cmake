#ifndef __MOTION_CONTEXT_HPP
#define __MOTION_CONTEXT_HPP

#include <stdint.h>
#include <cmath>
#include <array>
#include "FreeRTOS.h"
#include "task.h"
#include "RosLogger.hpp"
#include "MathUtils.hpp"


namespace auv {
namespace motion {

/**
 * @enum ControlLevel
 * @brief 控制层级枚举
 */
enum class ControlLevel : uint8_t {
    NONE = 0,     ///< 待机/锁定模式
    POSITION = 1, ///< 位置闭环
    VELOCITY = 2, ///< 速度闭环
    ACTUATOR = 3  ///< 直接推力控制
};

/**
 * @struct NavState
 * @brief 6-DOF 导航状态结构体
 *
 * 索引约定（与 MathUtils::Axis 一致）：
 *   [0]=X, [1]=Y, [2]=Z, [3]=Roll, [4]=Pitch, [5]=Yaw
 *
 * 世界系坐标采用 NED (North-East-Down) 惯例。
 * 机体系坐标采用 FRD (Front-Right-Down) 惯例。
 */
struct NavState {
    std::array<float, 6> pos_world = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};  ///< NED: [x, y, z, φ, θ, ψ]
    std::array<float, 6> vel_body  = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};  ///< FRD: [u, v, w, p, q, r]
};

/**
 * @struct TargetSetpoint
 * @brief 管理级联控制中各层级的目标设定值（6DOF）
 */
struct TargetSetpoint {
    std::array<float, 6> pos_world   = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};  ///< 世界系目标 [x, y, z, φ, θ, ψ]
    std::array<float, 6> vel_body    = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};  ///< 机体系速度 [u, v, w, p, q, r]
    std::array<float, 6> thrust_body = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};  ///< 机体系推力 [Fx, Fy, Fz, Mk, Mm, Mn]
};

/**
 * @struct RawSetpoint
 * @brief 记录原始的 AGX 控制设定值快照
 */
struct RawSetpoint {
    ControlLevel level = ControlLevel::NONE;
    float data[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint32_t type_mask = 0;
    bool is_body = false;
    bool is_incremental = false;
};

/**
 * @struct OffboardSetpoint
 * @brief 上位机原始指令结构体
 */
struct OffboardSetpoint {
    ControlLevel level;
    float data[4];
    uint32_t type_mask;
};

/**
 * @struct Constants
 * @brief 系统数学与控制常数
 */
struct Constants {
    static constexpr float CONTROL_FREQ = 100.0f;       ///< 控制频率 (Hz)
    static constexpr uint32_t CONTROL_PERIOD_MS = 10;   ///< 控制周期 (ms)
    static constexpr float DEG2RAD = 0.0174532925f;    ///< 角度转弧度
    static constexpr float RAD2DEG = 57.2957795f;      ///< 弧度转角度
};

class MotionContext {
public:
    static float wrapAngle(float angle);

    // 旧 4DOF 变换接口（过渡期保留，Step 4 替换为 6DOF 矩阵版本）
    void transformBodyToWorld(ControlLevel level, const float body_in[4], float world_out[4], bool is_inc) const;
    void transformWorldToBody(ControlLevel level, const float world_in[4], float body_out[4], bool is_inc) const;

    // 新 6DOF 变换接口（准备接入 MathUtils 矩阵）
    void transformBodyToWorld6(const float body_in[6], float world_out[6]) const;
    void transformWorldToBody6(const float world_in[6], float body_out[6]) const;

    // 线程安全存取接口
    NavState getNavState() const {
        NavState state;
        taskENTER_CRITICAL();
        state = nav_state;
        taskEXIT_CRITICAL();
        return state;
    }

    void setNavState(const NavState& state) {
        taskENTER_CRITICAL();
        nav_state = state;
        taskEXIT_CRITICAL();
    }

    TargetSetpoint getCurrentSetpoint() const {
        TargetSetpoint sp;
        taskENTER_CRITICAL();
        sp = current_setpoint;
        taskEXIT_CRITICAL();
        return sp;
    }

    void updateSetpoint(const TargetSetpoint& sp) {
        taskENTER_CRITICAL();
        current_setpoint = sp;
        taskEXIT_CRITICAL();
    }

    void resetSetpoint() {
        taskENTER_CRITICAL();
        for (int i = 0; i < 6; ++i) {
            current_setpoint.pos_world[i] = 0.0f;
            current_setpoint.vel_body[i] = 0.0f;
            current_setpoint.thrust_body[i] = 0.0f;
        }
        taskEXIT_CRITICAL();
    }

    RawSetpoint getRawSetpoint() const {
        RawSetpoint sp;
        taskENTER_CRITICAL();
        sp = raw_setpoint;
        taskEXIT_CRITICAL();
        return sp;
    }

    void setRawSetpoint(const RawSetpoint& sp) {
        taskENTER_CRITICAL();
        raw_setpoint = sp;
        taskEXIT_CRITICAL();
    }

    float getLastDtMs() const {
        float dt;
        taskENTER_CRITICAL();
        dt = last_dt_ms;
        taskEXIT_CRITICAL();
        return dt;
    }

    void setLastDtMs(float dt) {
        taskENTER_CRITICAL();
        last_dt_ms = dt;
        taskEXIT_CRITICAL();
    }

    uint32_t getLastReceivedSeq() const {
        uint32_t seq;
        taskENTER_CRITICAL();
        seq = last_received_seq;
        taskEXIT_CRITICAL();
        return seq;
    }

    void setLastReceivedSeq(uint32_t seq) {
        taskENTER_CRITICAL();
        last_received_seq = seq;
        taskEXIT_CRITICAL();
    }

    std::array<float, 6> getLastOutputForces() const {
        std::array<float, 6> forces;
        taskENTER_CRITICAL();
        for (int i = 0; i < 6; ++i) {
            forces[i] = last_output_forces[i];
        }
        taskEXIT_CRITICAL();
        return forces;
    }

    void setLastOutputForces(const std::array<float, 6>& forces) {
        taskENTER_CRITICAL();
        for (int i = 0; i < 6; ++i) {
            last_output_forces[i] = forces[i];
        }
        taskEXIT_CRITICAL();
    }

    // 坐标偏置变量及设置接口
    bool use_offset_ = false;
    float offset_x_ = 0.0f;
    float offset_y_ = 0.0f;
    float offset_z_ = 0.0f;
    float offset_roll_ = 0.0f;
    float offset_pitch_ = 0.0f;
    float offset_yaw_ = 0.0f;

    void setHomeOffset(float x, float y, float z, float roll, float pitch, float yaw);
    void clearHomeOffset();

private:
    NavState nav_state{};               ///< 实时真实位姿速度状态
    TargetSetpoint current_setpoint{};  ///< 当前控制目标设定值
    RawSetpoint raw_setpoint{};         ///< 原始 AGX 设定值快照
    float last_dt_ms = 0.0f;
    uint32_t last_received_seq = 0;
    float last_output_forces[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
};

extern MotionContext motion_context;

} // namespace motion
} // namespace auv

#endif // __MOTION_CONTEXT_HPP
