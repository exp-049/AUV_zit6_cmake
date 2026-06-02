/**
 * @file KinematicProfile.hpp
 * @brief 影子平滑器 (Kinematic Profile Generator)
 * 
 * 职责：
 * 1. 为每个控制轴维护一个“理想影子状态”。
 * 2. 接收阶跃或不平滑的目标点，通过物理限制 (max_v, max_a) 计算平滑的演进曲线。
 * 3. 产出三阶连续的状态量 [P_d, V_d, A_d]，作为底层 PID 和前馈控制的参考输入。
 * 4. 采用“理想制动曲线算法”，确保系统能以最大加速度精准停在目标点而不超调。
 */

#pragma once

#include <cmath>
#include <algorithm>

namespace auv {
namespace control {

/**
 * @struct ProfileState
 * @brief 一维运动学状态组
 */
struct ProfileState {
    float p = 0.0f; ///< 期望位置 (Position Target)
    float v = 0.0f; ///< 期望速度 (Velocity Reference / Feedforward)
    float a = 0.0f; ///< 期望加速度 (Acceleration Feedforward)
};

/**
 * @class KinematicProfile
 * @brief 物理边界约束下的轨迹生成器
 */
class KinematicProfile {
public:
    /**
     * @struct Limits
     * @brief 物理运动约束边界
     */
    struct Limits {
        float max_v = 0.5f; ///< 最大平移速度 (m/s) 或 旋转角速度 (rad/s)
        float max_a = 0.2f; ///< 最大瞬态加速度 (m/s^2) 或 角加速度 (rad/s^2)
    };

    KinematicProfile() = default;
    
    /**
     * @brief 设置物理运行边界
     */
    void setLimits(float max_v, float max_a);
    
    /**
     * @brief 获取当前物理运行边界
     */
    float getMaxV() const { return limits_.max_v; }
    float getMaxA() const { return limits_.max_a; }

    /**
     * @brief 100Hz 演进核心算法
     * 
     * 算法步骤：
     * 1. 基于剩余距离 delta_p，计算当前时刻在 a_max 限制下不超调的“最大允许速度”。
     * 2. 对该速度进行 max_v 截断，得到目标速度指令 v_cmd。
     * 3. 根据 (v_cmd - v_current) 计算所需加速度，并进行 a_max 截断。
     * 4. 对加速度和速度进行数值积分，更新影子状态。
     * 
     * @param target_p 最终期望停止的目标位置
     * @param dt 积分步长 (0.01s)
     * @return 最新的演进状态
     */
    ProfileState updatePosition(float target_p, float dt);

    /**
     * @brief 速度跟踪演进
     * @param target_v 目标期望速度
     * @param dt 步长
     * @return 最新的演进状态
     */
    ProfileState updateVelocity(float target_v, float dt);

    /**
     * @brief 无扰动状态对齐
     * @param actual_p 当前传感器的真实位置值
     * @param actual_v 当前传感器的真实速度值
     * 
     * 用于从 VELOCITY 模式切回 POSITION 模式时，将影子的起点强行拉回到现实，消除瞬时偏差。
     */
    void align(float actual_p, float actual_v);

    /**
     * @brief 获取当前影子的状态参考
     */
    const ProfileState& getState() const;

private:
    ProfileState state_; ///< 当前演进中的影子状态
    Limits limits_;      ///< 设定的物理边界
};

} // namespace control
} // namespace auv
