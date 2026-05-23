#pragma once
#include <array>
#include <cmath>
#include "SystemConfig.hpp"

namespace auv {
namespace control {

/**
 * @class AuvSimulator
 * @brief 运行在单片机内部的物理引擎 (HITL 模式)
 * 旁路传感器数据，而代之以数学模型。
 */
class AuvSimulator {
public:
    AuvSimulator(float dt = 0.01f) : dt_(dt) {}

    /**
     * @brief 推进物理引擎一步
     * @param forces 归一化推力 [-1.0, 1.0] (Body frame)
     * @param masses 各轴质量数组 [mx, my, mz, myaw]
     * @param drags 各轴阻力数组 [dx, dy, dz, dyaw]
     * @param k 推力增益 (1.0 推力对应多少牛顿)
     */
    void step(const std::array<float, 4>& forces, 
              const std::array<float, 4>& masses, 
              const std::array<float, 4>& drags, 
              float k) {
        
        // 1. 机体系加速度计算 (F_net = F_thrust - F_drag)
        for (int i = 0; i < 4; i++) {
            float m = (masses[i] > 0.00001f) ? masses[i] : 20.0f;
            float push = forces[i] * k;
            float resist = velocity_[i] * drags[i];
            float accel = (push - resist) / m;
            velocity_[i] += accel * dt_;
        }

        // 2. 将机体系速度转为世界系速度并更新位置
        float yaw = position_[3];
        float cos_y = std::cos(yaw);
        float sin_y = std::sin(yaw);

        float world_vx = velocity_[0] * cos_y - velocity_[1] * sin_y;
        float world_vy = velocity_[0] * sin_y + velocity_[1] * cos_y;

        position_[0] += world_vx * dt_;
        position_[1] += world_vy * dt_;
        position_[2] += velocity_[2] * dt_; // Z 轴
        position_[3] += velocity_[3] * dt_; // Yaw 轴

        // 航向角归一化 [-PI, PI]
        while (position_[3] > 3.14159265f) position_[3] -= 6.2831853f;
        while (position_[3] < -3.14159265f) position_[3] += 6.2831853f;
    }

    /**
     * @brief 推进物理引擎一步 (自动从全局配置中加载物理参数)
     * @param forces 4-DOF 归一化控制力矢量
     */
    void step(const std::array<float, 4>& forces) {
        float k = auv::config::sys_config.simulation.thrust_k;
        float sim_m = auv::config::sys_config.simulation.mass;
        float sim_d = auv::config::sys_config.simulation.drag;

        // 对各轴分配合理的物理质量与阻力，避免使用非常小的前馈质量导致发散
        std::array<float, 4> masses = {sim_m, sim_m, sim_m, sim_m * 0.1f};
        std::array<float, 4> drags = {sim_d, sim_d, sim_d, sim_d * 0.1f};
        step(forces, masses, drags, k);
    }

    const std::array<float, 4>& getPosition() const { return position_; }
    
    /**
     * @brief 获取机体系速度 (Surge, Sway, Heave, YawRate)
     */
    const std::array<float, 4>& getVelocity() const { return velocity_; }
    
    void reset(const float p[4]) {
        for(int n=0; n<4; n++) { 
            position_[n] = p[n]; 
            velocity_[n] = 0.0f; 
        }
    }

private:
    float dt_;
    std::array<float, 4> position_{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, 4> velocity_{0.0f, 0.0f, 0.0f, 0.0f};
};

} // namespace control
} // namespace auv
