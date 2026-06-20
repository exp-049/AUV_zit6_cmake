#pragma once
#include "SystemConfig.hpp"
#include <array>
#include <cmath>

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
  void step(const std::array<float, 6> &forces,
            const std::array<float, 6> &masses,
            const std::array<float, 6> &drags, float k) {

    // 1. 机体系加速度计算 (F_net = F_thrust - F_drag)
    for (int i = 0; i < 6; i++) {
      float m = (masses[i] > 0.00001f) ? masses[i] : 20.0f;
      float push = forces[i] * k;
      float resist = velocity_[i] * drags[i];
      float accel = (push - resist) / m;
      velocity_[i] += accel * dt_;
    }

    // 2. 用 6DOF 旋转矩阵更新位姿
    float roll = position_[3];
    float pitch = position_[4];
    float yaw = position_[5];
    float cφ = std::cos(roll), sφ = std::sin(roll);
    float cθ = std::cos(pitch), sθ = std::sin(pitch);
    float cψ = std::cos(yaw), sψ = std::sin(yaw);

    // 世界系速度 = R_ZYX · 机体系速度
    float world_vx = (cψ * cθ) * velocity_[0] +
                     (cψ * sθ * sφ - sψ * cφ) * velocity_[1] +
                     (cψ * sθ * cφ + sψ * sφ) * velocity_[2];
    float world_vy = (sψ * cθ) * velocity_[0] +
                     (sψ * sθ * sφ + cψ * cφ) * velocity_[1] +
                     (sψ * sθ * cφ - cψ * sφ) * velocity_[2];
    float world_vz = (-sθ) * velocity_[0] + (cθ * sφ) * velocity_[1] +
                     (cθ * cφ) * velocity_[2];

    position_[0] += world_vx * dt_;
    position_[1] += world_vy * dt_;
    position_[2] += world_vz * dt_;
    // 欧拉角速率积分（简化：小角度近似，实际应使用 T 矩阵）
    position_[3] += velocity_[3] * dt_; // Roll
    position_[4] += velocity_[4] * dt_; // Pitch
    position_[5] += velocity_[5] * dt_; // Yaw

    // 角度归一化
    for (int i = 3; i < 6; i++) {
      while (position_[i] > 3.14159265f)
        position_[i] -= 6.2831853f;
      while (position_[i] < -3.14159265f)
        position_[i] += 6.2831853f;
    }
  }

  /**
   * @brief 推进物理引擎一步 (自动从全局配置中加载物理参数)
   * @param forces 6-DOF 归一化控制力矢量 [Fx, Fy, Fz, Mroll, Mpitch, Myaw]
   */
  void step(const std::array<float, 6> &forces) {
    float k = auv::config::sys_config.simulation.thrust_k;
    float sim_m = auv::config::sys_config.simulation.mass;
    float sim_d = auv::config::sys_config.simulation.drag;

    std::array<float, 6> masses = {sim_m,        sim_m,        sim_m,
                                   sim_m * 0.1f, sim_m * 0.1f, sim_m * 0.1f};
    std::array<float, 6> drags = {sim_d,        sim_d,        sim_d,
                                  sim_d * 0.1f, sim_d * 0.1f, sim_d * 0.1f};
    step(forces, masses, drags, k);
  }

  const std::array<float, 6> &getPosition() const { return position_; }
  const std::array<float, 6> &getVelocity() const { return velocity_; }

  void reset(const float p[6]) {
    for (int n = 0; n < 6; n++) {
      position_[n] = p[n];
      velocity_[n] = 0.0f;
    }
  }

private:
  float dt_;
  std::array<float, 6> position_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  std::array<float, 6> velocity_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
};

} // namespace control
} // namespace auv
