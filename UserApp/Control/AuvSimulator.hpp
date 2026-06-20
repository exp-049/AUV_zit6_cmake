#pragma once
#include "MathUtils.hpp"
#include "SystemConfig.hpp"
#include <array>
#include <cmath>

namespace auv {
namespace control {

/**
 * @class AuvSimulator
 * @brief 运行在单片机内部的物理引擎 (HITL 模式)
 *
 * 物理模型：
 * - Fossen 6-DOF 运动学：η̇ = R(η) · ν（使用完整 T 矩阵，非小角度近似）
 * - 动力学：F = M·a + D_lin·v + D_quad·|v|·v + g_restore
 * - 浮力恢复力矩：基于稳心高 GM，模拟重心/浮心分离的被动稳定
 * - 二次阻力：真实水下阻力特性
 */
class AuvSimulator {
public:
  AuvSimulator(float dt = 0.01f) : dt_(dt) {
    const auto &sim = auv::config::sys_config.simulation;
    const float r2 = 0.09f;
    masses_ = {sim.mass, sim.mass, sim.mass,
               sim.mass * r2, sim.mass * r2, sim.mass * r2};
    drags_ = {sim.drag, sim.drag, sim.drag,
              sim.drag * 0.15f, sim.drag * 0.15f, sim.drag * 0.15f};
    drag_quad_ = {sim.drag * 0.3f, sim.drag * 0.3f, sim.drag * 0.3f,
                  sim.drag * 0.05f, sim.drag * 0.05f, sim.drag * 0.05f};
  }

  /**
   * @brief 推进物理引擎一步
   * @param forces 归一化推力 [-1.0, 1.0] (Body frame)
   */
  void step(const std::array<float, 6> &forces) {
    const auto &m = masses_;
    const auto &d = drags_;
    const auto &dq = drag_quad_;

    // 从物理配置加载每轴最大力/力矩
    // normalize 输出 [-1,1] 对应真实推进器的物理最大输出
    //   X/Y/Z: 最大推力 (N)
    //   Roll/Pitch/Yaw: 最大力矩 (N·m)
    // 这些值应与实际推进器配置匹配
    const auto &sim = auv::config::sys_config.simulation;
    static constexpr float kMaxForce[6] = {
        1000, 1000, 1000,  // X,Y,Z 最大推力 (N)
         200,  200,  200}; // Roll,Pitch,Yaw 最大力矩 (N·m)

    // 重力与浮力
    const float g = 9.81f;
    const float weight = m[0] * g;

    // 1. 机体系加速度计算
    float accel[6];
    for (int i = 0; i < 6; i++) {
      // 归一化 [-1,1] → 物理力/力矩（量纲正确）
      float push = forces[i] * kMaxForce[i];
      float drag_lin  = velocity_[i] * d[i];
      float drag_quad = dq[i] * std::abs(velocity_[i]) * velocity_[i];
      float restore = 0.0f;
      if (i == 3) restore = -weight * sim.metacentric_height * std::sin(position_[3]) * std::cos(position_[4]);
      if (i == 4) restore = -weight * sim.metacentric_height * std::cos(position_[3]) * std::sin(position_[4]);
      const float m_eff = (m[i] > 1e-6f) ? m[i] : (i < 3 ? 20.0f : 1.0f);
      accel[i] = (push - drag_lin - drag_quad + restore) / m_eff;
    }

    // 2. 速度积分
    for (int i = 0; i < 6; i++) velocity_[i] += accel[i] * dt_;

    // 3. 运动学更新：η̇ = R(η) · ν
    auv::math::applyRotationToWorld(velocity_.data(), eta_dot_.data(),
                                     position_[3], position_[4], position_[5]);
    for (int i = 0; i < 6; i++) position_[i] += eta_dot_[i] * dt_;

    // 角度归一化
    for (int i = 3; i < 6; i++) {
      if (position_[i] > auv::math::kPi) position_[i] -= auv::math::kTwoPi;
      else if (position_[i] < -auv::math::kPi) position_[i] += auv::math::kTwoPi;
    }
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
  std::array<float, 6> masses_{35.0f, 35.0f, 35.0f, 5.0f, 5.0f, 5.0f};
  std::array<float, 6> drags_{15.0f, 15.0f, 15.0f, 2.0f, 2.0f, 2.0f};
  std::array<float, 6> drag_quad_{4.5f, 4.5f, 4.5f, 0.6f, 0.6f, 0.6f};
  std::array<float, 6> position_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  std::array<float, 6> velocity_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  std::array<float, 6> eta_dot_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
};

} // namespace control
} // namespace auv
