/**
 * @file test_pid.cpp
 * @brief PID_Controller 主机端单元测试
 *
 * 测试策略：
 * - 纯 P/PI/PD 控制器的确定性响应
 * - 积分抗饱和 (Anti-windup)
 * - 状态继承与切换
 * - 边界条件（dt <= 0, 零增益）
 */

#include "PID_Controller.hpp"
#include <gtest/gtest.h>

namespace algo = auv::algorithm::control;

// ============================================================================
// 比例项测试
// ============================================================================

TEST(PIDController, POnly_StepResponse) {
  algo::PID_Controller pid;
  algo::PID_Controller::Config cfg;
  cfg.kp = 2.0f;
  cfg.ki = 0.0f;
  cfg.kd = 0.0f;
  cfg.output_limit = 10.0f;
  pid.setConfig(cfg);

  float out = pid.compute(1.5f, 0.01f);
  EXPECT_FLOAT_EQ(out, 3.0f);  // kp * error = 2.0 * 1.5
}

TEST(PIDController, POnly_OutputLimit) {
  algo::PID_Controller pid;
  algo::PID_Controller::Config cfg;
  cfg.kp = 5.0f;
  cfg.ki = 0.0f;
  cfg.kd = 0.0f;
  cfg.output_limit = 1.0f;
  pid.setConfig(cfg);

  float out = pid.compute(10.0f, 0.01f);
  EXPECT_FLOAT_EQ(out, 1.0f);  // 被 output_limit 截断
}

// ============================================================================
// 积分项测试
// ============================================================================

TEST(PIDController, IntegralAccumulates) {
  algo::PID_Controller pid;
  algo::PID_Controller::Config cfg;
  cfg.kp = 0.0f;
  cfg.ki = 1.0f;
  cfg.kd = 0.0f;
  cfg.i_limit = 10.0f;
  cfg.output_limit = 10.0f;
  pid.setConfig(cfg);

  // 恒定误差 2.0，持续 100 步
  for (int i = 0; i < 100; i++)
    pid.compute(2.0f, 0.01f);

  // 积分累加: integral += ki * error * dt = 1.0 * 2.0 * 0.01
  // 100步后 = 2.0
  EXPECT_NEAR(pid.getIntegral(), 2.0f, 1e-4f);
}

TEST(PIDController, AntiWindup) {
  algo::PID_Controller pid;
  algo::PID_Controller::Config cfg;
  cfg.kp = 0.0f;
  cfg.ki = 100.0f;
  cfg.kd = 0.0f;
  cfg.i_limit = 0.5f;
  cfg.output_limit = 10.0f;
  pid.setConfig(cfg);

  // 大误差下积分应被 i_limit 截断
  for (int i = 0; i < 1000; i++)
    pid.compute(10.0f, 0.01f);

  EXPECT_LE(pid.getIntegral(), 0.5f);
  EXPECT_GE(pid.getIntegral(), -0.5f);
}

// ============================================================================
// 微分项测试
// ============================================================================

TEST(PIDController, DTermOnDerivative) {
  algo::PID_Controller pid;
  algo::PID_Controller::Config cfg;
  cfg.kp = 0.0f;
  cfg.ki = 0.0f;
  cfg.kd = 2.0f;
  cfg.output_limit = 10.0f;
  pid.setConfig(cfg);

  // derivative = 0.5
  float out = pid.compute(0.0f, 0.01f, 0.5f);
  EXPECT_FLOAT_EQ(out, 1.0f);  // kd * derivative = 2.0 * 0.5
}

// ============================================================================
// 状态管理测试
// ============================================================================

TEST(PIDController, ResetIntegral) {
  algo::PID_Controller pid;
  algo::PID_Controller::Config cfg;
  cfg.ki = 1.0f;
  cfg.i_limit = 10.0f;
  pid.setConfig(cfg);

  pid.compute(5.0f, 0.01f);
  EXPECT_NE(pid.getIntegral(), 0.0f);

  pid.reset_i();
  EXPECT_FLOAT_EQ(pid.getIntegral(), 0.0f);
}

TEST(PIDController, SetIntegral_Inheritance) {
  algo::PID_Controller pid;
  algo::PID_Controller::Config cfg;
  cfg.ki = 0.0f;
  cfg.i_limit = 5.0f;
  cfg.output_limit = 5.0f;  // 确保 output_limit 不截断积分值
  pid.setConfig(cfg);

  pid.setIntegral(2.5f);
  EXPECT_FLOAT_EQ(pid.getIntegral(), 2.5f);

  // 即使 ki=0，set 的值也应保持（用于模式切换时的状态继承）
  float out = pid.compute(0.0f, 0.01f);
  EXPECT_FLOAT_EQ(out, 2.5f);
}

// ============================================================================
// 边界条件测试
// ============================================================================

TEST(PIDController, ZeroDtReturnsZero) {
  algo::PID_Controller pid;
  algo::PID_Controller::Config cfg;
  cfg.kp = 1.0f;
  cfg.ki = 1.0f;
  cfg.kd = 1.0f;
  pid.setConfig(cfg);

  float out = pid.compute(1.0f, 0.0f);
  EXPECT_FLOAT_EQ(out, 0.0f);
}

TEST(PIDController, NegativeDtReturnsZero) {
  algo::PID_Controller pid;
  algo::PID_Controller::Config cfg;
  cfg.kp = 1.0f;
  pid.setConfig(cfg);

  float out = pid.compute(1.0f, -1.0f);
  EXPECT_FLOAT_EQ(out, 0.0f);
}

TEST(PIDController, AllGainsZero) {
  algo::PID_Controller pid;
  algo::PID_Controller::Config cfg;  // 所有增益默认 0
  pid.setConfig(cfg);

  float out = pid.compute(100.0f, 0.01f);
  EXPECT_FLOAT_EQ(out, 0.0f);
}

// ============================================================================
// PI 控制器稳态性能
// ============================================================================

TEST(PIDController, PI_EliminatesSteadyStateError) {
  algo::PID_Controller pid;
  algo::PID_Controller::Config cfg;
  cfg.kp = 1.0f;
  cfg.ki = 0.5f;
  cfg.i_limit = 10.0f;
  cfg.output_limit = 10.0f;
  pid.setConfig(cfg);

  // 模拟一个恒值干扰：控制器需要积分项消除稳态误差
  float disturbance = 0.3f;
  float setpoint = 1.0f;
  float output = 0.0f;

  for (int i = 0; i < 1000; i++) {
    float error = setpoint - output;
    float control = pid.compute(error, 0.01f);
    // 简单被控对象模型：output += (control - disturbance) * dt
    output += (control - disturbance) * 0.01f;
  }

  // 稳态误差应趋近于 0
  EXPECT_NEAR(setpoint - output, 0.0f, 0.01f);
}
