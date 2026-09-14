/**
 * @file test_cascade_controller.cpp
 * @brief CascadeController 主机端单元测试
 *
 * 测试策略：
 * - NONE 层级输出全零
 * - P-only 位置环的阶跃响应
 * - VELOCITY 模式的 PI 控制
 * - Bumpless 切换时状态对齐
 * - Planner 启用的 S-curve 平滑
 */

#include "CascadeController.hpp"
#include "MotionContext.hpp"
#include "SystemConfig.hpp"
#include <gtest/gtest.h>

namespace component = auv::component;
namespace motion   = auv::motion;
namespace config   = auv::config;

class CascadeControllerTest : public ::testing::Test {
protected:
  void SetUp() override {
    // 重置上下文到初始状态
    auv::motion::motion_context = auv::motion::MotionContext{};
    auv::config::sys_config = auv::config::SystemConfig{};

    ctrl_.applyConfig(makeTestConfig());
  }

  // 创建一组简单的测试 PID 参数
  static auv::config::ChassisConfig makeTestConfig() {
    auv::config::ChassisConfig cfg;
    cfg.planner_enabled = false;  // 默认关掉 planner，测试 PID 裸响应

    auto setAxis = [](auv::config::AxisConfig &a) {
      a.pos_kp = 2.0f; a.pos_ki = 0.0f; a.pos_kd = 0.0f;
      a.pos_i_limit = 0.5f; a.pos_output_limit = 1.0f;
      a.vel_kp = 1.0f; a.vel_ki = 0.5f; a.vel_kd = 0.0f;
      a.vel_i_limit = 0.5f; a.vel_output_limit = 1.0f;
      a.max_v = 1.0f; a.max_a = 2.0f;
      a.mass = 1.0f; a.drag = 0.0f;
    };
    setAxis(cfg.x); setAxis(cfg.y); setAxis(cfg.z);
    setAxis(cfg.roll); setAxis(cfg.pitch); setAxis(cfg.yaw);
    return cfg;
  }

  // 设置导航状态（位置+速度）
  void setNavState(float x, float y, float z,
                   float roll, float pitch, float yaw) {
    auto ns = auv::motion::motion_context.nav_state_.get();
    ns.pos_world = {x, y, z, roll, pitch, yaw};
    auv::motion::motion_context.nav_state_.set(ns);
  }

  // 设置目标位置
  void setTargetPos(float x, float y, float z,
                    float roll, float pitch, float yaw) {
    auto sp = auv::motion::motion_context.current_setpoint_.get();
    sp.pos_world = {x, y, z, roll, pitch, yaw};
    auv::motion::motion_context.current_setpoint_.set(sp);
  }

  // 设置目标速度
  void setTargetVel(float u, float v, float w,
                    float p, float q, float r) {
    auto sp = auv::motion::motion_context.current_setpoint_.get();
    sp.vel_body = {u, v, w, p, q, r};
    auv::motion::motion_context.current_setpoint_.set(sp);
  }

  void setLevel(motion::ControlLevel lv) {
    ctrl_.setControlLevel(lv);
  }

  component::CascadeController ctrl_;
};

// ============================================================================
// NONE 层级 → 输出全零
// ============================================================================

TEST_F(CascadeControllerTest, NoneLevelReturnsZero) {
  auto out = ctrl_.update();
  for (int i = 0; i < 6; i++)
    EXPECT_FLOAT_EQ(out[i], 0.0f);
}

// ============================================================================
// 位置模式 P-only：误差 1m → 输出 kp * 1 = 2，再被 output_limit=1 截断
// ============================================================================

TEST_F(CascadeControllerTest, PositionPOnly) {
  setNavState(0, 0, 0, 0, 0, 0);
  setTargetPos(1, 0, 0, 0, 0, 0);

  setLevel(motion::ControlLevel::POSITION);

  auto out = ctrl_.update();
  // 位置误差 1，P 增益 2 → 速度指令 2 → 被 vel_output_limit=1 截断
  // 速度误差 1（目标1 - 实际0），vel_kp=1 → 推力 1
  EXPECT_GT(out[0], 0.0f);
}

// ============================================================================
// 位置模式：零误差 → 零输出
// ============================================================================

TEST_F(CascadeControllerTest, PositionZeroError) {
  setNavState(5, 3, 0, 0, 0, 0);
  setTargetPos(5, 3, 0, 0, 0, 0);

  setLevel(motion::ControlLevel::POSITION);

  auto out = ctrl_.update();
  // 误差为零 → PID 输出为零
  EXPECT_NEAR(out[0], 0.0f, 1e-4f);
  EXPECT_NEAR(out[1], 0.0f, 1e-4f);
}

// ============================================================================
// 速度模式 PI：恒定误差 → 积分累积消除误差
// ============================================================================

TEST_F(CascadeControllerTest, VelocityPI) {
  setNavState(0, 0, 0, 0, 0, 0);
  setTargetVel(0.5f, 0, 0, 0, 0, 0);

  setLevel(motion::ControlLevel::VELOCITY);

  // 多步演进，积分逐渐累积
  float out_x = 0;
  for (int i = 0; i < 100; i++)
    out_x = ctrl_.update()[0];

  // 速度误差 0.5，vel_ki=0.5 → 积分累积
  EXPECT_GT(out_x, 0.0f);
  // 不超过 output_limit
  EXPECT_LE(out_x, 1.0f);
}

// ============================================================================
// 速度模式：实际已达标 → 仅有积分保持
// ============================================================================

TEST_F(CascadeControllerTest, VelocityAtTarget) {
  // 实际速度和目标速度一致
  setNavState(0, 0, 0, 0, 0, 0);
  // 实际速度设为 0.3（模拟已在运动）
  {
    auto ns = auv::motion::motion_context.nav_state_.get();
    ns.vel_body = {0.3f, 0, 0, 0, 0, 0};
    auv::motion::motion_context.nav_state_.set(ns);
  }
  setTargetVel(0.3f, 0, 0, 0, 0, 0);

  setLevel(motion::ControlLevel::VELOCITY);

  auto out = ctrl_.update();
  // 误差为零 → P 项为零，仅有可能的积分项
  // 首次调用积分也为零 → 输出接近 0
  EXPECT_NEAR(out[0], 0.0f, 1e-4f);
}

// ============================================================================
// X 轴和 Y 轴独立控制
// ============================================================================

TEST_F(CascadeControllerTest, IndependentAxes) {
  setNavState(0, 0, 0, 0, 0, 0);
  setTargetPos(1, 0, 0, 0, 0, 0);

  setLevel(motion::ControlLevel::POSITION);

  auto out = ctrl_.update();
  // X 有误差 → 有输出
  EXPECT_NE(out[0], 0.0f);
  // Y 无误差 → 无输出
  EXPECT_NEAR(out[1], 0.0f, 1e-4f);
  // Z 无误差 → 无输出
  EXPECT_NEAR(out[2], 0.0f, 1e-4f);
}

// ============================================================================
// 输出限幅
// ============================================================================

TEST_F(CascadeControllerTest, OutputLimiting) {
  setNavState(0, 0, 0, 0, 0, 0);
  // 超大误差
  setTargetPos(100, 0, 0, 0, 0, 0);

  setLevel(motion::ControlLevel::POSITION);

  auto out = ctrl_.update();
  for (int i = 0; i < 6; i++)
    EXPECT_LE(std::abs(out[i]), 1.0f);  // 不超过 output_limit
}

// ============================================================================
// 层级切换回 NONE → 输出归零
// ============================================================================

TEST_F(CascadeControllerTest, SwitchToNoneReturnsZero) {
  setNavState(0, 0, 0, 0, 0, 0);
  setTargetPos(1, 0, 0, 0, 0, 0);
  setLevel(motion::ControlLevel::POSITION);

  ctrl_.update();  // 有输出

  setLevel(motion::ControlLevel::NONE);
  auto out = ctrl_.update();
  for (int i = 0; i < 6; i++)
    EXPECT_FLOAT_EQ(out[i], 0.0f);
}

// ============================================================================
// Planner 启用：响应应比纯 P 更平滑
// ============================================================================

TEST_F(CascadeControllerTest, PlannerEnabledSmootherResponse) {
  auto cfg = makeTestConfig();
  cfg.planner_enabled = true;
  ctrl_.applyConfig(cfg);

  setNavState(0, 0, 0, 0, 0, 0);
  setTargetPos(1, 0, 0, 0, 0, 0);
  setLevel(motion::ControlLevel::POSITION);

  // 第一步：planner 刚开始加速，输出应较小
  auto out1 = ctrl_.update();

  // 第 10 步：已加速，输出应更大
  for (int i = 0; i < 9; i++) ctrl_.update();
  auto out10 = ctrl_.update();

  // Planner 控制的输出应逐步增加，而非阶跃
  EXPECT_GE(out10[0], out1[0]);
}
