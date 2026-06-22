/**
 * @file test_kinematic.cpp
 * @brief KinematicProfile 主机端单元测试
 *
 * 测试策略：
 * - 影子平滑器到达目标位置/速度
 * - 物理约束（max_v, max_a）被遵守
 * - 无扰动状态对齐
 * - 边界条件（零限幅、小目标、远距离）
 */

#include "KinematicProfile.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>

namespace algo = auv::algorithm::control;

// ============================================================================
// 位置追踪
// ============================================================================

TEST(KinematicProfile, ReachesTargetPosition) {
  algo::KinematicProfile prof;
  prof.setLimits(1.0f, 0.5f);
  prof.align(0.0f, 0.0f);

  algo::ProfileState s;
  for (int i = 0; i < 500; i++)
    s = prof.updatePosition(2.0f, 0.01f);

  EXPECT_NEAR(s.p, 2.0f, 0.01f);   // 到达目标
  EXPECT_NEAR(s.v, 0.0f, 0.001f);  // 末端速度为零
  EXPECT_NEAR(s.a, 0.0f, 0.001f);  // 末端加速度为零
}

TEST(KinematicProfile, ReachesTargetFromNegative) {
  algo::KinematicProfile prof;
  prof.setLimits(1.0f, 0.5f);
  prof.align(-2.0f, 0.0f);

  algo::ProfileState s;
  for (int i = 0; i < 500; i++)
    s = prof.updatePosition(0.0f, 0.01f);

  EXPECT_NEAR(s.p, 0.0f, 0.01f);
  EXPECT_NEAR(s.v, 0.0f, 0.001f);
}

// ============================================================================
// 物理约束
// ============================================================================

TEST(KinematicProfile, VelocityLimitObeyed) {
  algo::KinematicProfile prof;
  prof.setLimits(0.5f, 10.0f);  // 强加速度，弱限速
  prof.align(0.0f, 0.0f);

  float max_v = 0.0f;
  for (int i = 0; i < 500; i++) {
    auto s = prof.updatePosition(10.0f, 0.01f);
    max_v = std::max(max_v, std::abs(s.v));
  }

  EXPECT_LE(max_v, 0.5f + 0.001f);
}

TEST(KinematicProfile, AccelerationLimitObeyed) {
  algo::KinematicProfile prof;
  prof.setLimits(10.0f, 0.2f);  // 弱加速度，强限速
  prof.align(0.0f, 0.0f);

  float max_a = 0.0f;
  for (int i = 0; i < 500; i++) {
    auto s = prof.updatePosition(10.0f, 0.01f);
    max_a = std::max(max_a, std::abs(s.a));
  }

  EXPECT_LE(max_a, 0.2f + 0.001f);
}

// ============================================================================
// 无扰动切换
// ============================================================================

TEST(KinematicProfile, BumplessAlign) {
  algo::KinematicProfile prof;
  prof.setLimits(1.0f, 1.0f);
  prof.align(5.0f, 0.5f);

  // align 后立即检查，状态已对齐
  auto s = prof.getState();
  EXPECT_FLOAT_EQ(s.p, 5.0f);   // 起点对齐到 5.0
  EXPECT_FLOAT_EQ(s.v, 0.5f);   // 速度平滑延续

  // 演进一步后应平滑离开起点
  s = prof.updatePosition(5.5f, 0.01f);
  EXPECT_GT(s.p, 5.0f);         // 位置向前移动
  EXPECT_GT(s.v, 0.0f);         // 速度大于零
}

// ============================================================================
// 速度追踪
// ============================================================================

TEST(KinematicProfile, VelocityTracking) {
  algo::KinematicProfile prof;
  prof.setLimits(2.0f, 1.0f);
  prof.align(0.0f, 0.0f);

  auto s = prof.updateVelocity(1.0f, 0.01f);
  EXPECT_GT(s.v, 0.0f);                // 开始加速
  EXPECT_LE(std::abs(s.v), 1.0f);      // 不超过目标

  // 200 步后应接近目标速度
  for (int i = 0; i < 200; i++)
    s = prof.updateVelocity(1.0f, 0.01f);
  EXPECT_NEAR(s.v, 1.0f, 0.01f);
}

TEST(KinematicProfile, VelocityLimitObeyedInVelMode) {
  algo::KinematicProfile prof;
  prof.setLimits(0.3f, 5.0f);
  prof.align(0.0f, 0.0f);

  for (int i = 0; i < 100; i++)
    prof.updateVelocity(10.0f, 0.01f);

  auto s = prof.getState();
  EXPECT_LE(std::abs(s.v), 0.3f + 0.001f);
}

// ============================================================================
// 边界条件
// ============================================================================

TEST(KinematicProfile, ZeroDtNoChange) {
  algo::KinematicProfile prof;
  prof.setLimits(1.0f, 1.0f);
  prof.align(0.0f, 0.0f);

  auto s = prof.updatePosition(5.0f, 0.0f);
  EXPECT_FLOAT_EQ(s.p, 0.0f);
  EXPECT_FLOAT_EQ(s.v, 0.0f);
}

TEST(KinematicProfile, AlreadyAtTarget) {
  algo::KinematicProfile prof;
  prof.setLimits(1.0f, 1.0f);
  prof.align(3.0f, 0.0f);

  auto s = prof.updatePosition(3.0f, 0.01f);
  EXPECT_FLOAT_EQ(s.p, 3.0f);
  EXPECT_FLOAT_EQ(s.v, 0.0f);
}

TEST(KinematicProfile, NegativeLimitsHandled) {
  algo::KinematicProfile prof;
  prof.setLimits(-1.0f, -0.5f);  // 负数应被取绝对值
  prof.align(0.0f, 0.0f);

  EXPECT_GE(prof.getMaxV(), 0.0f);
  EXPECT_GE(prof.getMaxA(), 0.0f);
}
