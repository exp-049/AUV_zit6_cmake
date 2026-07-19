/**
 * @file test_math_utils.cpp
 * @brief MathUtils 主机端单元测试
 *
 * 测试策略：
 * - 正反变换一致性（applyRotationToWorld ↔ applyRotationToBody）
 * - 旋转矩阵正交性（R * Rᵀ = I）
 * - 欧拉角变换矩阵可逆性（T * T⁻¹ = I）
 * - 边界角度（零角度、万向锁附近）
 */

#include "MathUtils.hpp"
#include <gtest/gtest.h>
#include <cmath>

namespace math = auv::algorithm::math;

// ============================================================================
// 正反变换一致性
// ============================================================================

TEST(MathUtils, ForwardInverseConsistency) {
  float roll = 0.3f, pitch = -0.1f, yaw = 1.2f;

  float body[6] = {1.0f, 2.0f, -0.5f, 0.1f, -0.2f, 0.3f};
  float world[6], back[6];

  math::applyRotationToWorld(body, world, roll, pitch, yaw);
  math::applyRotationToBody(world, back, roll, pitch, yaw);

  for (int i = 0; i < 6; i++)
    EXPECT_NEAR(back[i], body[i], 1e-5f);
}

TEST(MathUtils, InverseForwardConsistency) {
  float roll = -0.8f, pitch = 0.4f, yaw = 2.5f;

  float world[6] = {-3.0f, 1.5f, 0.0f, 0.5f, -0.1f, 0.0f};
  float body[6], back[6];

  math::applyRotationToBody(world, body, roll, pitch, yaw);
  math::applyRotationToWorld(body, back, roll, pitch, yaw);

  for (int i = 0; i < 6; i++)
    EXPECT_NEAR(back[i], world[i], 1e-5f);
}

TEST(MathUtils, WrenchUsesRotationForForceAndMoment) {
  float world[6] = {1.0f, 2.0f, 3.0f, -0.4f, 0.5f, 0.6f};
  float body[6], expected[6];
  auto full_rotation = math::eulerToRotationMatrix(0.3f, -0.2f, 0.7f);
  auto R = full_rotation.topLeftCorner<3, 3>();
  Eigen::Map<const Eigen::Matrix<float, 3, 1>> force_world(world);
  Eigen::Map<const Eigen::Matrix<float, 3, 1>> moment_world(world + 3);
  Eigen::Map<Eigen::Matrix<float, 3, 1>> force_expected(expected);
  Eigen::Map<Eigen::Matrix<float, 3, 1>> moment_expected(expected + 3);
  force_expected = R.transpose() * force_world;
  moment_expected = R.transpose() * moment_world;

  math::applyWrenchToBody(world, body, 0.3f, -0.2f, 0.7f);
  for (int i = 0; i < 6; ++i)
    EXPECT_NEAR(body[i], expected[i], 1e-5f);
}

// ============================================================================
// 旋转矩阵正交性: R * Rᵀ = I
// ============================================================================

TEST(MathUtils, RotationMatrixOrthogonal) {
  auto R = math::eulerToRotationMatrix(0.5f, -0.2f, 1.0f);
  auto R_inv = math::eulerToRotationMatrixInverse(0.5f, -0.2f, 1.0f);

  // R * R_inv ≈ I
  auto I = R * R_inv;

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      float expected = (i == j) ? 1.0f : 0.0f;
      EXPECT_NEAR(I(i, j), expected, 1e-5f);
    }
  }
}

// ============================================================================
// 零角度变换 = 恒等变换
// ============================================================================

TEST(MathUtils, ZeroAngleIsIdentity) {
  float body[6] = {1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f};
  float world[6];

  math::applyRotationToWorld(body, world, 0.0f, 0.0f, 0.0f);

  for (int i = 0; i < 6; i++)
    EXPECT_NEAR(world[i], body[i], 1e-6f);
}

TEST(MathUtils, IdentityRotationMatrix) {
  auto R = math::eulerToRotationMatrix(0.0f, 0.0f, 0.0f);

  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
      EXPECT_NEAR(R(i, j), (i == j) ? 1.0f : 0.0f, 1e-6f);
}

// ============================================================================
// 特定角度变换
// ============================================================================

TEST(MathUtils, RollOnly) {
  float body[6] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  float world[6];

  // 绕 X 轴旋转 90° = π/2
  math::applyRotationToWorld(body, world, M_PI_2, 0.0f, 0.0f);

  // 线速度部分：X 轴不变
  EXPECT_NEAR(world[0], 1.0f, 1e-5f);
  EXPECT_NEAR(world[1], 0.0f, 1e-5f);
  EXPECT_NEAR(world[2], 0.0f, 1e-5f);
}

// ============================================================================
// 万向锁附近（pitch ≈ ±90°）
// ============================================================================

TEST(MathUtils, NearGimbalLock) {
  float roll = 0.0f, pitch = M_PI_2 - 0.001f, yaw = 0.0f;

  // 应能正常计算，不崩溃
  auto R = math::eulerToRotationMatrix(roll, pitch, yaw);
  auto R_inv = math::eulerToRotationMatrixInverse(roll, pitch, yaw);

  // 正交性仍应保持
  auto I = R * R_inv;
  EXPECT_NEAR(I(0, 0), 1.0f, 1e-3f);
  EXPECT_NEAR(I(3, 3), 1.0f, 1e-3f);
}

// ============================================================================
// 单位向量测试
// ============================================================================

TEST(MathUtils, XAxisRotation) {
  // 仅 X 方向前进，绕 Z 转 90° → Y 方向
  float body[6] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  float world[6];

  math::applyRotationToWorld(body, world, 0.0f, 0.0f, M_PI_2);

  EXPECT_NEAR(world[0], 0.0f, 1e-5f);
  EXPECT_NEAR(world[1], 1.0f, 1e-5f);
  EXPECT_NEAR(world[2], 0.0f, 1e-5f);
}

// ============================================================================
// 数值稳定性：大角度值
// ============================================================================

TEST(MathUtils, LargeAngleValues) {
  // 多圈旋转应等价于模 2π
  float body[6] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  float world1[6], world2[6];

  math::applyRotationToWorld(body, world1, 0.0f, 0.0f, M_PI_2);
  math::applyRotationToWorld(body, world2, 0.0f, 0.0f, M_PI_2 + 2 * M_PI);

  for (int i = 0; i < 6; i++)
    EXPECT_NEAR(world1[i], world2[i], 1e-5f);
}

// ============================================================================
// 零向量变换
// ============================================================================

TEST(MathUtils, ZeroVectorRemainsZero) {
  float body[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  float world[6];

  math::applyRotationToWorld(body, world, 0.5f, -0.3f, 1.0f);

  for (int i = 0; i < 6; i++)
    EXPECT_FLOAT_EQ(world[i], 0.0f);
}
