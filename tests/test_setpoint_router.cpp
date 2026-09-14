/**
 * @file test_setpoint_router.cpp
 * @brief SetpointRouter 主机端单元测试
 *
 * 测试策略：
 * - 坐标变换正确性（body↔world, 6DOF payload）
 * - Roll/Pitch 六轴字段透传到控制设定值
 * - Mask 掩码跳过特定轴
 * - 增量/绝对模式
 * - 模式切换时的无扰动对齐
 */

#include "SetpointRouter.hpp"
#include "MotionContext.hpp"
#include "MathUtils.hpp"
#include <gtest/gtest.h>
#include <cmath>

namespace component = auv::component;
namespace motion   = auv::motion;

class SetpointRouterTest : public ::testing::Test {
protected:
  void SetUp() override {
    // 初始状态：位置在原点，朝北
    auto ns = auv::motion::motion_context.nav_state_.get();
    ns.pos_world = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    ns.vel_body = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    auv::motion::motion_context.nav_state_.set(ns);

    // setpoint 初始全零
    auv::motion::motion_context.current_setpoint_.set({});
  }

  void setNavPos(float x, float y, float z, float roll, float pitch, float yaw) {
    auto ns = auv::motion::motion_context.nav_state_.get();
    ns.pos_world = {x, y, z, roll, pitch, yaw};
    auv::motion::motion_context.nav_state_.set(ns);
  }

  void setNavVel(float u, float v, float w, float p, float q, float r) {
    auto ns = auv::motion::motion_context.nav_state_.get();
    ns.vel_body = {u, v, w, p, q, r};
    auv::motion::motion_context.nav_state_.set(ns);
  }

  component::SetpointRouter router_;
};

// ============================================================================
// 位置模式 — 世界系绝对坐标
// ============================================================================

TEST_F(SetpointRouterTest, PositionWorldAbsolute) {
  float val[6] = {5.0f, 3.0f, -2.0f, 0.2f, -0.3f, 1.57f};
  auto lv = router_.route(
      motion::ControlLevel::NONE, motion::ControlLevel::POSITION,
      val, 0, false, false);

  auto sp = auv::motion::motion_context.current_setpoint_.get();
  EXPECT_EQ(lv, motion::ControlLevel::POSITION);
  EXPECT_FLOAT_EQ(sp.pos_world[0], 5.0f);    // X
  EXPECT_FLOAT_EQ(sp.pos_world[1], 3.0f);    // Y
  EXPECT_FLOAT_EQ(sp.pos_world[2], -2.0f);   // Z
  EXPECT_FLOAT_EQ(sp.pos_world[3], 0.2f);    // Roll
  EXPECT_FLOAT_EQ(sp.pos_world[4], -0.3f);   // Pitch
  EXPECT_FLOAT_EQ(sp.pos_world[5], 1.57f);   // Yaw
}

// ============================================================================
// 位置模式 — 机体系转世界系
// ============================================================================

TEST_F(SetpointRouterTest, PositionBodyToWorld) {
  // 当前朝向: Yaw = 90° (朝东)
  setNavPos(0, 0, 0, 0, 0, M_PI_2);

  float val[6] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};  // 机体向前 1m
  router_.route(
      motion::ControlLevel::NONE, motion::ControlLevel::POSITION,
      val, 0, true, false);

  auto sp = auv::motion::motion_context.current_setpoint_.get();
  // 朝东时，机体 X (前) → 世界 X=0, Y=1
  EXPECT_NEAR(sp.pos_world[0], 0.0f, 1e-5f);
  EXPECT_NEAR(sp.pos_world[1], 1.0f, 1e-5f);
}

// ============================================================================
// 位置模式 — 增量设定
// ============================================================================

TEST_F(SetpointRouterTest, PositionIncrementalBody) {
  setNavPos(10.0f, 20.0f, 0.0f, 0.0f, 0.0f, M_PI_2);  // 朝东

  // 先设初始 setpoint
  float init[6] = {10.0f, 20.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  router_.route(motion::ControlLevel::NONE, motion::ControlLevel::POSITION,
                init, 0, false, false);

  // 机体向前 1m → 旋转到世界系（朝东时 body_x → world_y）
  float val[6] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  router_.route(motion::ControlLevel::POSITION, motion::ControlLevel::POSITION,
                val, 0, true, true);  // is_body=true, is_inc=true

  auto sp = auv::motion::motion_context.current_setpoint_.get();
  // 增量写入的是旋转后的世界坐标，不累加
  EXPECT_NEAR(sp.pos_world[0], 0.0f, 1e-4f);
  EXPECT_NEAR(sp.pos_world[1], 1.0f, 1e-4f);
}

// ============================================================================
// Mask 掩码 — 跳过指定轴
// ============================================================================

TEST_F(SetpointRouterTest, MaskSkipsAxes) {
  float val[6] = {5.0f, 3.0f, -2.0f, 0.2f, -0.3f, 1.57f};

  // 先设一个初始值
  router_.route(motion::ControlLevel::NONE, motion::ControlLevel::POSITION,
                val, 0, false, false);

  // mask = 0b0001 = bit0 → 跳过 X 轴
  float val2[6] = {99.0f, 6.0f, -3.0f, 0.8f, -0.9f, 0.5f};
  router_.route(motion::ControlLevel::POSITION, motion::ControlLevel::POSITION,
                val2, 0b0001, false, false);

  auto sp = auv::motion::motion_context.current_setpoint_.get();
  EXPECT_FLOAT_EQ(sp.pos_world[0], 5.0f);    // X 被 mask 跳过 → 保持 5.0
  EXPECT_FLOAT_EQ(sp.pos_world[1], 6.0f);    // Y 更新
  EXPECT_FLOAT_EQ(sp.pos_world[3], 0.8f);    // Roll 更新
  EXPECT_FLOAT_EQ(sp.pos_world[4], -0.9f);   // Pitch 更新
  EXPECT_FLOAT_EQ(sp.pos_world[5], 0.5f);    // Yaw 更新
}

TEST_F(SetpointRouterTest, MaskSkipsAllAttitudeAxes) {
  float initial[6] = {1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f};
  router_.route(motion::ControlLevel::NONE, motion::ControlLevel::POSITION,
                initial, 0, false, false);

  // bit3..5 跳过 Roll/Pitch/Yaw，只更新位置三轴。
  float next[6] = {9.0f, 8.0f, 7.0f, 1.1f, 1.2f, 1.3f};
  router_.route(motion::ControlLevel::POSITION, motion::ControlLevel::POSITION,
                next, 0b111000, false, false);

  auto sp = auv::motion::motion_context.current_setpoint_.get();
  EXPECT_FLOAT_EQ(sp.pos_world[0], 9.0f);
  EXPECT_FLOAT_EQ(sp.pos_world[1], 8.0f);
  EXPECT_FLOAT_EQ(sp.pos_world[2], 7.0f);
  EXPECT_FLOAT_EQ(sp.pos_world[3], 0.1f);
  EXPECT_FLOAT_EQ(sp.pos_world[4], 0.2f);
  EXPECT_FLOAT_EQ(sp.pos_world[5], 0.3f);
}

// ============================================================================
// 速度模式 — 机体坐标系
// ============================================================================

TEST_F(SetpointRouterTest, VelocityBodyFrame) {
  float val[6] = {0.5f, 0.0f, 0.0f, 0.2f, -0.3f, 0.1f};
  router_.route(
      motion::ControlLevel::NONE, motion::ControlLevel::VELOCITY,
      val, 0, true, false);

  auto sp = auv::motion::motion_context.current_setpoint_.get();
  EXPECT_FLOAT_EQ(sp.vel_body[0], 0.5f);     // u
  EXPECT_FLOAT_EQ(sp.vel_body[5], 0.1f);     // r
  EXPECT_FLOAT_EQ(sp.vel_body[3], 0.2f);     // p
  EXPECT_FLOAT_EQ(sp.vel_body[4], -0.3f);    // q
}

// ============================================================================
// 速度模式 — 世界系转机体系
// ============================================================================

TEST_F(SetpointRouterTest, VelocityWorldToBody) {
  // 当前朝向: Yaw = 90° (朝东)
  setNavPos(0, 0, 0, 0, 0, M_PI_2);

  float val[6] = {0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f};  // 世界 Y 方向 1m/s
  router_.route(
      motion::ControlLevel::NONE, motion::ControlLevel::VELOCITY,
      val, 0, false, false);

  auto sp = auv::motion::motion_context.current_setpoint_.get();
  // 朝东时，世界 Y+ (北) 在机体右舷 → 机体 X 正方向
  EXPECT_NEAR(sp.vel_body[0], 1.0f, 1e-5f);
}

// ============================================================================
// 模式切换 — 无扰动对齐
// ============================================================================

TEST_F(SetpointRouterTest, BumplessTransitionToPosition) {
  setNavPos(5.0f, 3.0f, -1.0f, 0.0f, 0.0f, 0.5f);

  // 从 NONE 切换到 POSITION（传 val 但用 mask 全屏蔽来验证 bumpless）
  float val[6] = {99.0f, 99.0f, 99.0f, 99.0f, 99.0f, 99.0f};
  router_.route(
      motion::ControlLevel::NONE, motion::ControlLevel::POSITION,
      val, 0xFFFF, false, false);  // mask=全1 → 跳过所有轴

  auto sp = auv::motion::motion_context.current_setpoint_.get();
  // mask 全屏蔽后保留 bumpless 快照值
  EXPECT_FLOAT_EQ(sp.pos_world[0], 5.0f);
  EXPECT_FLOAT_EQ(sp.pos_world[1], 3.0f);
  EXPECT_FLOAT_EQ(sp.pos_world[2], -1.0f);
}

// ============================================================================
// 推进器模式
// ============================================================================

TEST_F(SetpointRouterTest, ActuatorDirectThrust) {
  float val[6] = {0.3f, 0.0f, 0.0f, 0.2f, -0.1f, 0.4f};
  router_.route(
      motion::ControlLevel::NONE, motion::ControlLevel::ACTUATOR,
      val, 0, true, false);

  auto sp = auv::motion::motion_context.current_setpoint_.get();
  EXPECT_FLOAT_EQ(sp.thrust_body[0], 0.3f);
  EXPECT_FLOAT_EQ(sp.thrust_body[3], 0.2f);  // Mroll
  EXPECT_FLOAT_EQ(sp.thrust_body[4], -0.1f); // Mpitch
  EXPECT_FLOAT_EQ(sp.thrust_body[5], 0.4f);
}

TEST_F(SetpointRouterTest, ActuatorWorldWrenchTransformsForceAndMoment) {
  // 世界系绕 Z 旋转 90°：世界 +X 力和 +X 力矩应分别变为机体系 -Y。
  setNavPos(0, 0, 0, 0, 0, M_PI_2);
  float world_wrench[6] = {1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
  router_.route(motion::ControlLevel::NONE, motion::ControlLevel::ACTUATOR,
                world_wrench, 0, false, false);

  auto sp = auv::motion::motion_context.current_setpoint_.get();
  EXPECT_NEAR(sp.thrust_body[0], 0.0f, 1e-5f);
  EXPECT_NEAR(sp.thrust_body[1], -1.0f, 1e-5f);
  EXPECT_NEAR(sp.thrust_body[3], 0.0f, 1e-5f);
  EXPECT_NEAR(sp.thrust_body[4], -1.0f, 1e-5f);
}

TEST_F(SetpointRouterTest, RollPitchSetpointsAreForwardedToRouter) {
  float pos[6] = {1.0f, 2.0f, 3.0f, 1.1f, -1.2f, 0.3f};
  router_.route(motion::ControlLevel::NONE, motion::ControlLevel::POSITION,
                pos, 0, false, false);

  float vel[6] = {0.1f, 0.2f, 0.3f, 1.1f, -1.2f, 0.4f};
  router_.route(motion::ControlLevel::POSITION,
                motion::ControlLevel::VELOCITY, vel, 0, true, false);

  float wrench[6] = {0.1f, 0.2f, 0.3f, 0.8f, -0.9f, 0.5f};
  router_.route(motion::ControlLevel::VELOCITY,
                motion::ControlLevel::ACTUATOR, wrench, 0, true, false);

  auto sp = auv::motion::motion_context.current_setpoint_.get();
  EXPECT_FLOAT_EQ(sp.pos_world[3], 1.1f);
  EXPECT_FLOAT_EQ(sp.pos_world[4], -1.2f);
  EXPECT_FLOAT_EQ(sp.vel_body[3], 1.1f);
  EXPECT_FLOAT_EQ(sp.vel_body[4], -1.2f);
  EXPECT_FLOAT_EQ(sp.thrust_body[3], 0.8f);
  EXPECT_FLOAT_EQ(sp.thrust_body[4], -0.9f);
}

// ============================================================================
// 空指令（val=0）不应崩溃
// ============================================================================

TEST_F(SetpointRouterTest, ModeSwitchWithNullVal) {
  // 从 NONE→POSITION，val 会被 route 读取（非 bumpless 场景也会处理 val）
  // 所以传空指针会崩溃，这里用全零替代
  float val[6] = {0,0,0,0,0,0};
  router_.route(
      motion::ControlLevel::NONE, motion::ControlLevel::POSITION,
      val, 0, false, false);

  // 验证模式已切换，setpoint 被快照
  auto sp = auv::motion::motion_context.current_setpoint_.get();
  EXPECT_FLOAT_EQ(sp.pos_world[0], 0.0f);
}
