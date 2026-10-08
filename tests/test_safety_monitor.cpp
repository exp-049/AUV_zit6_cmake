/**
 * @file test_safety_monitor.cpp
 * @brief SafetyMonitor 状态机主机端单元测试
 *
 * 测试策略：
 * - 解锁条件检测（心跳计数 + 持续时间 + 导航状态）
 * - 心跳超时自动上锁
 * - 上锁后控制层级归零
 * - 仿真模式绕过导航检查
 * - 长时间无心跳清零
 */

#include "SafetyMonitor.hpp"
#include "MotionContext.hpp"
#include "SystemConfig.hpp"
#include "SystemContext.hpp"
#include "ChassisManager.hpp"
#include "main.h"
#include <gtest/gtest.h>
#include <limits>

namespace component = auv::component;
namespace motion   = auv::motion;
namespace config   = auv::config;

// ============================================================================
// 测试夹具
// ============================================================================

class SafetyMonitorTest : public ::testing::Test {
protected:
  void SetUp() override {
    // 将系统置于初始状态
    auv::system::system_context = auv::system::SystemContext{};
    auv::motion::motion_context = auv::motion::MotionContext{};
    config::sys_config = config::SystemConfig{};

    // 使用仿真模式绕过导航硬件检查
    config::sys_config.simulation.sitl_enabled = true;

    // 重置全局 HAL tick
    setHALTick(0);
    motion::NavState raw{};
    raw.pos_world = {10.0f, 20.0f, -5.0f, 0.1f, -0.2f, 1.5f};
    motion::motion_context.updateNavigationSnapshot(raw, 0, true);
    motion::OriginCommit commit;
    ASSERT_TRUE(motion::motion_context.trySetOrigin(0, 200, commit));
  }

  // 辅助：快速构造 context
  auv::system::AppContext makeContext() {
    auv::system::AppContext ctx;
    ctx.chassis = &mock_chassis_;
    return ctx;
  }

  // 辅助：注入心跳事件（模拟上位机发送心跳）
  void injectHeartbeat(auv::system::SystemContext &sys, uint32_t now_ms,
                       uint32_t data = 1) {
    auto a = sys.arm_state_.get();
    a.last_heartbeat_ms = now_ms;
    a.last_heartbeat_data = data;
    a.heartbeat_count++;
    if (a.start_ms == 0)
      a.start_ms = now_ms;
    sys.arm_state_.set(a);
  }

  // Host tests pass the same monotonic tick the MCU monitor reads and keep
  // simulated navigation samples fresh unless a case explicitly tests staleness.
  void checkAt(component::SafetyMonitor &sm, uint32_t now_ms) {
    setHALTick(now_ms);
    auto raw = motion::motion_context.raw_nav_snapshot_.get();
    motion::motion_context.updateNavigationSnapshot(
        raw.raw_nav, now_ms, raw.nav_valid);
    sm.check(now_ms);
  }

  component::ChassisManager mock_chassis_;
};

// ============================================================================
// 1. 未解锁状态 — 不应自动解锁
// ============================================================================

TEST_F(SafetyMonitorTest, StaysDisarmedWithoutHeartbeats) {
  auto ctx = makeContext();
  component::SafetyMonitor sm(&ctx);

  checkAt(sm, 1000);    // 1 秒时检查
  checkAt(sm, 2000);    // 2 秒时检查

  auto a = auv::system::system_context.arm_state_.get();
  EXPECT_FALSE(a.is_armed);
  EXPECT_EQ(mock_chassis_.getControlLevel(), motion::ControlLevel::NONE);
}

// ============================================================================
// 2. 心跳足够但时间不够 → 不解锁
// ============================================================================

TEST_F(SafetyMonitorTest, DoesNotArmTooEarly) {
  auto ctx = makeContext();
  component::SafetyMonitor sm(&ctx);

  // 快速发送 20 个心跳，但不到 1 秒
  for (int i = 1; i <= 20; i++)
    injectHeartbeat(auv::system::system_context, i * 10, 1);

  checkAt(sm, 200);

  auto a = auv::system::system_context.arm_state_.get();
  EXPECT_FALSE(a.is_armed);
}

// ============================================================================
// 3. 心跳足够 + 时间足够 → 解锁
// ============================================================================

TEST_F(SafetyMonitorTest, ArmsWhenConditionsMet) {
  auto ctx = makeContext();
  component::SafetyMonitor sm(&ctx);

  // 发送 >10 个心跳，每 50ms 一个，总耗时 >1s
  for (int i = 1; i <= 15; i++)
    injectHeartbeat(auv::system::system_context, i * 50, 1);

  // 模拟用时 1200ms
  checkAt(sm, 1200);

  auto a = auv::system::system_context.arm_state_.get();
  EXPECT_TRUE(a.is_armed);
}

// ============================================================================
// 4. 心跳超时 → 自动上锁
// ============================================================================

TEST_F(SafetyMonitorTest, DisarmsOnHeartbeatTimeout) {
  auto ctx = makeContext();
  component::SafetyMonitor sm(&ctx);

  // 先解锁
  for (int i = 1; i <= 15; i++)
    injectHeartbeat(auv::system::system_context, i * 50, 1);
  checkAt(sm, 1200);

  EXPECT_TRUE(auv::system::system_context.arm_state_.get().is_armed);

  // 停止心跳超过 1s → 触发超时上锁
  checkAt(sm, 1800);  // 距上次心跳 1800-750=1050ms > 1000ms

  auto a = auv::system::system_context.arm_state_.get();
  EXPECT_FALSE(a.is_armed);
  EXPECT_EQ(a.heartbeat_count, 0u);
}

// ============================================================================
// 4b. 2Hz 心跳（500ms 周期）不应被误判为超时
// ============================================================================

TEST_F(SafetyMonitorTest, KeepsArmedAtTwoHz) {
  auto ctx = makeContext();
  component::SafetyMonitor sm(&ctx);

  for (int i = 1; i <= 15; i++)
    injectHeartbeat(auv::system::system_context, i * 500, 3);
  checkAt(sm, 8000);
  EXPECT_TRUE(auv::system::system_context.arm_state_.get().is_armed);

  injectHeartbeat(auv::system::system_context, 8500, 3);
  checkAt(sm, 9000);  // 恰好 500ms，仍属于 2Hz 心跳
  EXPECT_TRUE(auv::system::system_context.arm_state_.get().is_armed);
}

// ============================================================================
// 5. 上锁后控制层级归零
// ============================================================================

TEST_F(SafetyMonitorTest, SetsControlLevelNoneOnDisarm) {
  auto ctx = makeContext();
  component::SafetyMonitor sm(&ctx);

  // 先解锁
  for (int i = 1; i <= 15; i++)
    injectHeartbeat(auv::system::system_context, i * 50, 1);
  checkAt(sm, 1200);
  EXPECT_TRUE(auv::system::system_context.arm_state_.get().is_armed);

  // 模拟控制层级被设置为非 NONE
  mock_chassis_.setMockLevel(motion::ControlLevel::VELOCITY);

  // 心跳超时 → 应自动上锁并重置控制层级
  setHALTick(2000);
  checkAt(sm, 2000);

  EXPECT_FALSE(auv::system::system_context.arm_state_.get().is_armed);
  EXPECT_EQ(mock_chassis_.getControlLevel(), motion::ControlLevel::NONE);
}

// ============================================================================
// 6. 远程解锁模式 (heartbeat_data == 3)
// ============================================================================

TEST_F(SafetyMonitorTest, RemoteArmMode) {
  auto ctx = makeContext();
  component::SafetyMonitor sm(&ctx);

  // data=3 表示远程解锁指令，不需要导航有效
  config::sys_config.simulation.sitl_enabled = false;

  for (int i = 1; i <= 15; i++)
    injectHeartbeat(auv::system::system_context, i * 50, 3);
  checkAt(sm, 1200);

  auto a = auv::system::system_context.arm_state_.get();
  EXPECT_TRUE(a.is_armed);
}

// ============================================================================
// 7. 长时间未收到心跳 → 清零计数
// ============================================================================

TEST_F(SafetyMonitorTest, HeartbeatCounterResetsOnLongIdle) {
  auto ctx = makeContext();
  component::SafetyMonitor sm(&ctx);

  // 先发几个心跳
  injectHeartbeat(auv::system::system_context, 100, 1);
  injectHeartbeat(auv::system::system_context, 200, 1);
  auto a = auv::system::system_context.arm_state_.get();
  EXPECT_EQ(a.heartbeat_count, 2u);

  // 超过 kDisarmedHeartbeatTimeoutMs(1000) 未收到心跳
  setHALTick(2000);
  checkAt(sm, 2000);

  a = auv::system::system_context.arm_state_.get();
  EXPECT_EQ(a.heartbeat_count, 0u);
}

// ============================================================================
// 8. 未解锁状态下，若控制层级不为 NONE → 强制归零
// ============================================================================

TEST_F(SafetyMonitorTest, ResetsControlLevelWhenDisarmed) {
  auto ctx = makeContext();
  component::SafetyMonitor sm(&ctx);

  // 模拟控制层级被意外设置
  mock_chassis_.setMockLevel(motion::ControlLevel::POSITION);

  checkAt(sm, 100);

  EXPECT_EQ(mock_chassis_.getControlLevel(), motion::ControlLevel::NONE);
}

// ============================================================================
// 9. 解锁后设置 home offset
// ============================================================================

TEST_F(SafetyMonitorTest, ArmPreservesExplicitOrigin) {
  auto ctx = makeContext();
  component::SafetyMonitor sm(&ctx);

  // Move raw navigation away from the explicitly captured origin before ARM.
  motion::NavState raw{};
  raw.pos_world = {99.0f, 50.0f, 9.0f, 0.1f, -0.2f, -0.5f};
  motion::motion_context.updateNavigationSnapshot(raw, 0, true);

  // 触发解锁
  for (int i = 1; i <= 15; i++)
    injectHeartbeat(auv::system::system_context, i * 50, 1);
  checkAt(sm, 1200);

  // 验证 home offset
  auto home = auv::motion::motion_context.home_offset_.get();
  EXPECT_TRUE(home.active);
  EXPECT_FLOAT_EQ(home.offset[0], 10.0f);
  EXPECT_FLOAT_EQ(home.offset[1], 20.0f);
  EXPECT_FLOAT_EQ(home.offset[2], -5.0f);
  EXPECT_FLOAT_EQ(home.offset[3], 0.0f);  // Roll 强制为 0
  EXPECT_FLOAT_EQ(home.offset[4], 0.0f);  // Pitch 强制为 0
  EXPECT_FLOAT_EQ(home.offset[5], 1.5f);  // Yaw
}

// ============================================================================
// 10. 上锁后清除 home offset
// ============================================================================

TEST_F(SafetyMonitorTest, DisarmPreservesExplicitOrigin) {
  auto ctx = makeContext();
  component::SafetyMonitor sm(&ctx);

  // 先解锁
  for (int i = 1; i <= 15; i++)
    injectHeartbeat(auv::system::system_context, i * 50, 1);
  checkAt(sm, 1200);
  EXPECT_TRUE(auv::system::system_context.arm_state_.get().is_armed);

  // 心跳超时 → 上锁 → 清除 offset
  setHALTick(2000);
  checkAt(sm, 2000);

  auto home = auv::motion::motion_context.home_offset_.get();
  EXPECT_TRUE(home.active);
  EXPECT_EQ(motion::motion_context.getOdomSnapshot().origin_generation, 1U);
}

TEST_F(SafetyMonitorTest, AllArmModesRequireOriginInCurrentBoot) {
  auto ctx = makeContext();
  component::SafetyMonitor sm(&ctx);
  for (uint32_t mode : {1U, 3U}) {
    motion::motion_context = motion::MotionContext{};
    motion::motion_context.updateNavigationSnapshot(motion::NavState{}, 0, true);
    auv::system::system_context.arm_state_.set({});
    for (int i = 1; i <= 15; ++i)
      injectHeartbeat(auv::system::system_context, i * 50, mode);
    checkAt(sm, 1200);
    EXPECT_FALSE(auv::system::system_context.arm_state_.get().is_armed);
  }
}

TEST_F(SafetyMonitorTest, StaleNavigationDeniesNormalArm) {
  auto ctx = makeContext();
  component::SafetyMonitor sm(&ctx);
  for (int i = 0; i < 15; ++i)
    injectHeartbeat(auv::system::system_context, 9500 + i * 100, 1);
  setHALTick(11000); // navigation sample is older than NAV_VALID_MAX_AGE_MS
  sm.check(11000);
  EXPECT_FALSE(auv::system::system_context.arm_state_.get().is_armed);
}

TEST_F(SafetyMonitorTest, ExpiredHeartbeatsCannotArm) {
  auto ctx = makeContext();
  component::SafetyMonitor sm(&ctx);
  for (int i = 1; i <= 15; ++i)
    injectHeartbeat(auv::system::system_context, i * 50, 3);
  checkAt(sm, 3000);
  EXPECT_FALSE(auv::system::system_context.arm_state_.get().is_armed);
}

TEST_F(SafetyMonitorTest, NewOriginRequiresFreshHeartbeatQualification) {
  auto ctx = makeContext();
  component::SafetyMonitor sm(&ctx);
  auto &sys = auv::system::system_context;
  for (int i = 1; i <= 15; ++i)
    injectHeartbeat(sys, i * 50, 3);
  // The successful setorigin callback invalidates this old qualification.
  sys.resetArmingQualification();
  auto arm = sys.arm_state_.get();
  EXPECT_EQ(arm.heartbeat_count, 0U);
  EXPECT_EQ(arm.start_ms, 0U);
  EXPECT_EQ(arm.last_heartbeat_ms, 750U);
  EXPECT_EQ(arm.last_heartbeat_data, 3U);
  checkAt(sm, 1200);
  EXPECT_FALSE(sys.arm_state_.get().is_armed);
  for (int i = 0; i < 10; ++i)
    injectHeartbeat(sys, 1300 + i * 100, 3);
  checkAt(sm, 2200);  // new qualification has not lasted 1 second yet
  EXPECT_FALSE(sys.arm_state_.get().is_armed);
  checkAt(sm, 2400);
  EXPECT_TRUE(sys.arm_state_.get().is_armed);
}

TEST(MotionOrigin, CapturesRawNavAndCommitsOdomWithVersion) {
  motion::MotionContext context;
  motion::NavState raw{};
  raw.pos_world = {10.0f, 20.0f, 3.0f, .1f, -.2f, 1.5707963268f};
  raw.vel_body = {1.0f, 2.0f, 3.0f, .2f, .3f, .4f};
  context.updateNavigationSnapshot(raw, 123, true);
  motion::OriginCommit first;
  ASSERT_TRUE(context.trySetOrigin(130, 200, first));
  EXPECT_EQ(first.origin_generation, 1U);
  EXPECT_EQ(first.nav_timestamp_ms, 123U);
  EXPECT_EQ(first.origin_nav, (std::array<float,6>{10,20,3,0,0,1.5707963268f}));
  auto odom = context.getOdomSnapshot();
  EXPECT_TRUE(odom.origin_initialized);
  EXPECT_EQ(odom.origin_generation, first.origin_generation);
  EXPECT_FLOAT_EQ(odom.nav_state.pos_world[0], 0);
  EXPECT_FLOAT_EQ(odom.nav_state.pos_world[1], 0);
  EXPECT_FLOAT_EQ(odom.nav_state.pos_world[2], 0);
  EXPECT_FLOAT_EQ(odom.nav_state.pos_world[3], .1f);
  EXPECT_FLOAT_EQ(odom.nav_state.pos_world[4], -.2f);
  EXPECT_FLOAT_EQ(odom.nav_state.pos_world[5], 0);
  EXPECT_EQ(odom.nav_state.vel_body, raw.vel_body);
  raw.pos_world[0] += 1;
  raw.pos_world[1] += 2;
  context.updateNavigationSnapshot(raw, 140, true);
  odom = context.getOdomSnapshot();
  EXPECT_NEAR(odom.nav_state.pos_world[0], 2, 1e-6);
  EXPECT_NEAR(odom.nav_state.pos_world[1], -1, 1e-6);
  motion::OriginCommit second;
  ASSERT_TRUE(context.trySetOrigin(140, 200, second));
  EXPECT_EQ(second.origin_generation, 2U);
  // Repeated reset must capture raw nav, never the already shifted odom.
  EXPECT_FLOAT_EQ(second.origin_nav[0], 11);
  EXPECT_FLOAT_EQ(second.origin_nav[1], 22);
  EXPECT_FLOAT_EQ(context.getOdomSnapshot().nav_state.pos_world[0], 0);
}

TEST(MotionOrigin, RejectsMissingInvalidStaleAndNonfiniteSamples) {
  motion::MotionContext context;
  motion::OriginCommit commit;
  EXPECT_FALSE(context.trySetOrigin(0, 200, commit));
  motion::NavState raw{};
  context.updateNavigationSnapshot(raw, 100, false);
  EXPECT_FALSE(context.trySetOrigin(100, 200, commit));
  context.updateNavigationSnapshot(raw, 100, true);
  EXPECT_FALSE(context.trySetOrigin(301, 200, commit));
  raw.pos_world[0] = std::numeric_limits<float>::quiet_NaN();
  context.updateNavigationSnapshot(raw, 301, true);
  EXPECT_FALSE(context.trySetOrigin(301, 200, commit));
  EXPECT_FALSE(context.getOdomSnapshot().origin_initialized);
  EXPECT_EQ(context.getOdomSnapshot().origin_generation, 0U);
}

TEST(MotionOrigin, RequiresOneValidAbsoluteDepthSource) {
  motion::MotionContext context;
  motion::OriginCommit commit;
  motion::NavState raw{};

  // Fresh INS navigation alone is insufficient when no z source is available.
  context.updateNavigationSnapshot(raw, 100, true, false);
  EXPECT_FALSE(context.trySetOrigin(100, 200, commit));

  // A single absolute z source is enough; x/y/yaw still come from INS.
  context.updateNavigationSnapshot(raw, 110, true, true);
  EXPECT_TRUE(context.trySetOrigin(110, 200, commit));
}

TEST(MotionOrigin, SampleAgeHandlesMillisecondWraparound) {
  motion::MotionContext context;
  context.updateNavigationSnapshot(motion::NavState{}, UINT32_MAX - 10, true);
  motion::OriginCommit commit;
  EXPECT_TRUE(context.trySetOrigin(20, 200, commit));
}

TEST_F(SafetyMonitorTest, LateMonitorCannotArmFromExpiredHeartbeat) {
  auto ctx = makeContext();
  component::SafetyMonitor sm(&ctx);
  for (int i = 1; i <= 15; ++i)
    injectHeartbeat(auv::system::system_context, i * 50, 3);
  // The monitor callback arrived late; an old captured tick would have armed.
  setHALTick(2500);
  sm.check(1200);
  EXPECT_FALSE(auv::system::system_context.arm_state_.get().is_armed);
  auto arm = auv::system::system_context.arm_state_.get();
  EXPECT_EQ(arm.heartbeat_count, 0U);
  EXPECT_EQ(arm.last_heartbeat_ms, 750U);
}

TEST_F(SafetyMonitorTest, QualificationResetPreservesNewHeartbeat) {
  auv::system::ArmState snapshot{};
  snapshot.last_heartbeat_ms = 100;
  snapshot.last_heartbeat_data = 3;
  snapshot.heartbeat_count = 4;
  snapshot.start_ms = 50;
  injectHeartbeat(auv::system::system_context, 200, 3);

  EXPECT_FALSE(auv::system::system_context
                   .resetArmingQualificationIfUnchanged(snapshot));
  const auto current = auv::system::system_context.arm_state_.get();
  EXPECT_EQ(current.heartbeat_count, 1U);
  EXPECT_EQ(current.last_heartbeat_ms, 200U);
}
