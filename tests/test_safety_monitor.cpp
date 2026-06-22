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
#include "main.h"
#include <gtest/gtest.h>

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

  component::ChassisManager mock_chassis_;
};

// ============================================================================
// 1. 未解锁状态 — 不应自动解锁
// ============================================================================

TEST_F(SafetyMonitorTest, StaysDisarmedWithoutHeartbeats) {
  auto ctx = makeContext();
  component::SafetyMonitor sm(&ctx);

  sm.check(1000);    // 1 秒时检查
  sm.check(2000);    // 2 秒时检查

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

  sm.check(200);

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
  sm.check(1200);

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
  sm.check(1200);

  EXPECT_TRUE(auv::system::system_context.arm_state_.get().is_armed);

  // 停止心跳 600ms → 超过 500ms 超时阈值
  sm.check(1800);  // 距上次心跳 1800-750=1050ms > 500ms

  auto a = auv::system::system_context.arm_state_.get();
  EXPECT_FALSE(a.is_armed);
  EXPECT_EQ(a.heartbeat_count, 0u);
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
  sm.check(1200);
  EXPECT_TRUE(auv::system::system_context.arm_state_.get().is_armed);

  // 模拟控制层级被设置为非 NONE
  mock_chassis_.setMockLevel(motion::ControlLevel::VELOCITY);

  // 心跳超时 → 应自动上锁并重置控制层级
  setHALTick(2000);
  sm.check(2000);

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
  sm.check(1200);

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
  sm.check(2000);

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

  sm.check(100);

  EXPECT_EQ(mock_chassis_.getControlLevel(), motion::ControlLevel::NONE);
}

// ============================================================================
// 9. 解锁后设置 home offset
// ============================================================================

TEST_F(SafetyMonitorTest, SetsHomeOffsetOnArm) {
  auto ctx = makeContext();
  component::SafetyMonitor sm(&ctx);

  // 初始化位置
  {
    auto ns = auv::motion::motion_context.nav_state_.get();
    ns.pos_world = {10.0f, 20.0f, -5.0f, 0.1f, -0.2f, 1.5f};
    auv::motion::motion_context.nav_state_.set(ns);
  }

  // 触发解锁
  for (int i = 1; i <= 15; i++)
    injectHeartbeat(auv::system::system_context, i * 50, 1);
  sm.check(1200);

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

TEST_F(SafetyMonitorTest, ClearsHomeOffsetOnDisarm) {
  auto ctx = makeContext();
  component::SafetyMonitor sm(&ctx);

  // 先解锁
  for (int i = 1; i <= 15; i++)
    injectHeartbeat(auv::system::system_context, i * 50, 1);
  sm.check(1200);
  EXPECT_TRUE(auv::system::system_context.arm_state_.get().is_armed);

  // 心跳超时 → 上锁 → 清除 offset
  setHALTick(2000);
  sm.check(2000);

  auto home = auv::motion::motion_context.home_offset_.get();
  EXPECT_FALSE(home.active);
}
