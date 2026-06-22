/**
 * @file test_soft_watchdog.cpp
 * @brief SoftWatchdog 主机端单元测试
 *
 * 测试策略：
 * - 初始化后所有组件时间戳一致
 * - 喂狗更新特定组件时间戳
 * - 超时检测
 * - 禁用的组件不触发超时
 */

#include "SoftWatchdog.hpp"
#include "main.h"
#include <gtest/gtest.h>

namespace component = auv::component;

class SoftWatchdogTest : public ::testing::Test {
protected:
  void SetUp() override {
    setHALTick(1000);  // 从 1000ms 开始
    watchdog_.init(default_cfg_);
  }

  void advance(uint32_t ms) {
    advanceHALTick(ms);
  }

  auv::config::SoftWatchdogConfig default_cfg_{
      .timeout_ms = 500,
      .check_microros = true,
      .check_ins = true,
      .check_depth = true
  };

  component::SoftWatchdog watchdog_;
};

// ============================================================================
// 初始化后所有组件处于正常状态
// ============================================================================

TEST_F(SoftWatchdogTest, InitAllOk) {
  EXPECT_TRUE(watchdog_.check());
}

// ============================================================================
// 超时检测 — 未喂狗超过 timeout
// ============================================================================

TEST_F(SoftWatchdogTest, TimeoutDetected) {
  advance(600);  // 超过 500ms 阈值
  // 喂一次狗让时间戳更新到当前，然后等超时
  watchdog_.feed(component::SoftWatchdog::Component::MICROROS);
  watchdog_.feed(component::SoftWatchdog::Component::INS);
  watchdog_.feed(component::SoftWatchdog::Component::DEPTH);
  advance(600);
  EXPECT_FALSE(watchdog_.check());
}

// ============================================================================
// 喂狗后超时重置
// ============================================================================

TEST_F(SoftWatchdogTest, FeedResetsTimeout) {
  // Init 时已喂过，等待 300ms 仍在超时内
  advance(300);
  EXPECT_TRUE(watchdog_.check());

  // 再喂，重置时间戳
  watchdog_.feed(component::SoftWatchdog::Component::MICROROS);
  watchdog_.feed(component::SoftWatchdog::Component::INS);
  watchdog_.feed(component::SoftWatchdog::Component::DEPTH);

  advance(300);  // 从喂狗起 300ms，还差 200ms
  EXPECT_TRUE(watchdog_.check());

  advance(300);  // 从喂狗起 600ms > 500ms
  EXPECT_FALSE(watchdog_.check());
}

// ============================================================================
// 只喂一个组件 → 其他组件超时
// ============================================================================

TEST_F(SoftWatchdogTest, PartialFeedTriggersTimeout) {
  // 先喂所有，然后只喂一个
  watchdog_.feed(component::SoftWatchdog::Component::MICROROS);
  watchdog_.feed(component::SoftWatchdog::Component::INS);
  watchdog_.feed(component::SoftWatchdog::Component::DEPTH);
  advance(600);
  watchdog_.feed(component::SoftWatchdog::Component::MICROROS);
  // INS 和 DEPTH 未喂 → 超时
  EXPECT_FALSE(watchdog_.check());
}

// ============================================================================
// 禁用某个组件的检查
// ============================================================================

TEST_F(SoftWatchdogTest, DisabledComponentSkipped) {
  default_cfg_.check_ins = false;
  default_cfg_.check_depth = false;
  watchdog_.init(default_cfg_);

  advance(600);
  watchdog_.feed(component::SoftWatchdog::Component::MICROROS);
  // INS 和 DEPTH 虽然超时但被禁用 → 只有 MICROROS 被检查
  EXPECT_TRUE(watchdog_.check());
}

// ============================================================================
// 全部禁用 → 永远返回 true
// ============================================================================

TEST_F(SoftWatchdogTest, AllDisabledAlwaysOk) {
  default_cfg_.check_microros = false;
  default_cfg_.check_ins = false;
  default_cfg_.check_depth = false;
  watchdog_.init(default_cfg_);

  advance(100000);  // 很久以后
  EXPECT_TRUE(watchdog_.check());
}

// ============================================================================
// 超时前一刻返回 true
// ============================================================================

TEST_F(SoftWatchdogTest, ExactlyAtLimit) {
  advance(499);  // 499ms < 500ms
  EXPECT_TRUE(watchdog_.check());
}

// ============================================================================
// 重新 init 后超时重置
// ============================================================================

TEST_F(SoftWatchdogTest, ReInitResetsTimestamps) {
  watchdog_.feed(component::SoftWatchdog::Component::MICROROS);
  watchdog_.feed(component::SoftWatchdog::Component::INS);
  watchdog_.feed(component::SoftWatchdog::Component::DEPTH);
  advance(600);
  EXPECT_FALSE(watchdog_.check());

  // 重新初始化 → 时间戳重置
  setHALTick(2000);
  watchdog_.init(default_cfg_);
  EXPECT_TRUE(watchdog_.check());
}
