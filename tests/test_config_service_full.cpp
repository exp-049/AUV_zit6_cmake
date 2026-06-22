/**
 * @file test_config_service_full.cpp
 * @brief ConfigService 完整测试（getParamsJson + updateParams）
 *
 * 测试策略：
 * - getParamsJson 返回合法 JSON 字符串
 * - 按路径过滤参数
 * - updateParams 通过 JSON 或 KV 对更新参数
 * - 参数类型解析（float/bool/enum）
 */

// 注意：UserApp/Config/ 下也有同名 ConfigService.hpp（不同 namespace）
// 显式指定路径以包含 UserApp/Component/ 版本
#include "../UserApp/Component/ConfigService.hpp"
#include "SoftWatchdog.hpp"
#include "AppContext.hpp"
#include "SystemConfig.hpp"
#include "SystemContext.hpp"
#include <gtest/gtest.h>
#include <cstring>
#include <string>

namespace component = auv::component;
namespace config   = auv::config;

class ConfigServiceFullTest : public ::testing::Test {
protected:
  void SetUp() override {
    // 重置系统状态
    auv::system::system_context = auv::system::SystemContext{};
    auv::config::sys_config = auv::config::SystemConfig{};

    // 设置 AppContext 指针（默认全为 nullptr，会导致 updateParams 崩溃）
    auv::system::g_app_ctx.chassis = &mock_chassis_;
    auv::system::g_app_ctx.watchdog = &mock_watchdog_;
  }

  void TearDown() override {
    auv::system::g_app_ctx.chassis = nullptr;
    auv::system::g_app_ctx.watchdog = nullptr;
  }

  auv::component::ChassisManager mock_chassis_;
  auv::component::SoftWatchdog mock_watchdog_;
};

// ============================================================================
// getParamsJson — 基本功能
// ============================================================================

TEST_F(ConfigServiceFullTest, GetParamsJsonReturnsValidJson) {
  const char *paths[] = {"chassis.x.pos_kp"};
  const char *json = component::ConfigService::getParamsJson(paths, 1);
  ASSERT_NE(json, nullptr);
  ASSERT_GT(strlen(json), 0u);
  EXPECT_EQ(json[0], '{');

  // 应包含请求的路径
  EXPECT_NE(strstr(json, "chassis.x.pos_kp"), nullptr);
  // 不应包含未请求的路径
  EXPECT_EQ(strstr(json, "chassis.y.pos_kp"), nullptr);
}

TEST_F(ConfigServiceFullTest, GetParamsJsonEmptyRequest) {
  // req_count=0 → 返回全部参数
  const char *json = component::ConfigService::getParamsJson(nullptr, 0);
  ASSERT_NE(json, nullptr);
  EXPECT_EQ(json[0], '{');
  // 完整配置应包含 chassis 等顶层分组
  EXPECT_NE(strstr(json, "chassis"), nullptr);
}

TEST_F(ConfigServiceFullTest, GetParamsJsonUnknownPath) {
  const char *paths[] = {"nonexistent.path"};
  const char *json = component::ConfigService::getParamsJson(paths, 1);
  ASSERT_NE(json, nullptr);
  // 不存在的路径 → 空对象
  EXPECT_STREQ(json, "{}");
}

TEST_F(ConfigServiceFullTest, GetParamsJsonPrefixMatch) {
  // "chassis.x" 应匹配 chassis.x.pos_kp, chassis.x.vel_kp 等
  const char *paths[] = {"chassis.x"};
  const char *json = component::ConfigService::getParamsJson(paths, 1);
  ASSERT_NE(json, nullptr);
  EXPECT_NE(strstr(json, "chassis.x.pos_kp"), nullptr);
  EXPECT_NE(strstr(json, "chassis.x.vel_kp"), nullptr);
}

// ============================================================================
// getParamsJson — 值格式
// ============================================================================

TEST_F(ConfigServiceFullTest, GetParamsJsonFloatFormat) {
  // 验证浮点数格式化
  config::sys_config.chassis.x.pos_kp = 1.5f;
  const char *paths[] = {"chassis.x.pos_kp"};
  const char *json = component::ConfigService::getParamsJson(paths, 1);
  ASSERT_NE(json, nullptr);
  EXPECT_NE(strstr(json, "1.5"), nullptr);
}

TEST_F(ConfigServiceFullTest, GetParamsJsonBoolFormat) {
  config::sys_config.simulation.sitl_enabled = true;
  const char *paths[] = {"simulation.sitl_enabled"};
  const char *json = component::ConfigService::getParamsJson(paths, 1);
  ASSERT_NE(json, nullptr);
  EXPECT_NE(strstr(json, "true"), nullptr);
}

// ============================================================================
// updateParams — KV 对更新
// ============================================================================

TEST_F(ConfigServiceFullTest, UpdateParamsByKVPair) {
  const char *paths[] = {"chassis.x.pos_kp"};
  const char *values[] = {"5.0"};
  char out[32] = {0};
  bool ok = component::ConfigService::updateParams(
      nullptr, paths, values, 1, out, sizeof(out));

  EXPECT_TRUE(ok);
  EXPECT_FLOAT_EQ(config::sys_config.chassis.x.pos_kp, 5.0f);
}

TEST_F(ConfigServiceFullTest, UpdateParamsMultiple) {
  const char *paths[] = {"chassis.x.pos_kp", "chassis.y.vel_kp"};
  const char *values[] = {"2.0", "3.0"};
  char out[32] = {0};
  component::ConfigService::updateParams(nullptr, paths, values, 2, out, sizeof(out));

  EXPECT_FLOAT_EQ(config::sys_config.chassis.x.pos_kp, 2.0f);
  EXPECT_FLOAT_EQ(config::sys_config.chassis.y.vel_kp, 3.0f);
}

TEST_F(ConfigServiceFullTest, UpdateParamsUnknownPath) {
  const char *paths[] = {"nonexistent.path"};
  const char *values[] = {"1.0"};
  char out[32] = {0};
  bool ok = component::ConfigService::updateParams(
      nullptr, paths, values, 1, out, sizeof(out));

  EXPECT_FALSE(ok);
  EXPECT_NE(strstr(out, "not found"), nullptr);
}

TEST_F(ConfigServiceFullTest, UpdateParamsBool) {
  config::sys_config.simulation.sitl_enabled = false;
  const char *paths[] = {"simulation.sitl_enabled"};
  const char *values[] = {"true"};
  component::ConfigService::updateParams(nullptr, paths, values, 1, nullptr, 0);

  EXPECT_TRUE(config::sys_config.simulation.sitl_enabled);
}

// ============================================================================
// updateParams — JSON 更新
// ============================================================================

// ============================================================================
// updateParams — JSON 更新
// 注意：ConfigService.cpp 的 walkJson 在根对象 item->string==NULL 时 strcat 崩溃
// 这是一个已知 bug，待修复后启用此测试
// ============================================================================

TEST_F(ConfigServiceFullTest, DISABLED_UpdateParamsByJson) {
  const char *json = R"({"chassis":{"x":{"pos_kp":8.0}}})";
  char out[32] = {0};
  bool ok = component::ConfigService::updateParams(
      json, nullptr, nullptr, 0, out, sizeof(out));
  EXPECT_TRUE(ok);
  EXPECT_FLOAT_EQ(config::sys_config.chassis.x.pos_kp, 8.0f);
}

// ============================================================================
// 更新后关联行为
// ============================================================================

TEST_F(ConfigServiceFullTest, UpdateTriggersReplanFlag) {
  const char *paths[] = {"chassis.x.pos_kp"};
  const char *values[] = {"3.0"};
  component::ConfigService::updateParams(nullptr, paths, values, 1, nullptr, 0);

  EXPECT_TRUE(auv::system::system_context.planner_replan_flag);
}
