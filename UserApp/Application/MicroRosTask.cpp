#include "MicroRosTask.hpp"
#include "AppMain.hpp"
#include "FreeRTOS.h"
#include "SoftWatchdog.hpp"
#include "cJSON.h"
#include "main.h"
#include "task.h"
#include <rcl/error_handling.h>
#include <rmw_microros/rmw_microros.h>

// cJSON 使用 micro-ROS 分配器
extern "C" void *microros_allocate(size_t size, void *state);
extern "C" void microros_deallocate(void *ptr, void *state);

static void *cjson_malloc(size_t size) { return microros_allocate(size, NULL); }
static void cjson_free(void *ptr) { microros_deallocate(ptr, NULL); }

// ============================================================================
// 主循环
// ============================================================================

void MicroRosTask::run() {
  // 一次性配置 cJSON 分配器
  {
    cJSON_Hooks hooks;
    hooks.malloc_fn = cjson_malloc;
    hooks.free_fn = cjson_free;
    cJSON_InitHooks(&hooks);
  }

  for (;;) {
    // 喂软件看门狗
    auv::device::SoftWatchdog::getInstance().feed(
        auv::device::SoftWatchdog::Component::MICROROS);

    uint32_t now_ms = HAL_GetTick();

    switch (state_) {
    case State::WAITING_AGENT:
      // 每 100ms 探测一次 Agent
      if (transport_.pingAgent(200, 1)) {
        connectAgent();
      } else {
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      break;

    case State::AGENT_CONNECTED:
      // 每 2s 检查 Agent 在线状态
      if (now_ms - last_ping_ms_ >= 2000) {
        last_ping_ms_ = now_ms;
        if (!transport_.pingAgent(100, 1)) {
          disconnectAgent();
          break;
        }
      }

      // spin executor：处理所有订阅回调和服务请求
      rclc_executor_spin_some(&transport_.getExecutor(), RCL_MS_TO_NS(1));

      // 定时发布状态
      publisher_.publish(now_ms);
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ============================================================================
// 连接 / 断开 Agent
// ============================================================================

void MicroRosTask::connectAgent() {
  // 初始化 Transport（创建 support / node / executor）
  if (!transport_.init("zit6_node", "", 18)) {
    return; // 初始化失败，保持 WAITING_AGENT
  }

  // 按依赖顺序初始化各模块
  if (!subscriber_.init(&transport_.getNode(), &transport_.getExecutor())) {
    disconnectAgent();
    return;
  }
  if (!service_.init(&transport_.getNode(), &transport_.getExecutor())) {
    disconnectAgent();
    return;
  }
  if (!publisher_.init(&transport_.getNode())) {
    disconnectAgent();
    return;
  }

  state_ = State::AGENT_CONNECTED;
  last_ping_ms_ = HAL_GetTick();
}

void MicroRosTask::disconnectAgent() {
  // 按依赖逆序清理
  publisher_.cleanup(&transport_.getNode());
  subscriber_.cleanup(&transport_.getNode());
  service_.cleanup(&transport_.getNode());
  transport_.fini();

  state_ = State::WAITING_AGENT;
}

// ============================================================================
// FreeRTOS 任务入口
// ============================================================================

void UserApp_MicroRosTask(void *argument) {
  (void)argument;
  MicroRosTask runner;
  runner.run();
}
