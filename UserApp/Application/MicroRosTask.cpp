#include "MicroRosTask.hpp"
#include "AppMain.hpp"
#include "FreeRTOS.h"
#include "RosLogger.hpp"
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

  last_wake_time_ = xTaskGetTickCount();
  for (;;) {
    // Keep NORMAL-mode watchdog behavior identical to the known-good 7/22
    // firmware. Agent availability is a communication state, not a reason to
    // reset the local control firmware while its tasks are alive.
    ctx_->watchdog->feed(auv::component::SoftWatchdog::Component::MICROROS);

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

      // 推杆协议 ACK/重发由 micro-ROS 任务周期驱动，避免在 ROS 回调中
      // 阻塞等待 UART 响应。
      // The executor may run a subscription callback immediately before this
      // call. Use a fresh tick: a GPIO pushrod task records its start time in
      // the callback, so reusing the tick captured before spin_some() can make
      // (now - start) underflow and complete a multi-second task instantly.
      subscriber_.update(HAL_GetTick());

      // 定时发布状态
      publisher_.publish(now_ms);
      break;
    }

    vTaskDelayUntil(&last_wake_time_, pdMS_TO_TICKS(kLoopPeriodMs));
  }
}

// ============================================================================
// 连接 / 断开 Agent
// ============================================================================

void MicroRosTask::connectAgent() {
  // 初始化 Transport（创建 support / node / executor）
  if (!transport_.init("zit6_node", "", 18)) {
    ROS_LOG_ERROR("micro-ROS transport init failed");
    return; // 初始化失败，保持 WAITING_AGENT
  }

  // 按依赖顺序初始化各模块
  if (!subscriber_.init(&transport_.getNode(), &transport_.getExecutor())) {
    ROS_LOG_ERROR("micro-ROS subscriber core init failed");
    disconnectAgent();
    return;
  }
  if (!service_.init(&transport_.getNode(), &transport_.getExecutor())) {
    ROS_LOG_ERROR("micro-ROS service init failed");
    disconnectAgent();
    return;
  }
  if (!publisher_.init(&transport_.getNode())) {
    ROS_LOG_ERROR("micro-ROS publisher init failed");
    disconnectAgent();
    return;
  }

  state_ = State::AGENT_CONNECTED;
  last_ping_ms_ = HAL_GetTick();
  ROS_LOG_INFO("micro-ROS Agent connected; state publishers ready");
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
  MicroRosTask runner(&auv::system::g_app_ctx);
  runner.run();
}
