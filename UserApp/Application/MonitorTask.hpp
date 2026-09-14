#ifndef __MONITOR_TASK_HPP
#define __MONITOR_TASK_HPP

#include "AppContext.hpp"
#include "FreeRTOS.h"
#include "SafetyMonitor.hpp"
#include "task.h"
#include <stdint.h>

/**
 * @class MonitorTask
 * @brief 独立安全监控任务 (10Hz)
 *
 * 职责（从 ControlTask 分离）：
 * - 软件看门狗检查 → 喂硬件看门狗
 * - 心跳超时检测 → 自动上锁
 * - 解锁条件检查 → 自动解锁
 */
class MonitorTask {
public:
  MonitorTask(auv::system::AppContext *ctx) : ctx_(ctx), safety_monitor_(ctx) {}

  void run();

private:
  auv::system::AppContext *ctx_;
  static constexpr uint32_t kLoopPeriodMs = 100; // 10Hz

  TickType_t last_wake_time_ = 0;
  auv::component::SafetyMonitor safety_monitor_;

  void init();
  void refreshHardwareWatchdogIfNeeded();
};

#endif // __MONITOR_TASK_HPP
