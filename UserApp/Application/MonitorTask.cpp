#include "MonitorTask.hpp"
#include "AppMain.hpp"
#include "FreeRTOS.h"
#include "RosLogger.hpp"
#include "SoftWatchdog.hpp"
#include "SystemConfig.hpp"
#include "SystemContext.hpp"
#include "iwdg.h"
#include "task.h"

void MonitorTask::run() {
  init();

  for (;;) {
    refreshHardwareWatchdogIfNeeded();
    safety_monitor_.check(HAL_GetTick());

    vTaskDelayUntil(&last_wake_time_, pdMS_TO_TICKS(kLoopPeriodMs));
  }
}

void MonitorTask::init() {
  ctx_->watchdog->init(auv::config::sys_config.soft_watchdog);
  last_wake_time_ = xTaskGetTickCount();

  ROS_LOG_INFO("System MonitorTask initialized (10Hz)");
}

void MonitorTask::refreshHardwareWatchdogIfNeeded() {
  if (ctx_->watchdog->check()) {
    HAL_IWDG_Refresh(&hiwdg1);
  }
}

void UserApp_MonitorTask(void *argument) {
  (void)argument;
  MonitorTask runner(&auv::system::g_app_ctx);
  runner.run();
}
