#include "MonitorTask.hpp"
#include "AppMain.hpp"
#include "FreeRTOS.h"
#include "RosLogger.hpp"
#include "SoftWatchdog.hpp"
#include "SystemConfig.hpp"
#include "SystemContext.hpp"
#include "cmsis_os2.h"
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
  ctx_->watchdog->init(auv::config::sys_config.system.soft_watchdog);
  last_wake_time_ = xTaskGetTickCount();

  ROS_LOG_INFO("System MonitorTask initialized (10Hz)");
}

void MonitorTask::refreshHardwareWatchdogIfNeeded() {
  if (ctx_->watchdog->check()) {
    HAL_IWDG_Refresh(&hiwdg1);
  }
}

void UserApp_MonitorTask(void *argument) {
#ifdef RTT_DEBUG
  /* RTT 调试模式：MonitorTask 不运行，DebugTask 在 freertos.c 中创建 */
  for (;;)
    vTaskSuspend(NULL);
#endif
  (void)argument;
  MonitorTask runner(&auv::system::g_app_ctx);
  runner.run();
}
