/**
 * @file DebugTask.cpp
 * @brief RTT 调试任务 — RTT_DEBUG 模式下替代 ControlTask + MicroRosTask
 */

#include "DebugTask.hpp"
#include "AppContext.hpp"
#include "AppMain.hpp"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "SEGGER_RTT.h"
#include "SystemConfig.hpp"
#include "SystemContext.hpp"
#include "main.h"
#include "task.h"

void UserApp_DebugTask(void *argument) {
  (void)argument;

  SEGGER_RTT_WriteString(0, "=== RTT Debug Mode ===\r\n");

  auto *sensor = auv::system::g_app_ctx.depth_sensor;

#ifdef RTT_MS5837_CAL_DEBUG
  /* === 深度计标定模式 === */
  SEGGER_RTT_WriteString(0, "Mode: MS5837 Calibration\r\n");
  SEGGER_RTT_WriteString(0, "Depth     Temp     Status\r\n");
  SEGGER_RTT_WriteString(0, "-------   ------  ------\r\n");

  sensor->Init();
  sensor->start();

  uint32_t last_print = 0;
  uint32_t ok_count = 0;

  for (;;) {
    int r = sensor->Read();

    if (r == 1) {
      float d = 0;
      sensor->Depth(&d);
      ok_count++;

      uint32_t now = HAL_GetTick();
      if (now - last_print >= 100) {
        last_print = now;
        SEGGER_RTT_printf(0, "%+7.3f  %6.2f  OK(%lu)\r\n", d,
                          sensor->temperture, (unsigned long)ok_count);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
#else
  /* === 通用调试模式 === */
  SEGGER_RTT_WriteString(0, "Mode: Generic Debug\r\n");

  for (;;) {
    SEGGER_RTT_printf(0, "Tick: %lu\r\n", (unsigned long)HAL_GetTick());
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
#endif
}

extern "C" void UserApp_CreateDebugTask(void) {
#ifdef RTT_DEBUG
  const osThreadAttr_t debug_attr = {
      .name = "rtt_debug",
      .stack_size = 1024,
      .priority = osPriorityNormal,
  };
  osThreadNew(UserApp_DebugTask, NULL, &debug_attr);
#else
  (void)0;
#endif
}
