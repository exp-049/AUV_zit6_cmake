#include "DebugApp.hpp"

#include "AppContext.hpp"
#include "SEGGER_RTT.h"
#include "cmsis_os2.h"
#include "iwdg.h"
#include "main.h"

#include "MS5837_Driver.hpp"

extern "C" void UserApp_Ms5837CalDebugTask(void *argument) {
  (void)argument;
  auto *sensor = auv::system::g_app_ctx.depth_sensor;

  SEGGER_RTT_WriteString(0, "=== MS5837_CAL_DEBUG ===\r\n");
  SEGGER_RTT_WriteString(0, "Depth(m), Temp(C), Connected, Samples\r\n");
  sensor->Init();
  sensor->start();

  uint32_t last_print = 0;
  uint32_t samples = 0;
  for (;;) {
    if (sensor->Read() == 1) {
      ++samples;
      const uint32_t now = HAL_GetTick();
      if (now - last_print >= 100) {
        float depth = 0.0f;
        sensor->Depth(&depth);
        SEGGER_RTT_printf(0, "%lu, %.4f, %.2f, %u, %lu\r\n",
                          (unsigned long)now, depth, sensor->temperture,
                          sensor->is_connected ? 1U : 0U,
                          (unsigned long)samples);
        last_print = now;
      }
    }
    HAL_IWDG_Refresh(&hiwdg1);
    osDelay(5);
  }
}
