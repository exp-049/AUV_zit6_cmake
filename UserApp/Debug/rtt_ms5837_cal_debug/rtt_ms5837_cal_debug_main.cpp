/**
 * @file rtt_ms5837_cal_debug_main.c
 * @brief 深度计标定调试入口
 *
 * 当 RTT_MS5837_CAL_DEBUG 定义时，替代 ControlTask 进入标定循环。
 * 复用现有 HAL 初始化，使用全局 depth_sensor 实例。
 *
 * 使用：
 *   1. config.json: "rtt_ms5837_cal_debug": true
 *   2. 编译烧录
 *   3. J-Link RTT Viewer 查看
 */

#include "AppContext.hpp" // g_app_ctx 声明
#include "AppMain.hpp"
#include "SEGGER_RTT.h"
#include "SystemContext.hpp" // 深度驱动完整类型
#include "cmsis_os2.h"
#include "main.h"

// ============================================================================
// 深度计标定循环（替代正常飞控循环）
// ============================================================================
void rtt_ms5837_cal_debug_main(void) {
  SEGGER_RTT_WriteString(0, "=== MS5837 Calibration Debug ===\r\n");
  SEGGER_RTT_WriteString(0, "Depth     Temp     Status\r\n");
  SEGGER_RTT_WriteString(0, "-------   ------  ------\r\n");

  uint32_t last_print = 0;
  uint32_t ok_count = 0;

  auto *sensor = auv::system::g_app_ctx.depth_sensor;
  sensor->Init();
  sensor->start();

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

    osDelay(5);
  }
}
