#include "IICTask.hpp"
#include "AppMain.hpp"
#include "FreeRTOS.h"
#include "SoftWatchdog.hpp"
#include "SystemConfig.hpp"
#include "SystemContext.hpp"
#include "task.h"
#include <cmath>

void IICTask::run() {
#ifdef USE_DEPTH_CALC_BOARD
  // ======== 解算板方案（UART）========
  // TODO: 解算板 Porting 就绪后实现
  // - 初始化解算板 UART (DepthCalcBoard_Porting)
  // - 循环读取 UART 行数据
  // - 调用 DepthSensor_ParseData() 解析
  // - 调用 ctx_->depth_sensor->setMS5837Z() 注入深度
  // - 喂狗

  last_wake_time_ = xTaskGetTickCount();
  for (;;) {
    // 占位：后续替换为 UART 读取逻辑
    vTaskDelayUntil(&last_wake_time_, pdMS_TO_TICKS(20));
  }
#else
  // ======== I2C 方案（MS5837 直连，当前方案）========
  ctx_->depth_sensor->Init();

  // Validation state
  float last_valid_depth = 0.0f;
  int bad_count = 0;
  const int kMaxBadCount = 5;

  last_wake_time_ = xTaskGetTickCount();

  for (;;) {
    if (ctx_->depth_sensor->is_connected) {
      int r = ctx_->depth_sensor->Read();
      if (r > 0) {
        float d = 0.0f;
        ctx_->depth_sensor->Depth(&d);

        bool valid = true;
        if (!std::isfinite(d))
          valid = false;

        // If reading is exactly zero while we previously had a sensible depth,
        // treat it as invalid (common symptom of I2C read returning zeros).
        if (d == 0.0f && last_valid_depth > 0.5f)
          valid = false;

        // Range sanity check (adjust bounds if your platform needs different
        // limits)
        if (d < -5.0f || d > 500.0f)
          valid = false;

        if (valid) {
          bad_count = 0;
          ctx_->depth_sensor->setMS5837Z(d);

          last_valid_depth = d;
          ctx_->watchdog->feed(auv::component::SoftWatchdog::Component::DEPTH);
        } else {
          bad_count++;
          if (bad_count >= kMaxBadCount) {
            // Try to recover sensor if repeated bad readings
            ctx_->depth_sensor->Init();
            bad_count = 0;
          }
        }
      } else if (r < 0) {
        // I2C error
        bad_count++;
        if (bad_count >= kMaxBadCount) {
          ctx_->depth_sensor->Init();
          bad_count = 0;
        }
      } else {
        // r == 0 -> conversion in progress, nothing to do this cycle
      }
    } else {
      // Not connected, try to init
      ctx_->depth_sensor->Init();
    }
    // Aim for ~60Hz sampling calls to the non-blocking Read()
    vTaskDelayUntil(&last_wake_time_, pdMS_TO_TICKS(
        8)); // ~125Hz loop; with 2-step conversion yields ~62.5Hz samples
  }
#endif
}

void UserApp_IICTask(void *argument) {
  (void)argument;
  IICTask runner(&auv::system::g_app_ctx);
  runner.run();
}
