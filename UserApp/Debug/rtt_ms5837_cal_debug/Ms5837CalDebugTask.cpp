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

  SEGGER_RTT_Init();
  SEGGER_RTT_ConfigUpBuffer(0, nullptr, nullptr, 0,
                            SEGGER_RTT_MODE_NO_BLOCK_SKIP);
  SEGGER_RTT_WriteString(0, "=== MS5837_CAL_DEBUG ===\r\n");
  SEGGER_RTT_WriteString(0, "UART4 115200 8N1 + DMA_CIRCULAR_IDLE\r\n");
  SEGGER_RTT_WriteString(
      0, "z = protocol DATA.depth_cm / 100.0 (meters)\r\n");
  sensor->Init();
  sensor->start();
  SEGGER_RTT_printf(0, "driver_connected_after_init=%u\r\n",
                    sensor->is_connected ? 1U : 0U);
  SEGGER_RTT_WriteString(0,
                         "host sends HANDSHAKE every 2s; device ACK is required\r\n");

  uint32_t last_report = HAL_GetTick();
  uint32_t last_sample = 0U;
  uint32_t polls = 0U;
  uint32_t samples = 0U;
  for (;;) {
    ++polls;
    if (sensor->Read() != 0) {
      ++samples;
      last_sample = HAL_GetTick();
    }

    const uint32_t now = HAL_GetTick();
    if (now - last_report >= 1000U) {
      float depth = 0.0f;
      sensor->Depth(&depth);
      const uint32_t sample_age =
          samples == 0U ? 0xFFFFFFFFUL : now - last_sample;

      // SEGGER_RTT_printf is a small formatter and does not implement %f.
      // Convert values to fixed-point before passing them as varargs; using
      // %f here would leave the following connected argument misaligned.
      const long z_milli = (long)(depth * 1000.0f + 0.5f);
      const long temp_centi =
          (long)(sensor->temperture * 100.0f +
                 (sensor->temperture >= 0.0f ? 0.5f : -0.5f));
      const long temp_abs = temp_centi < 0L ? -temp_centi : temp_centi;
      const char *temp_sign = temp_centi < 0L ? "-" : "";

      SEGGER_RTT_printf(
          0,
          "tick=%lu polls=%lu samples=%lu age_ms=%lu z_protocol=%ld.%03ld "
          "temp=%s%ld.%02ld connected=%u handshake_ack=%u rx_recoveries=%lu "
          "rx_errors=%lu last_error=%lu recovery_reason=%lu rx_events=%lu "
          "dma_pos=%lu\r\n",
          (unsigned long)now, (unsigned long)polls, (unsigned long)samples,
          (unsigned long)sample_age, z_milli / 1000L, z_milli % 1000L,
          temp_sign, temp_abs / 100L, temp_abs % 100L,
          sensor->is_connected ? 1U : 0U,
          sensor->isHandshakeAcknowledged() ? 1U : 0U,
          (unsigned long)sensor->getRxRecoveryCount(),
          (unsigned long)sensor->getRxErrorCount(),
          (unsigned long)sensor->getLastRxError(),
          (unsigned long)sensor->getLastRxRecoveryReason(),
          (unsigned long)sensor->getRxEventCount(),
          (unsigned long)sensor->getDmaWritePos());
      polls = 0U;
      last_report = now;
    }

    HAL_IWDG_Refresh(&hiwdg1);
    osDelay(5);
  }
}
