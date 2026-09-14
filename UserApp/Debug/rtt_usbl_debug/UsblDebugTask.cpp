#include "DebugApp.hpp"

#include "AppContext.hpp"
#include "SEGGER_RTT.h"
#include "USBL_Driver.hpp"
#include "cmsis_os2.h"
#include "iwdg.h"
#include "main.h"

#include <cmath>
#include <cstdint>
#include <cstring>

namespace {

constexpr uint16_t kFrameSize = auv::peripheral::USBL_Driver::kFrameSize;

void printFieldPrefix(const uint8_t *frame, uint16_t offset, uint16_t size,
                      const char *meaning) {
  SEGGER_RTT_printf(0, "offset=%03u, source=", offset);
  for (uint16_t i = 0; i < size; ++i) {
    SEGGER_RTT_printf(0, "%02X%s", frame[offset + i],
                      (i + 1 == size) ? "" : " ");
  }
  SEGGER_RTT_printf(0, ", meaning=%s, parsed=", meaning);
}

void printRawField(const uint8_t *frame, uint16_t offset, uint16_t size,
                   const char *meaning) {
  printFieldPrefix(frame, offset, size, meaning);
  SEGGER_RTT_WriteString(0, "raw\r\n");
}

void printFixedFloat(float value, uint32_t scale, uint8_t decimals) {
  if (!std::isfinite(value)) {
    SEGGER_RTT_printf(0, "%s\r\n", std::isnan(value) ? "nan" : "inf");
    return;
  }

  const bool negative = value < 0.0f;
  const char *sign = negative ? "-" : "";
  const double magnitude = negative ? -static_cast<double>(value)
                                    : static_cast<double>(value);
  const double scaled_value = magnitude * static_cast<double>(scale) + 0.5;
  if (scaled_value > 4294967295.0) {
    SEGGER_RTT_printf(0, "%soverflow\r\n", sign);
    return;
  }

  const uint32_t scaled = static_cast<uint32_t>(scaled_value);
  const uint32_t whole = scaled / scale;
  const uint32_t fraction = scaled % scale;
  if (decimals == 2) {
    SEGGER_RTT_printf(0, "%s%lu.%02lu\r\n", sign,
                      static_cast<unsigned long>(whole),
                      static_cast<unsigned long>(fraction));
  } else {
    SEGGER_RTT_printf(0, "%s%lu.%07lu\r\n", sign,
                      static_cast<unsigned long>(whole),
                      static_cast<unsigned long>(fraction));
  }
}

void printFloatField(const uint8_t *frame, uint16_t offset, const char *meaning,
                     float value) {
  printFieldPrefix(frame, offset, 4, meaning);
  printFixedFloat(value, 10000000U, 7);
}

void printScaledI16Field(const uint8_t *frame, uint16_t offset,
                         const char *meaning, int16_t raw) {
  printFieldPrefix(frame, offset, 2, meaning);
  const bool negative = raw < 0;
  const uint32_t magnitude =
      negative ? static_cast<uint32_t>(-static_cast<int32_t>(raw))
               : static_cast<uint32_t>(raw);
  SEGGER_RTT_printf(0, "%s%lu.%02lu\r\n", negative ? "-" : "",
                    static_cast<unsigned long>(magnitude / 100U),
                    static_cast<unsigned long>(magnitude % 100U));
}

void printU16Field(const uint8_t *frame, uint16_t offset, const char *meaning,
                   uint16_t value) {
  printFieldPrefix(frame, offset, 2, meaning);
  SEGGER_RTT_printf(0, "%u (0x%04X)\r\n", value, value);
}

void printU8Field(const uint8_t *frame, uint16_t offset, const char *meaning,
                  uint8_t value) {
  printFieldPrefix(frame, offset, 1, meaning);
  SEGGER_RTT_printf(0, "%u (0x%02X)\r\n", value, value);
}

void printFrame(const auv::peripheral::USBL_Driver &driver,
                const auv::peripheral::UsblState &state,
                uint32_t frame_number) {
  uint8_t frame[kFrameSize] = {};
  if (driver.copyLastFrame(frame, sizeof(frame)) != kFrameSize) {
    SEGGER_RTT_WriteString(0, "USBL frame copy failed\r\n");
    return;
  }

  SEGGER_RTT_printf(0, "\r\nUSBL frame #%lu (%u bytes)\r\n",
                    static_cast<unsigned long>(frame_number), kFrameSize);
  SEGGER_RTT_WriteString(0, "偏置,源码,含义,解析\r\n");

  printRawField(frame, 0, 2, "frame_header");
  printFloatField(frame, 2, "attitude_roll", state.roll);
  printFloatField(frame, 6, "attitude_pitch", state.pitch);
  printFloatField(frame, 10, "attitude_yaw", state.yaw);
  printFloatField(frame, 14, "pressure", state.pressure);
  for (uint8_t i = 0; i < 4; ++i) {
    printFloatField(frame, static_cast<uint16_t>(18 + i * 4), "slant_range",
                    state.slant_range[i]);
  }
  printRawField(frame, 34, 4, "reserved_34_37");
  printFloatField(frame, 38, "latitude", state.latitude);
  printFloatField(frame, 42, "longitude", state.longitude);
  for (uint8_t i = 0; i < 3; ++i) {
    printFloatField(frame, static_cast<uint16_t>(46 + i * 4), "time_diff",
                    state.time_diff[i]);
  }
  printRawField(frame, 58, 1, "reserved_58");
  for (uint8_t i = 0; i < 3; ++i) {
    printScaledI16Field(frame, static_cast<uint16_t>(59 + i * 2),
                        "passive_attitude", state.passive_attitude[i]);
  }
  for (uint8_t i = 0; i < 4; ++i) {
    printU16Field(frame, static_cast<uint16_t>(65 + i * 2), "signal_strength",
                  state.signal_strength[i]);
  }
  printRawField(frame, 73, 10, "reserved_73_82");
  printRawField(frame, 83, 8, "energy_0_7");
  printFloatField(frame, 91, "signal", state.signal);
  printFloatField(frame, 95, "gain", state.gain);
  printFloatField(frame, 99, "north", state.beacon_north);
  printFloatField(frame, 103, "east", state.beacon_east);
  printFloatField(frame, 107, "depth", state.beacon_depth);
  printRawField(frame, 111, 4, "reserved_111_114");
  printU8Field(frame, 115, "status", state.sensor_status);
  printU8Field(frame, 116, "year", state.year);
  printU8Field(frame, 117, "month", state.month);
  printU8Field(frame, 118, "day", state.day);
  printU8Field(frame, 119, "hour", state.hour);
  printU8Field(frame, 120, "minute", state.minute);
  printFloatField(frame, 121, "second", state.second);
  printRawField(frame, 125, 4, "reserved_125_128");
  printU8Field(frame, 129, "navigation_status", state.nav_mode);
  printRawField(frame, 130, 1, "xor_checksum");
  printRawField(frame, 131, 2, "frame_tail");
}

} // namespace

extern "C" void UserApp_UsblDebugTask(void *argument) {
  (void)argument;

  auto *driver = auv::system::g_app_ctx.usbl_driver;
  if (driver == nullptr) {
    SEGGER_RTT_WriteString(0, "USBL driver unavailable\r\n");
    osThreadExit();
  }

  // The parsed table is larger than the default RTT up-buffer.  Blocking mode
  // keeps the debug output complete; this setting is used only in USBL_DEBUG.
  SEGGER_RTT_ConfigUpBuffer(0, nullptr, nullptr, 0,
                            SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
  SEGGER_RTT_WriteString(0, "=== USBL_DEBUG USART3 DMA-CIRCULAR+IDLE ===\r\n");
  driver->init();

  auv::peripheral::UsblState state{};
  uint32_t last_diag = HAL_GetTick();
  for (;;) {
    if (driver->update(state)) {
      auv::peripheral::UsblPortDiagnostics diagnostics{};
      driver->getDiagnostics(diagnostics);
      printFrame(*driver, state, diagnostics.valid_frames);
    }

    HAL_IWDG_Refresh(&hiwdg1);
    const uint32_t now = HAL_GetTick();
    if (now - last_diag >= 1000) {
      auv::peripheral::UsblPortDiagnostics diagnostics{};
      driver->getDiagnostics(diagnostics);
      SEGGER_RTT_printf(
          0,
          "USBL diag: events=%lu valid=%lu bad=%lu invalid=%lu "
          "write_pos=%u ndtr=%u dma=%u isr=0x%08lX rx=%02X %02X %02X %02X\r\n",
          static_cast<unsigned long>(diagnostics.events),
          static_cast<unsigned long>(diagnostics.valid_frames),
          static_cast<unsigned long>(diagnostics.invalid_frames),
          static_cast<unsigned long>(diagnostics.invalid_events),
          diagnostics.write_pos, diagnostics.dma_remaining,
          diagnostics.dma_enabled ? 1U : 0U,
          static_cast<unsigned long>(diagnostics.uart_isr),
          diagnostics.rx_preview[0], diagnostics.rx_preview[1],
          diagnostics.rx_preview[2], diagnostics.rx_preview[3]);
      last_diag = now;
    }
    osDelay(1);
  }
}
