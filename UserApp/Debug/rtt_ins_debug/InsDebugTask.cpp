#include "DebugApp.hpp"

#include "AppContext.hpp"
#include "INS_Driver.hpp"
#include "SEGGER_RTT.h"
#include "SystemContext.hpp"
#include "cmsis_os2.h"
#include "iwdg.h"
#include "main.h"

#include <cmath>
#include <cstdint>
#include <cstring>

namespace {

constexpr uint16_t kFrameSize = auv::peripheral::INS_Driver::kFrameSize;

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

void printFixedFloat(float value, uint32_t scale) {
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
  if (scale == 100U) {
    SEGGER_RTT_printf(0, "%s%lu.%02lu\r\n", sign,
                      static_cast<unsigned long>(whole),
                      static_cast<unsigned long>(fraction));
  } else {
    SEGGER_RTT_printf(0, "%s%lu.%07lu\r\n", sign,
                      static_cast<unsigned long>(whole),
                      static_cast<unsigned long>(fraction));
  }
}

float readLEFloat(const uint8_t *data) {
  uint32_t bits = static_cast<uint32_t>(data[0]) |
                  (static_cast<uint32_t>(data[1]) << 8) |
                  (static_cast<uint32_t>(data[2]) << 16) |
                  (static_cast<uint32_t>(data[3]) << 24);
  float value = 0.0f;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

int32_t readLE32(const uint8_t *data) {
  const uint32_t value = static_cast<uint32_t>(data[0]) |
                         (static_cast<uint32_t>(data[1]) << 8) |
                         (static_cast<uint32_t>(data[2]) << 16) |
                         (static_cast<uint32_t>(data[3]) << 24);
  return static_cast<int32_t>(value);
}

void printFloatField(const uint8_t *frame, uint16_t offset, const char *meaning,
                     uint32_t scale = 10000000U) {
  printFieldPrefix(frame, offset, 4, meaning);
  printFixedFloat(readLEFloat(frame + offset), scale);
}

void printScaledI32Field(const uint8_t *frame, uint16_t offset,
                         const char *meaning, uint32_t scale = 1000000U) {
  printFieldPrefix(frame, offset, 4, meaning);
  const int64_t raw = readLE32(frame + offset);
  const bool negative = raw < 0;
  const uint64_t magnitude = static_cast<uint64_t>(negative ? -raw : raw);
  const uint64_t whole = magnitude / scale;
  const uint64_t fraction = magnitude % scale;
  if (scale == 1000000U) {
    SEGGER_RTT_printf(0, "%s%lu.%06lu\r\n", negative ? "-" : "",
                      static_cast<unsigned long>(whole),
                      static_cast<unsigned long>(fraction));
  } else {
    SEGGER_RTT_printf(0, "%s%lu.%07lu\r\n", negative ? "-" : "",
                      static_cast<unsigned long>(whole),
                      static_cast<unsigned long>(fraction));
  }
}

void printU8Field(const uint8_t *frame, uint16_t offset, const char *meaning,
                  uint8_t value) {
  printFieldPrefix(frame, offset, 1, meaning);
  SEGGER_RTT_printf(0, "%u (0x%02X)\r\n", value, value);
}

void printFrame(const auv::peripheral::INS_Driver &driver,
                uint32_t frame_number) {
  uint8_t frame[kFrameSize] = {};
  if (driver.copyLastFrame(frame, sizeof(frame)) != kFrameSize) {
    SEGGER_RTT_WriteString(0, "INS frame copy failed\r\n");
    return;
  }

  SEGGER_RTT_printf(0, "\r\nINS frame #%lu (%u bytes)\r\n",
                    static_cast<unsigned long>(frame_number), kFrameSize);
  SEGGER_RTT_WriteString(0, "偏置,源码,含义,解析\r\n");

  printRawField(frame, 0, 2, "frame_header");
  printFloatField(frame, 2, "roll_deg");
  printFloatField(frame, 6, "pitch_deg");
  printFloatField(frame, 10, "yaw_deg");
  printFloatField(frame, 14, "north_velocity");
  printFloatField(frame, 18, "east_velocity");
  printFloatField(frame, 22, "ground_velocity");
  printFloatField(frame, 26, "body_velocity_x");
  printFloatField(frame, 30, "body_velocity_y");
  printFloatField(frame, 34, "body_velocity_z");
  printScaledI32Field(frame, 38, "latitude");
  printScaledI32Field(frame, 42, "longitude");
  printFloatField(frame, 46, "combined_depth");
  printRawField(frame, 50, 21, "reserved_50_70");
  printFloatField(frame, 71, "angular_velocity_x");
  printFloatField(frame, 75, "angular_velocity_y");
  printFloatField(frame, 79, "angular_velocity_z");
  printScaledI32Field(frame, 83, "gps_longitude");
  printScaledI32Field(frame, 87, "gps_latitude");
  printFloatField(frame, 91, "gps_altitude");
  printFloatField(frame, 95, "gps_course");
  printFloatField(frame, 99, "gps_north_velocity");
  printFloatField(frame, 103, "gps_east_velocity");
  printFloatField(frame, 107, "depth_sensor");
  printFloatField(frame, 111, "altimeter_height");
  printU8Field(frame, 115, "sensor_status", frame[115]);
  printU8Field(frame, 116, "year", frame[116]);
  printU8Field(frame, 117, "month", frame[117]);
  printU8Field(frame, 118, "day", frame[118]);
  printU8Field(frame, 119, "hour", frame[119]);
  printU8Field(frame, 120, "minute", frame[120]);
  printFloatField(frame, 121, "second");
  printRawField(frame, 125, 4, "reserved_125_128");
  printU8Field(frame, 129, "navigation_mode", frame[129]);
  printU8Field(frame, 130, "xor_checksum", frame[130]);
  printRawField(frame, 131, 2, "frame_tail");

  SEGGER_RTT_WriteString(0, "INS frame parsed by INS_Driver\r\n");
}

} // namespace

extern "C" void UserApp_InsDebugTask(void *argument) {
  (void)argument;

  auto *driver = auv::system::g_app_ctx.ins_driver;
  if (driver == nullptr) {
    SEGGER_RTT_WriteString(0, "INS driver unavailable\r\n");
    osThreadExit();
  }

  SEGGER_RTT_ConfigUpBuffer(0, nullptr, nullptr, 0,
                            SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
  SEGGER_RTT_WriteString(
      0, "=== INS_DEBUG NAV-300 USART1 133B DMA-CIRCULAR ===\r\n");
  driver->init();

  auv::motion::NavState state{};
  uint32_t last_diag = HAL_GetTick();
  for (;;) {
    if (driver->update(state)) {
      auv::peripheral::InsPortDiagnostics diagnostics{};
      driver->getDiagnostics(diagnostics);
      printFrame(*driver, diagnostics.valid_frames);
    }

    HAL_IWDG_Refresh(&hiwdg1);
    const uint32_t now = HAL_GetTick();
    if (now - last_diag >= 1000U) {
      auv::peripheral::InsPortDiagnostics diagnostics{};
      driver->getDiagnostics(diagnostics);
      SEGGER_RTT_printf(
          0,
          "INS diag: reads=%lu bytes=%lu valid=%lu bad=%lu "
          "write_pos=%u ndtr=%u dma=%u isr=0x%08lX rx=%02X %02X %02X %02X\r\n",
          static_cast<unsigned long>(diagnostics.read_events),
          static_cast<unsigned long>(diagnostics.total_bytes),
          static_cast<unsigned long>(diagnostics.valid_frames),
          static_cast<unsigned long>(diagnostics.invalid_frames),
          diagnostics.write_pos, diagnostics.dma_remaining,
          diagnostics.dma_enabled ? 1U : 0U,
          static_cast<unsigned long>(diagnostics.uart_isr),
          diagnostics.rx_preview[0], diagnostics.rx_preview[1],
          diagnostics.rx_preview[2], diagnostics.rx_preview[3]);
      SEGGER_RTT_printf(
          0,
          "INS tx: calls=%lu attempts=%lu ok=%lu fail=%lu size=%u "
          "status=%u ready=%u\r\n",
          static_cast<unsigned long>(diagnostics.tx_calls),
          static_cast<unsigned long>(diagnostics.tx_attempts),
          static_cast<unsigned long>(diagnostics.tx_successes),
          static_cast<unsigned long>(diagnostics.tx_failures),
          diagnostics.tx_last_size, diagnostics.tx_last_status,
          diagnostics.tx_uart_ready ? 1U : 0U);
      last_diag = now;
    }
    osDelay(1);
  }
}
