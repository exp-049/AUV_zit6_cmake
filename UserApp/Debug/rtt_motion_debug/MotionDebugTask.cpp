#include "DebugApp.hpp"

#include "AppContext.hpp"
#include "MotionController_Driver.hpp"
#include "SEGGER_RTT.h"
#include "cmsis_os2.h"
#include "iwdg.h"
#include "main.h"
#include "usart.h"

#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace {

constexpr std::size_t kLineCapacity = 128U;
constexpr std::size_t kReadCapacity = 32U;
uint8_t g_motion_rx_byte = 0U;

struct MotionState {
  float x = 0.0F;
  float r = 0.0F;
  float d = 0.0F;
  float y = 0.0F;
};

void printFixed(float value, unsigned decimals = 3U) {
  if (!std::isfinite(value)) {
    SEGGER_RTT_WriteString(0, "nan");
    return;
  }

  const bool negative = value < 0.0F;
  const double magnitude = negative ? -static_cast<double>(value)
                                    : static_cast<double>(value);
  const unsigned long scale = decimals == 1U ? 10UL : 1000UL;
  const unsigned long scaled = static_cast<unsigned long>(
      magnitude * static_cast<double>(scale) + 0.5);
  if (decimals == 1U) {
    SEGGER_RTT_printf(0, "%s%lu.%01lu", negative ? "-" : "",
                      scaled / scale, scaled % scale);
  } else {
    SEGGER_RTT_printf(0, "%s%lu.%03lu", negative ? "-" : "",
                      scaled / scale, scaled % scale);
  }
}

void printState(const MotionState &state) {
  SEGGER_RTT_WriteString(0, " state={X=");
  printFixed(state.x);
  SEGGER_RTT_WriteString(0, ",R=");
  printFixed(state.r);
  SEGGER_RTT_WriteString(0, ",D=");
  printFixed(state.d);
  SEGGER_RTT_WriteString(0, ",Y=");
  printFixed(state.y);
  SEGGER_RTT_WriteString(0, "}\r\n");
}

void printHelp() {
  SEGGER_RTT_WriteString(
      0,
      "Commands (finish with Enter): H, X/R/D/Y[-1..1], S[-180..180], "
      "L[0..3], STOP, HELP\r\n");
  SEGGER_RTT_WriteString(
      0, "X=Fx forward/back, R=Fy right/left, D=Fz down/up, Y=Fyaw\r\n");
}

void printHandshakeResponse(auv::peripheral::MotionController_Driver &driver) {
  uint8_t status = 0U;
  while (driver.takeHandshakeResponse(status)) {
    SEGGER_RTT_printf(0, "RX HANDSHAKE: FA AF 04 %02X FB BF -> %s\r\n",
                      status, status == 0x01U ? "OK" : "ERR status");
  }
}

char *trim(char *text) {
  while (*text != '\0' && std::isspace(static_cast<unsigned char>(*text))) {
    ++text;
  }

  char *end = text + std::strlen(text);
  while (end > text && std::isspace(static_cast<unsigned char>(end[-1]))) {
    --end;
  }
  *end = '\0';
  return text;
}

bool parseFloat(const char *text, float &value) {
  char *end = nullptr;
  value = std::strtof(text, &end);
  if (end == text || !std::isfinite(value)) {
    return false;
  }
  while (*end != '\0' && std::isspace(static_cast<unsigned char>(*end))) {
    ++end;
  }
  return *end == '\0';
}

bool parseLight(const char *text, uint8_t &state) {
  char *end = nullptr;
  const long parsed = std::strtol(text, &end, 10);
  if (end == text) {
    return false;
  }
  while (*end != '\0' && std::isspace(static_cast<unsigned char>(*end))) {
    ++end;
  }
  if (*end != '\0' || parsed < 0L || parsed > 3L) {
    return false;
  }
  state = static_cast<uint8_t>(parsed);
  return true;
}

bool publish(auv::peripheral::MotionController_Driver &driver,
             const MotionState &state) {
  return driver.publishThrust(state.x, state.r, state.d, state.y, 0.0F, 0.0F);
}

void processCommand(char *raw_line, MotionState &state,
                    auv::peripheral::MotionController_Driver &driver) {
  char *line = trim(raw_line);
  if (*line == '\0') {
    return;
  }

  SEGGER_RTT_printf(0, "RX: %s -> ", line);

  if (std::strcmp(line, "HELP") == 0 || std::strcmp(line, "?") == 0) {
    SEGGER_RTT_WriteString(0, "OK\r\n");
    printHelp();
    return;
  }

  if (std::strcmp(line, "STOP") == 0) {
    state = {};
    const bool sent = publish(driver, state);
    SEGGER_RTT_printf(0, "%s STOP", sent ? "OK" : "ERR tx");
    printState(state);
    return;
  }

  if (std::strcmp(line, "H") == 0) {
    const bool sent = driver.sendHandshake();
    SEGGER_RTT_printf(0, "TX HANDSHAKE: FA AF 04 FB BF -> %s\r\n",
                      sent ? "DMA OK" : "ERR local TX");
    return;
  }

  const char command = static_cast<char>(
      std::toupper(static_cast<unsigned char>(line[0])));
  const char *argument = line + 1;

  if (command == 'X' || command == 'R' || command == 'D' || command == 'Y') {
    float value = 0.0F;
    if (!parseFloat(argument, value) || value < -1.0F || value > 1.0F) {
      SEGGER_RTT_WriteString(0, "ERR expected value in [-1,1]\r\n");
      return;
    }

    switch (command) {
    case 'X':
      state.x = value;
      break;
    case 'R':
      state.r = value;
      break;
    case 'D':
      state.d = value;
      break;
    case 'Y':
      state.y = value;
      break;
    default:
      break;
    }

    const bool sent = publish(driver, state);
    SEGGER_RTT_printf(0, "%s %c=", sent ? "OK" : "ERR tx", command);
    printFixed(value);
    printState(state);
    return;
  }

  if (command == 'S') {
    float angle = 0.0F;
    if (!parseFloat(argument, angle) || angle < -180.0F || angle > 180.0F) {
      SEGGER_RTT_WriteString(0,
                             "ERR expected servo angle in [-180,180] deg\r\n");
      return;
    }

    const bool sent = driver.setServoAngle(angle);
    SEGGER_RTT_printf(0, "%s S=", sent ? "OK" : "ERR tx");
    printFixed(angle, 1U);
    SEGGER_RTT_WriteString(0, " deg\r\n");
    return;
  }

  if (command == 'L') {
    uint8_t light_state = 0U;
    if (!parseLight(argument, light_state)) {
      SEGGER_RTT_WriteString(0, "ERR expected light state 0,1,2,3\r\n");
      return;
    }

    const bool sent = driver.setLightState(light_state);
    SEGGER_RTT_printf(0, "%s L=%u\r\n", sent ? "OK" : "ERR tx",
                      light_state);
    return;
  }

  SEGGER_RTT_WriteString(0, "ERR unknown command; type HELP\r\n");
}

} // namespace

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart != &huart6) {
    return;
  }

  auto *driver = auv::system::g_app_ctx.motor_driver;
  if (driver != nullptr) {
    driver->onRxByte(g_motion_rx_byte);
  }
  (void)HAL_UART_Receive_IT(&huart6, &g_motion_rx_byte, 1U);
}

extern "C" void UserApp_MotionDebugTask(void *argument) {
  (void)argument;

  SEGGER_RTT_Init();
  SEGGER_RTT_ConfigUpBuffer(0, "MOTION_FEEDBACK", nullptr, 0,
                            SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);

  static char down_buffer[128] = {};
  SEGGER_RTT_ConfigDownBuffer(0, "MOTION_COMMAND", down_buffer,
                              sizeof(down_buffer),
                              SEGGER_RTT_MODE_NO_BLOCK_SKIP);

  auto *driver = auv::system::g_app_ctx.motor_driver;
  if (driver == nullptr) {
    SEGGER_RTT_WriteString(0, "ERR motor driver is null\r\n");
    osThreadExit();
  }

  if (HAL_UART_Receive_IT(&huart6, &g_motion_rx_byte, 1U) != HAL_OK) {
    SEGGER_RTT_WriteString(0, "ERR UART6 RX start failed\r\n");
  }

  SEGGER_RTT_WriteString(0, "rtt_motion_debug ready\r\n");
  printHelp();

  MotionState state;
  char line[kLineCapacity] = {};
  char read_buffer[kReadCapacity] = {};
  std::size_t line_length = 0U;
  bool line_overflow = false;

  for (;;) {
    printHandshakeResponse(*driver);
    const unsigned received = SEGGER_RTT_Read(0, read_buffer, sizeof(read_buffer));
    for (unsigned i = 0U; i < received; ++i) {
      const char ch = read_buffer[i];
      if (ch == '\r' || ch == '\n') {
        if (line_overflow) {
          SEGGER_RTT_WriteString(0, "RX: <too long> -> ERR line too long\r\n");
        } else if (line_length > 0U) {
          line[line_length] = '\0';
          processCommand(line, state, *driver);
        }
        line_length = 0U;
        line_overflow = false;
      } else if (ch == '\b' || ch == 0x7FU) {
        if (line_length > 0U) {
          --line_length;
        }
      } else if (std::isprint(static_cast<unsigned char>(ch))) {
        if (line_length + 1U < sizeof(line)) {
          line[line_length++] = ch;
        } else {
          line_overflow = true;
        }
      }
    }

    printHandshakeResponse(*driver);

    HAL_IWDG_Refresh(&hiwdg1);
    osDelay(1U);
  }
}
