#include "DebugApp.hpp"

#include "AppContext.hpp"
#include "Depth_Sensor_Driver.hpp"
#include "Hardware_Preset.h"
#include "Pushrod_Driver.hpp"
#include "SEGGER_RTT.h"
#include "cmsis_os2.h"
#include "iwdg.h"
#include "main.h"

#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace {

constexpr std::size_t kLineCapacity = 128U;
constexpr std::size_t kReadCapacity = 32U;
constexpr uint32_t kMaxDurationMs = 60000U;

const char *resultName(uint8_t result) {
  using namespace auv::peripheral::pushrod;
  switch (result) {
  case kOk:
    return "OK";
  case kInvalidFrame:
    return "INVALID_FRAME";
  case kCrcError:
    return "CRC_ERROR";
  case kPowerOutOfRange:
    return "POWER_OUT_OF_RANGE";
  case kDurationInvalid:
    return "DURATION_INVALID";
  case kIdOutOfOrder:
    return "ID_OUT_OF_ORDER";
  case kIdConflict:
    return "ID_CONFLICT";
  case kQueueFull:
    return "QUEUE_FULL";
  case kNotInitialized:
    return "NOT_INITIALIZED";
  default:
    return "UNKNOWN";
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

bool parsePowerDuration(const char *text, int16_t &power,
                        uint32_t &duration_ms) {
  char *end = nullptr;
  const long parsed_power = std::strtol(text, &end, 10);
  if (end == text || parsed_power < -1000L || parsed_power > 1000L) {
    return false;
  }

  while (*end != '\0' && std::isspace(static_cast<unsigned char>(*end))) {
    ++end;
  }
  if (*end == ',' || *end == ':') {
    ++end;
  }
  while (*end != '\0' && std::isspace(static_cast<unsigned char>(*end))) {
    ++end;
  }

  char *duration_end = nullptr;
  const unsigned long parsed_duration = std::strtoul(end, &duration_end, 10);
  if (duration_end == end || parsed_duration == 0UL ||
      parsed_duration > kMaxDurationMs) {
    return false;
  }
  while (*duration_end != '\0' &&
         std::isspace(static_cast<unsigned char>(*duration_end))) {
    ++duration_end;
  }
  if (*duration_end != '\0') {
    return false;
  }

  power = static_cast<int16_t>(parsed_power);
  duration_ms = static_cast<uint32_t>(parsed_duration);
  return true;
}

void advanceTaskId(uint32_t &next_task_id) {
  ++next_task_id;
}

void printHelp() {
  SEGGER_RTT_WriteString(
      0,
      "Commands (finish with Enter): P<power> <duration_ms>, STOP, "
      "STATUS, HELP\r\n");
  SEGGER_RTT_WriteString(
      0,
      "power range=-1000..1000; duration range=1..60000 ms; "
      "P500 1000 / P-500,1000\r\n");
}

void printAcks(auv::peripheral::Pushrod_Driver &driver) {
  auv::peripheral::PushrodAck ack{};
  while (driver.readAck(&ack)) {
    SEGGER_RTT_printf(0, "ACK task_id=%lu result=0x%02X(%s) queue=%u ready=%u\r\n",
                      static_cast<unsigned long>(ack.task_id), ack.result,
                      resultName(ack.result), ack.queue_count, ack.ready);
  }
}

void printStatus(auv::peripheral::Pushrod_Driver &driver) {
  SEGGER_RTT_printf(0, "STATUS supported=%u\r\n",
                    driver.isSupported() ? 1U : 0U);
}

void stopPushrod(auv::peripheral::Pushrod_Driver &driver,
                 uint32_t &next_task_id) {
  driver.stop();

#if !AUV_PRESET_USES_GPIO_PUSHROD
  // The UART backend has no local stop pin. A zero-power short task is the
  // transport-level safe-stop command for the self-calculation board.
  const auv::peripheral::PushrodTask task{next_task_id, 0, 1U};
  const bool sent = driver.sendTask(task);
  if (sent) {
    advanceTaskId(next_task_id);
  }
  SEGGER_RTT_printf(0, "STOP %s\r\n", sent ? "OK" : "ERR tx");
#else
  (void)next_task_id;
  SEGGER_RTT_WriteString(0, "STOP OK\r\n");
#endif
}

void processCommand(char *raw_line, auv::peripheral::Pushrod_Driver &driver,
                    uint32_t &next_task_id) {
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

  if (std::strcmp(line, "STATUS") == 0) {
    SEGGER_RTT_WriteString(0, "OK\r\n");
    printStatus(driver);
    return;
  }

  if (std::strcmp(line, "STOP") == 0) {
    stopPushrod(driver, next_task_id);
    return;
  }

  if (std::toupper(static_cast<unsigned char>(line[0])) == 'P') {
    int16_t power = 0;
    uint32_t duration_ms = 0U;
    if (!parsePowerDuration(line + 1, power, duration_ms)) {
      SEGGER_RTT_WriteString(
          0, "ERR expected P<power> <duration_ms>, power [-1000,1000], "
             "duration [1,60000]\r\n");
      return;
    }

    const auv::peripheral::PushrodTask task{next_task_id, power, duration_ms};
    const bool sent = driver.sendTask(task);
    if (sent) {
      advanceTaskId(next_task_id);
    }
    SEGGER_RTT_printf(0, "%s task_id=%lu power=%d duration_ms=%lu\r\n",
                      sent ? "OK" : "ERR tx",
                      static_cast<unsigned long>(task.task_id), power,
                      static_cast<unsigned long>(duration_ms));
    return;
  }

  SEGGER_RTT_WriteString(0, "ERR unknown command; type HELP\r\n");
}

bool initPushrodTransport(auv::peripheral::Pushrod_Driver &driver,
                          auv::peripheral::Depth_Sensor_Driver *depth) {
#if AUV_PRESET_USES_GPIO_PUSHROD
  (void)depth;
  if (!driver.Init()) {
    return false;
  }
  driver.start();
  return true;
#else
  // The default self-UART preset shares UART4 with the depth driver. Its
  // facade owns the transport lifecycle, so initialize it exactly once here.
  if (depth == nullptr) {
    return false;
  }
  depth->Init();
  depth->start();
  return driver.isSupported();
#endif
}

} // namespace

extern "C" void UserApp_PushrodDebugTask(void *argument) {
  (void)argument;

  SEGGER_RTT_Init();
  SEGGER_RTT_ConfigUpBuffer(0, "PUSHROD_FEEDBACK", nullptr, 0,
                            SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);

  static char down_buffer[128] = {};
  SEGGER_RTT_ConfigDownBuffer(0, "PUSHROD_COMMAND", down_buffer,
                              sizeof(down_buffer),
                              SEGGER_RTT_MODE_NO_BLOCK_SKIP);

  auto *driver = auv::system::g_app_ctx.pushrod_driver;
  auto *depth = auv::system::g_app_ctx.depth_sensor;
  if (driver == nullptr || !driver->isSupported()) {
    SEGGER_RTT_WriteString(0, "ERR pushrod driver is unavailable\r\n");
    osThreadExit();
    return;
  }

  if (!initPushrodTransport(*driver, depth)) {
    SEGGER_RTT_WriteString(0, "ERR pushrod transport init failed\r\n");
    osThreadExit();
    return;
  }

  SEGGER_RTT_WriteString(0, "rtt_pushrod_debug ready\r\n");
  printHelp();

  char line[kLineCapacity] = {};
  char read_buffer[kReadCapacity] = {};
  std::size_t line_length = 0U;
  bool line_overflow = false;
  uint32_t next_task_id = 0U;

  for (;;) {
#if AUV_PRESET_USES_GPIO_PUSHROD
    driver->poll(HAL_GetTick());
#else
    // Self-UART ACKs and DMA recovery are consumed through the shared depth
    // facade's backend poll path.
    (void)depth->Read();
#endif
    printAcks(*driver);

    const unsigned received = SEGGER_RTT_Read(0, read_buffer, sizeof(read_buffer));
    for (unsigned i = 0U; i < received; ++i) {
      const char ch = read_buffer[i];
      if (ch == '\r' || ch == '\n') {
        if (line_overflow) {
          SEGGER_RTT_WriteString(0, "RX: <too long> -> ERR line too long\r\n");
        } else if (line_length > 0U) {
          line[line_length] = '\0';
          processCommand(line, *driver, next_task_id);
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

    printAcks(*driver);
    HAL_IWDG_Refresh(&hiwdg1);
    osDelay(1U);
  }
}
