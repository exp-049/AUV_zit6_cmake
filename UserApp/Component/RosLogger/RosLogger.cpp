#include "RosLogger.hpp"
#include "FreeRTOS.h"
#include "SEGGER_RTT.h"
#include "queue.h"
#include <cstdio>
#include <cstring>
#include <stdarg.h>

namespace auv {
namespace component {

void RosLogger::init() {
  SEGGER_RTT_Init(); // 初始化 RTT（J-Link 连接后即可查看）
  if (log_queue_ == nullptr) {
    log_queue_ = xQueueCreate(3, sizeof(LogEntry));
  }
}

void RosLogger::log(uint8_t level, const char *format, ...) {
  if (log_queue_ == nullptr) {
    return;
  }

  LogEntry entry;
  entry.level = level;

  va_list args;
  va_start(args, format);
  vsnprintf(entry.msg, sizeof(entry.msg), format, args);
  va_end(args);

  // 同时输出到 RTT（J-Link RTT Viewer 可见）
  SEGGER_RTT_printf(0, "[%s] %s\n",
                    level >= 40 ? "ERR" :
                    level >= 30 ? "WRN" :
                    level >= 20 ? "INF" : "DBG",
                    entry.msg);

  xQueueSend(static_cast<QueueHandle_t>(log_queue_), &entry, 0);
}

bool RosLogger::popLog(LogEntry &entry) {
  if (log_queue_ == nullptr) {
    return false;
  }
  return xQueueReceive(static_cast<QueueHandle_t>(log_queue_), &entry, 0) ==
         pdTRUE;
}

} // namespace component
} // namespace auv
