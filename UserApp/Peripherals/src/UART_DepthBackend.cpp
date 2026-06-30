#include "UART_DepthBackend.hpp"
#include <cstdlib>
#include <cstring>

namespace auv {
namespace peripheral {

UART_DepthBackend::UART_DepthBackend(UartPortOps ops) : ops_(ops) {}

bool UART_DepthBackend::init() {
  connected_ = true;
  return true;
}

void UART_DepthBackend::poll() {
  // 委托 Porting 层将 DMA 缓冲中的新数据喂给 onRxByte()
  if (ops_.poll) {
    ops_.poll(ops_.ctx);
  }
}

/** @brief 解析 "Depth:xxx Temp:xxx" 格式字符串 */
static bool parseDepthLine(const char *line, float &depth, float &temp) {
  const char *d = strstr(line, "Depth:");
  const char *t = strstr(line, "Temp:");
  if (!d || !t)
    return false;

  float dv = atof(d + 6);
  float tv = atof(t + 5);
  if (tv < -50.0f || tv > 100.0f)
    return false;
  if (dv < -100.0f || dv > 1000.0f)
    return false;

  depth = dv;
  temp = tv;
  return true;
}

bool UART_DepthBackend::read() {
  if (!line_ready_)
    return false;

  float d = 0, t = 0;
  if (parseDepthLine(line_buf_, d, t)) {
    depth_ = d;
    temperature_ = t;
    line_ready_ = false;

    if (cb_.onDepthReady) {
      cb_.onDepthReady(cb_.ctx, depth_, temperature_);
    }
    return true;
  }

  line_ready_ = false;
  return false;
}

void UART_DepthBackend::onRxByte(uint8_t byte) {
  if (line_len_ >= kLineBufSize - 1) {
    line_len_ = 0;
  }

  if (byte == '\n') {
    line_buf_[line_len_] = '\0';
    line_ready_ = true;
    line_len_ = 0;

    float d = 0, t = 0;
    if (parseDepthLine(line_buf_, d, t)) {
      depth_ = d;
      temperature_ = t;
      if (cb_.onDepthReady) {
        cb_.onDepthReady(cb_.ctx, depth_, temperature_);
      }
    }
    return;
  }

  if (byte == '\r')
    return;

  line_buf_[line_len_++] = (char)byte;
}

void UART_DepthBackend::start() {
  // 启动 DMA 接收（通过 UartPortOps）
  if (ops_.startRx) {
    ops_.startRx(ops_.ctx);
  }
}

bool UART_DepthBackend::sendCmd(const char *cmd) {
  if (ops_.transmit) {
    return ops_.transmit(ops_.ctx, (const uint8_t *)cmd, strlen(cmd));
  }
  return false;
}

} // namespace peripheral
} // namespace auv
