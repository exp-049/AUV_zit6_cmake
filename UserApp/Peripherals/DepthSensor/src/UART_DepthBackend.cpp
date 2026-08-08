//深度计解算版方案,商家刷新率1Hz,已弃用
#include "UART_DepthBackend.hpp"
#include "RosLogger.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace auv {
namespace peripheral {

UART_DepthBackend::UART_DepthBackend(UartPortOps ops) : ops_(ops) {
  ROS_LOG_DEBUG("[DepthBackend] UART_DepthBackend constructed, ops=%p", &ops_);
}

bool UART_DepthBackend::init() {
  connected_ = true;
  ROS_LOG_DEBUG("[DepthBackend] init() OK, connected=%d", connected_);
  return true;
}

void UART_DepthBackend::poll() {
  // 委托 Porting 层将 DMA 缓冲中的新数据喂给 onRxByte()
  if (ops_.poll) {
    ops_.poll(ops_.ctx);
  }
}

/** @brief 解析 "T=XX.XXD=XX.XX" 格式字符串（深度解算板手册定义） */
static bool parseDepthLine(const char *line, float &depth, float &temp) {
  const char *t = strstr(line, "T=");
  const char *d = strstr(line, "D=");
  if (!t || !d) {
    ROS_LOG_DEBUG("[DepthBackend] parseDepthLine FAIL: no T=/D= marker");
    return false;
  }

  float tv = atof(t + 2);
  float dv = atof(d + 2);
  if (tv < -50.0f || tv > 100.0f) {
    ROS_LOG_DEBUG("[DepthBackend] parseDepthLine FAIL: T=%.2f out of range",
                  tv);
    return false;
  }
  if (dv < -100.0f || dv > 1000.0f) {
    ROS_LOG_DEBUG("[DepthBackend] parseDepthLine FAIL: D=%.2f out of range",
                  dv);
    return false;
  }

  depth = dv;
  temp = tv;
  ROS_LOG_DEBUG("[DepthBackend] parseDepthLine OK: D=%.2f T=%.2f", depth, temp);
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
      ROS_LOG_DEBUG("[DepthBackend] read() -> onDepthReady(Depth=%.2f, "
                    "Temp=%.2f)",
                    depth_, temperature_);
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

  if (byte == '\r') {
    line_buf_[line_len_] = '\0';
    // 以 hex 打印前 32 字节，看清实际收到的数据
    char hex[32 * 3 + 1] = {0};
    uint16_t hex_len = line_len_ > 32 ? 32 : line_len_;
    for (uint16_t i = 0; i < hex_len; i++)
      snprintf(hex + i * 3, sizeof(hex) - i * 3, "%02x ",
               (uint8_t)line_buf_[i]);
    ROS_LOG_DEBUG("[DepthBackend] LINE len=%u hex=[%s] ascii=\"%s\"",
                  line_len_, hex, line_buf_);
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

  if (byte == '\n')
    return;

  line_buf_[line_len_++] = (char)byte;
}

void UART_DepthBackend::start() {
  ROS_LOG_DEBUG("[DepthBackend] start() enter");
  // 启动 DMA 接收（通过 UartPortOps）
  if (ops_.startRx) {
    bool ok = ops_.startRx(ops_.ctx);
    ROS_LOG_DEBUG("[DepthBackend] start() -> startRx returned %d", ok);
  } else {
    ROS_LOG_DEBUG("[DepthBackend] start() -> startRx is NULL!");
  }

  // 解算板上电后会自动输出 T=xx.xxD=xx.xx\r，无需发送命令
  // 注：!!\r 会暂停输出，不要乱发
}

bool UART_DepthBackend::sendCmd(const char *cmd) {
  ROS_LOG_DEBUG("[DepthBackend] sendCmd: \"%s\"", cmd);
  if (ops_.transmit) {
    bool ok = ops_.transmit(ops_.ctx, (const uint8_t *)cmd, strlen(cmd));
    ROS_LOG_DEBUG("[DepthBackend] sendCmd -> transmit returned %d", ok);
    return ok;
  }
  ROS_LOG_DEBUG("[DepthBackend] sendCmd: transmit is NULL!");
  return false;
}

} // namespace peripheral
} // namespace auv
