//深度计解算版方案,商家刷新率1Hz,已弃用
#pragma once

#include "MS5837_Driver.hpp" // DepthBackend
#include <stdint.h>

namespace auv {
namespace peripheral {

/**
 * @brief UART 硬件操作接口（函数指针表，由 Porting 层实现）
 */
struct UartPortOps {
  void *ctx;
  /** 发送数据到解算板 */
  bool (*transmit)(void *ctx, const uint8_t *data, uint16_t len);
  /** 轮询 DMA 双缓冲，将有新数据的字节喂给 onRxByte */
  void (*poll)(void *ctx);
  /** 启动 DMA 接收 */
  bool (*startRx)(void *ctx);
};

/**
 * @class UART_DepthBackend
 * @brief 解算板 UART 深度传感器后端
 *
 * 纯协议层，不涉及任何 HAL/硬件调用。
 * 硬件操作通过 UartPortOps 函数指针表注入。
 *
 * 数据格式："Depth=0.89m Temp=37.88"
 * 由 Porting 层的 UART 中断驱动 → onRxByte() → 解析 → DataReadyCallback
 */
class UART_DepthBackend : public DepthBackend {
public:
  UART_DepthBackend(UartPortOps ops);

  /** @name Backend 接口方法 */
  /**@{*/
  bool init();
  void poll();
  bool read();
  void setCallback(DepthDataReadyCallback cb) { cb_ = cb; }
  void start();
  bool isConnected() const { return connected_; }
  float getDepth() const { return depth_; }
  float getTemperature() const { return temperature_; }
  /**@}*/

  /**
   * @brief 由 Porting 层的 UART 中断回调调用
   * @param byte 接收到的字节
   *
   * 积累行缓冲，遇到 \n 时解析 + 触发 DataReadyCallback
   */
  void onRxByte(uint8_t byte);

  /** @brief 发送命令到解算板（通过 UartPortOps） */
  bool sendCmd(const char *cmd);

private:
  UartPortOps ops_;

  static constexpr uint16_t kLineBufSize = 128;
  char line_buf_[kLineBufSize];
  uint16_t line_len_ = 0;
  bool line_ready_ = false;

  DepthDataReadyCallback cb_ = {nullptr, nullptr};

  float depth_ = 0.0f;
  float temperature_ = 0.0f;
  bool connected_ = false;
};

} // namespace peripheral
} // namespace auv
