#pragma once

#include "stm32h7xx_hal.h"
#include <stdint.h>

namespace auv {
namespace peripheral {
struct InsPortDiagnostics;
}
} // namespace auv

// DMA 接收缓冲区（在 INS_Porting.cpp 中定义，位于 RAM_D2 段）
extern uint8_t ins_rx_buffer[512];

namespace auv {
namespace porting {

/**
 * @class INS_Porting
 * @brief INS 惯导硬件适配层（由 SerialHandles.hpp 选择 UART）
 *
 * 封装 UART DMA 接收环形缓冲和指令发送。
 * 替代原 SerialPort + 裸 HAL 调用的模式。
 */
class INS_Porting {
public:
  INS_Porting(UART_HandleTypeDef *rx_uart, UART_HandleTypeDef *tx_uart,
              uint8_t *ext_buf, uint16_t buf_size);

  /** @brief 供 InsPortOps 使用的静态包装 */
  static bool initPort(void *ctx);
  static uint16_t readPort(void *ctx, uint8_t *buf, uint16_t max_len);
  static bool transmitPort(void *ctx, const uint8_t *data, uint16_t len);
  static void diagnosticsPort(
      void *ctx, auv::peripheral::InsPortDiagnostics *out);

  bool init();
  uint16_t read(uint8_t *buf, uint16_t max_len);
  bool transmit(const uint8_t *data, uint16_t len);
  void diagnostics(auv::peripheral::InsPortDiagnostics *out) const;

  /** @brief 检查是否在超时内收到过新数据 */
  bool isDataFresh(uint32_t timeout_ms) const;

  /** @brief 获取最近一次接收的时间戳 */
  uint32_t getLastRxTick() const { return last_rx_tick_; }

  /** @brief DMA 接收完成回调（由中断调用） */
  void onRxCompleted();

private:
  UART_HandleTypeDef *rx_uart_;
  UART_HandleTypeDef *tx_uart_;
  uint8_t *rx_buf_;
  uint16_t rx_buf_size_;
  uint16_t rx_read_idx_ = 0;
  volatile uint32_t last_rx_tick_ = 0;
  volatile uint32_t read_events_ = 0;
  volatile uint32_t total_bytes_ = 0;
};

} // namespace porting
} // namespace auv
