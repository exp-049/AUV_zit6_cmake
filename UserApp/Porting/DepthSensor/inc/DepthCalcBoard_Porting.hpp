#pragma once

#include "UART_DepthBackend.hpp" // UartPortOps, DepthUartRxSink
#include "stm32h7xx_hal.h"
#include <stdint.h>

namespace auv {
namespace porting {

/**
 * @class DepthCalcBoard_Porting
 * @brief Text-protocol depth UART adapter using a polled circular DMA buffer.
 * The selected preset supplies the UART handle (currently UART4).
 */
class DepthCalcBoard_Porting {
public:
  DepthCalcBoard_Porting(UART_HandleTypeDef *huart,
                         auv::peripheral::DepthUartRxSink *backend = nullptr);
  void setBackend(auv::peripheral::DepthUartRxSink *backend) {
    backend_ = backend;
  }

  /** @brief 供 UartPortOps::transmit 使用的静态包装 */
  static bool transmitPort(void *ctx, const uint8_t *data, uint16_t len);

  /** @brief 供 UartPortOps::poll 使用的静态包装 */
  static void pollPort(void *ctx);

  /** @brief 供 UartPortOps::startRx 使用的静态包装 */
  static bool startRxPort(void *ctx);
  static void getDiagnosticsPort(void *ctx,
                                 auv::peripheral::DepthDiagnostics &out);
  static bool serviceRxRecoveryPort(void *ctx,
                                   bool no_valid_frame_timeout);
  static void handleHalError(UART_HandleTypeDef *huart);
  void getDiagnostics(auv::peripheral::DepthDiagnostics &out) const;

  /** @brief 启动 DMA 接收（填充 buf_a） */
  bool startRx();
  bool serviceRxRecovery(bool no_valid_frame_timeout);

  /** @brief 获取 UART 句柄 */
  UART_HandleTypeDef *getUart() const { return huart_; }

  /**
   * @brief 轮询：读取 DMA 计数器检查新数据，增量喂给 backend->onRxByte()
   * 由 Depth_Sensor_Driver::Read() → backend->poll() 调用（50Hz）
   *
   * 不依赖 DMA 完成中断或 UART 空闲中断，纯轮询方式，
   * 避免中断风暴导致系统无法喂狗。
   */
  void poll();

  /** @brief 静态实例指针 */
  static DepthCalcBoard_Porting *active_instance;

  /** @brief 单缓冲大小 */
  static constexpr uint16_t kBufSize = 256;

private:
  bool startReceive();
  bool isReceiveActive() const;

  UART_HandleTypeDef *huart_;
  auv::peripheral::DepthUartRxSink *backend_;

  /** @brief DMA 双缓冲（位于 RAM_D2） */
  uint8_t *dma_buf_a_;
  uint8_t *dma_buf_b_;

  uint8_t *active_buf_ = nullptr;  // DMA 当前填充的缓冲
  uint16_t dma_pos_ = 0;           // 当前缓冲中已处理的字节位置（poll 中更新）
  int32_t rx_start_status_ = -1;
  volatile uint32_t uart_error_count_ = 0U;
  volatile uint32_t last_uart_error_ = 0U;
  volatile int32_t rx_abort_status_ = -1;
  volatile int32_t rx_dma_deinit_status_ = -1;
  volatile int32_t rx_dma_init_status_ = -1;
  volatile int32_t rx_hal_start_status_ = -1;
  volatile uint32_t rx_recovery_attempts_ = 0U;
  volatile uint32_t rx_recovery_successes_ = 0U;
  volatile uint32_t rx_recovery_failures_ = 0U;
  volatile uint32_t last_rx_recovery_reason_ = 0U;
  volatile bool rx_recovery_pending_ = false;
  uint32_t consecutive_recovery_failures_ = 0U;
  uint32_t last_recovery_attempt_ms_ = 0U;
  bool has_recovery_attempted_ = false;
};

} // namespace porting
} // namespace auv
