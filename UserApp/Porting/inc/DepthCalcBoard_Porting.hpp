#pragma once

#include "UART_DepthBackend.hpp" // UartPortOps
#include "stm32h7xx_hal.h"
#include <stdint.h>

namespace auv {
namespace porting {

/**
 * @class DepthCalcBoard_Porting
 * @brief 深度解算板 UART 硬件适配层（DMA 双缓冲）
 *
 * 封装 UART5 的 HAL 操作：
 * - UartPortOps::transmit → HAL_UART_Transmit
 * - DMA 双缓冲接收 → 中断只切缓冲 → 调用方解析
 *
 * 双缓冲：
 *   1. DMA 填充 buf_a → TC 中断 → 切到 buf_b，标记 buf_a 就绪
 *   2. 调用方调 poll() → 获取就绪缓冲 → 推字节到 Backend::onRxByte()
 */
class DepthCalcBoard_Porting {
public:
  DepthCalcBoard_Porting(UART_HandleTypeDef *huart,
                         auv::peripheral::UART_DepthBackend *backend = nullptr);
  void setBackend(auv::peripheral::UART_DepthBackend *backend) {
    backend_ = backend;
  }

  /** @brief 供 UartPortOps::transmit 使用的静态包装 */
  static bool transmitPort(void *ctx, const uint8_t *data, uint16_t len);

  /** @brief 供 UartPortOps::poll 使用的静态包装 */
  static void pollPort(void *ctx);

  /** @brief 供 UartPortOps::startRx 使用的静态包装 */
  static bool startRxPort(void *ctx);

  /** @brief 启动 DMA 接收（填充 buf_a） */
  bool startRx();

  /** @brief 获取 UART 句柄 */
  UART_HandleTypeDef *getUart() const { return huart_; }

  /**
   * @brief 轮询：若有就绪缓冲，将字节逐个喂给 backend->onRxByte()
   * 由 MS5837_Driver::Read() 或 IICTask 调用
   */
  void poll();

  /** @brief 由 HAL_UART_RxCpltCallback 调用，切缓冲 + 重启 DMA */
  void onDmaComplete();

  /** @brief 静态实例指针，供 DMA 中断回调路由 */
  static DepthCalcBoard_Porting *active_instance;

  /** @brief 单缓冲大小 */
  static constexpr uint16_t kBufSize = 256;

private:
  /** @brief 启动/重启 DMA 填充指定缓冲 */
  void startDma(uint8_t *buf);

  UART_HandleTypeDef *huart_;
  auv::peripheral::UART_DepthBackend *backend_;

  /** @brief DMA 双缓冲（位于 RAM_D2） */
  uint8_t *dma_buf_a_; // 实际缓冲在 .cpp 中分配（RAM_D2）
  uint8_t *dma_buf_b_;

  uint8_t *active_buf_ = nullptr;         // DMA 当前填充的缓冲
  uint8_t *volatile ready_buf_ = nullptr; // 已填满待处理的缓冲（中断中写入）
  volatile bool buf_ready_ = false;       // 中断中置位
};

} // namespace porting
} // namespace auv
