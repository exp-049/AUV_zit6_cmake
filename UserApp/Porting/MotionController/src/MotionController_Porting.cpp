#include "MotionController_Porting.hpp"
#include "MotionController_Driver.hpp"
#include "core_cm7.h"
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

__attribute__((section(".dma_buffer"), used)) uint8_t
    auv::porting::MotionController_Porting::tx_packet_buf_[39];

namespace auv {
namespace porting {

MotionController_Porting::MotionController_Porting(UART_HandleTypeDef *huart)
    : huart_(huart) {}

bool MotionController_Porting::transmitDMA(void *ctx, const uint8_t *data,
                                           uint16_t size) {
  auto *self = static_cast<MotionController_Porting *>(ctx);
  if (!self || !self->huart_)
    return false;

  // A thrust frame is sent every 10 ms.  One-shot LED/servo/configuration
  // frames must wait for that DMA transfer instead of being dropped when the
  // UART is momentarily busy.
  constexpr int kWaitRetries = 6;
  for (int retry = 0; retry < kWaitRetries; ++retry) {
    bool dma_busy = false;
    if (self->huart_->hdmatx) {
      auto *stream = (DMA_Stream_TypeDef *)self->huart_->hdmatx->Instance;
      dma_busy = (stream->CR & DMA_SxCR_EN) != 0U;
    }
    if (self->huart_->gState == HAL_UART_STATE_READY && !dma_busy)
      break;
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  if (self->huart_->gState != HAL_UART_STATE_READY)
    return false;
  if (self->huart_->hdmatx) {
    auto *stream = (DMA_Stream_TypeDef *)self->huart_->hdmatx->Instance;
    if ((stream->CR & DMA_SxCR_EN) != 0U)
      return false;
  }
  uintptr_t dcache_line = 32u;
  uintptr_t addr = (uintptr_t)data;
  uintptr_t end = (addr + size + dcache_line - 1) & ~(dcache_line - 1);
  SCB_CleanDCache_by_Addr((uint32_t *)(addr & ~(dcache_line - 1)),
                          (int32_t)(end - (addr & ~(dcache_line - 1))));
  return HAL_UART_Transmit_DMA(self->huart_, (uint8_t *)data, size) == HAL_OK;
}

void *MotionController_Porting::getTxPacket(void *ctx) {
  (void)ctx;
  return tx_packet_buf_;
}

} // namespace porting
} // namespace auv
