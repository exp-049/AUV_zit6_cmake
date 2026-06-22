#include "MotionController_Porting.hpp"
#include "MotionController_Driver.hpp"
#include "core_cm7.h"
#include "stm32h7xx_hal.h"

__attribute__((section(".dma_buffer"), used)) uint8_t
    auv::porting::MotionController_Porting::tx_packet_buf_[26];

namespace auv {
namespace porting {

MotionController_Porting::MotionController_Porting(UART_HandleTypeDef *huart)
    : huart_(huart) {}

bool MotionController_Porting::transmitDMA(void *ctx, const uint8_t *data,
                                           uint16_t size) {
  auto *self = static_cast<MotionController_Porting *>(ctx);
  if (!self || !self->huart_)
    return false;
  if (self->huart_->gState != HAL_UART_STATE_READY)
    return false;
  if (self->huart_->hdmatx) {
    auto *stream = (DMA_Stream_TypeDef *)self->huart_->hdmatx->Instance;
    int retries = 3;
    while (retries-- > 0) {
      if ((stream->CR & DMA_SxCR_EN) == 0U)
        break;
      for (volatile int i = 0; i < 100; ++i)
        __asm__ volatile("nop");
    }
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
