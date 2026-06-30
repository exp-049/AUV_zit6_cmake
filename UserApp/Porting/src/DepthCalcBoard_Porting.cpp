#include "DepthCalcBoard_Porting.hpp"

// DMA 双缓冲（位于 RAM_D2，DMA 可访问）
__attribute__((section(".dma_buffer"))) static uint8_t s_dma_buf_a[256];
__attribute__((section(".dma_buffer"))) static uint8_t s_dma_buf_b[256];

namespace auv {
namespace porting {

DepthCalcBoard_Porting *DepthCalcBoard_Porting::active_instance = nullptr;

DepthCalcBoard_Porting::DepthCalcBoard_Porting(
    UART_HandleTypeDef *huart, auv::peripheral::UART_DepthBackend *backend)
    : huart_(huart), backend_(backend), dma_buf_a_(s_dma_buf_a),
      dma_buf_b_(s_dma_buf_b) {
  active_instance = this;
}

bool DepthCalcBoard_Porting::transmitPort(void *ctx, const uint8_t *data,
                                          uint16_t len) {
  auto *self = static_cast<DepthCalcBoard_Porting *>(ctx);
  return HAL_UART_Transmit(self->huart_, (uint8_t *)data, len, 100) == HAL_OK;
}

void DepthCalcBoard_Porting::pollPort(void *ctx) {
  static_cast<DepthCalcBoard_Porting *>(ctx)->poll();
}

bool DepthCalcBoard_Porting::startRxPort(void *ctx) {
  return static_cast<DepthCalcBoard_Porting *>(ctx)->startRx();
}

bool DepthCalcBoard_Porting::startRx() {
  active_buf_ = dma_buf_a_;
  ready_buf_ = nullptr;
  buf_ready_ = false;
  return HAL_UART_Receive_DMA(huart_, dma_buf_a_, kBufSize) == HAL_OK;
}

void DepthCalcBoard_Porting::startDma(uint8_t *buf) {
  active_buf_ = buf;
  HAL_UART_Receive_DMA(huart_, buf, kBufSize);
}

void DepthCalcBoard_Porting::poll() {
  if (!buf_ready_ || !ready_buf_ || !backend_)
    return;

  // 将就绪缓冲中的字节逐个喂给 backend
  for (uint16_t i = 0; i < kBufSize; i++) {
    backend_->onRxByte(ready_buf_[i]);
  }

  // 释放缓冲
  ready_buf_ = nullptr;
  buf_ready_ = false;
}

void DepthCalcBoard_Porting::onDmaComplete() {
  // 当前 active_buf_ 填满了，标记为就绪
  ready_buf_ = active_buf_;

  // 切到另一个缓冲继续接收
  uint8_t *next_buf = (active_buf_ == dma_buf_a_) ? dma_buf_b_ : dma_buf_a_;
  startDma(next_buf);

  // 内存屏障保证 buf_ready_ 在 active_buf_ 切换后写入
  __DSB();
  buf_ready_ = true;
}

} // namespace porting
} // namespace auv

// ============================================================================
// DMA 传输完成中断
// 切缓冲 → 标记就绪 → 重启 DMA，不做解析
// ============================================================================
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  auto *port = auv::porting::DepthCalcBoard_Porting::active_instance;
  if (port && huart == port->getUart()) {
    port->onDmaComplete();
  }
}
