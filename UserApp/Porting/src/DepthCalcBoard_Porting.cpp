#include "DepthCalcBoard_Porting.hpp"
#include "RosLogger.hpp"

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
  ROS_LOG_DEBUG("[DepthPorting] constructed, huart=%p, backend=%p", huart_,
                backend_);
}

bool DepthCalcBoard_Porting::transmitPort(void *ctx, const uint8_t *data,
                                          uint16_t len) {
  auto *self = static_cast<DepthCalcBoard_Porting *>(ctx);
  ROS_LOG_DEBUG("[DepthPorting] transmit len=%u", len);
  HAL_StatusTypeDef ret =
      HAL_UART_Transmit(self->huart_, (uint8_t *)data, len, 100);
  ROS_LOG_DEBUG("[DepthPorting] transmit -> HAL_Status=%d", (int)ret);
  return ret == HAL_OK;
}

void DepthCalcBoard_Porting::pollPort(void *ctx) {
  static_cast<DepthCalcBoard_Porting *>(ctx)->poll();
}

bool DepthCalcBoard_Porting::startRxPort(void *ctx) {
  bool ok = static_cast<DepthCalcBoard_Porting *>(ctx)->startRx();
  ROS_LOG_DEBUG("[DepthPorting] startRxPort -> %d", ok);
  return ok;
}

bool DepthCalcBoard_Porting::startRx() {
  active_buf_ = dma_buf_a_;
  dma_pos_ = 0;
  HAL_StatusTypeDef ret = HAL_UART_Receive_DMA(huart_, dma_buf_a_, kBufSize);
  ROS_LOG_DEBUG("[DepthPorting] startRx() -> HAL_UART_Receive_DMA=%d, "
                "buf_a=%p, buf_b=%p, hdmarx=%p",
                (int)ret, dma_buf_a_, dma_buf_b_, huart_->hdmarx);
  return ret == HAL_OK;
}

void DepthCalcBoard_Porting::startDma(uint8_t *buf) {
  ROS_LOG_DEBUG("[DepthPorting] startDma -> buf=%p", buf);
  active_buf_ = buf;
  HAL_StatusTypeDef ret = HAL_UART_Receive_DMA(huart_, buf, kBufSize);
  if (ret != HAL_OK) {
    ROS_LOG_DEBUG("[DepthPorting] startDma FAILED! HAL_Status=%d", (int)ret);
  }
}

void DepthCalcBoard_Porting::poll() {
  if (!huart_ || !backend_)
    return;

  // 读取 DMA 剩余计数，计算已接收字节数
  uint16_t remaining = __HAL_DMA_GET_COUNTER(huart_->hdmarx);
  uint16_t received = kBufSize - remaining;

  // 处理新收到的字节（增量方式，不触发任何额外中断）
  while (dma_pos_ < received) {
    backend_->onRxByte(active_buf_[dma_pos_]);
    dma_pos_++;
  }

  // 缓冲区已满：DMA 已停止（NORMAL 模式），切到另一个缓冲重启
  if (remaining == 0 && received > 0) {
    // 打印缓冲前几个字节用于诊断
    ROS_LOG_DEBUG("[DepthPorting] BUF_FULL: active=%p dma_pos=%u, "
                  "first4=[%02x %02x %02x %02x]",
                  active_buf_, dma_pos_,
                  active_buf_[0], active_buf_[1], active_buf_[2],
                  active_buf_[3]);
    uint8_t *next_buf = (active_buf_ == dma_buf_a_) ? dma_buf_b_ : dma_buf_a_;
    dma_pos_ = 0;
    startDma(next_buf);
  }
}

} // namespace porting
} // namespace auv
