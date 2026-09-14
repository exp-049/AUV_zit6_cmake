#include "INS_Porting.hpp"
#include "INS_Driver.hpp"

#include <cstring>

// DMA 接收缓冲区（必须位于 RAM_D2，DMA 才能访问）
__attribute__((section(".dma_buffer"))) uint8_t ins_rx_buffer[512];

namespace auv {
namespace porting {

INS_Porting::INS_Porting(UART_HandleTypeDef *rx_uart,
                         UART_HandleTypeDef *tx_uart, uint8_t *ext_buf,
                         uint16_t buf_size)
    : rx_uart_(rx_uart), tx_uart_(tx_uart), rx_buf_(ext_buf),
      rx_buf_size_(buf_size) {}

bool INS_Porting::initPort(void *ctx) {
  return static_cast<INS_Porting *>(ctx)->init();
}
uint16_t INS_Porting::readPort(void *ctx, uint8_t *buf, uint16_t max_len) {
  return static_cast<INS_Porting *>(ctx)->read(buf, max_len);
}
bool INS_Porting::transmitPort(void *ctx, const uint8_t *data, uint16_t len) {
  return static_cast<INS_Porting *>(ctx)->transmit(data, len);
}
void INS_Porting::diagnosticsPort(
    void *ctx, auv::peripheral::InsPortDiagnostics *out) {
  static_cast<INS_Porting *>(ctx)->diagnostics(out);
}

bool INS_Porting::init() {
  if (rx_uart_ == nullptr || rx_uart_->hdmarx == nullptr || rx_buf_ == nullptr ||
      rx_buf_size_ == 0) {
    return false;
  }

  for (int i = 0; i < 5; i++) {
    rx_read_idx_ = 0;
    read_events_ = 0;
    total_bytes_ = 0;
    last_rx_tick_ = 0;
    tx_calls_ = 0;
    tx_attempts_ = 0;
    tx_successes_ = 0;
    tx_failures_ = 0;
    tx_last_size_ = 0;
    tx_last_status_ = 0;
    std::memset(rx_buf_, 0, rx_buf_size_);
    if (HAL_UART_Receive_DMA(rx_uart_, rx_buf_, rx_buf_size_) == HAL_OK) {
      // INS uses a circular DMA ring and polls the producer position.  Half
      // and full transfer callbacks are unnecessary and would only add
      // interrupt load.
      __HAL_DMA_DISABLE_IT(rx_uart_->hdmarx, DMA_IT_HT | DMA_IT_TC);
      return true;
    }
    HAL_Delay(10);
  }
  return false;
}

uint16_t INS_Porting::read(uint8_t *buf, uint16_t max_len) {
  if (buf == nullptr || max_len == 0 || rx_uart_ == nullptr ||
      rx_uart_->hdmarx == nullptr || rx_buf_ == nullptr || rx_buf_size_ == 0) {
    return 0;
  }

  const uint16_t remaining = __HAL_DMA_GET_COUNTER(rx_uart_->hdmarx);
  if (remaining > rx_buf_size_) {
    return 0;
  }
  const uint16_t write_idx =
      static_cast<uint16_t>(rx_buf_size_ - remaining);
  const uint16_t avail = (write_idx >= rx_read_idx_)
                             ? static_cast<uint16_t>(write_idx - rx_read_idx_)
                             : static_cast<uint16_t>(rx_buf_size_ -
                                                     rx_read_idx_ + write_idx);
  if (avail == 0)
    return 0;

  uint16_t to_read = (avail < max_len) ? avail : max_len;
  for (uint16_t i = 0; i < to_read; i++) {
    buf[i] = rx_buf_[(rx_read_idx_ + i) % rx_buf_size_];
  }
  rx_read_idx_ = (rx_read_idx_ + to_read) % rx_buf_size_;
  ++read_events_;
  total_bytes_ += to_read;
  last_rx_tick_ = HAL_GetTick();
  return to_read;
}

bool INS_Porting::transmit(const uint8_t *data, uint16_t len) {
  ++tx_calls_;
  tx_last_size_ = len;

  if (tx_uart_ == nullptr || data == nullptr || len == 0U) {
    tx_last_status_ = static_cast<uint8_t>(HAL_ERROR);
    ++tx_failures_;
    return false;
  }

  for (int retry = 0; retry < 3; retry++) {
    ++tx_attempts_;
    const HAL_StatusTypeDef status =
        HAL_UART_Transmit(tx_uart_, const_cast<uint8_t *>(data), len, 50);
    tx_last_status_ = static_cast<uint8_t>(status);
    if (status == HAL_OK) {
      ++tx_successes_;
      return true;
    }
    ++tx_failures_;
  }
  return false;
}

bool INS_Porting::isDataFresh(uint32_t timeout_ms) const {
  return (HAL_GetTick() - last_rx_tick_ < timeout_ms);
}

void INS_Porting::onRxCompleted() {
  // 保留此接口供需要中断驱动的板级代码使用；当前 INS 采用 DMA 计数器
  // 轮询，避免依赖全局字节回调。
  last_rx_tick_ = HAL_GetTick();
}

void INS_Porting::diagnostics(
    auv::peripheral::InsPortDiagnostics *out) const {
  if (out == nullptr) {
    return;
  }
  out->read_events = read_events_;
  out->total_bytes = total_bytes_;
  out->tx_calls = tx_calls_;
  out->tx_attempts = tx_attempts_;
  out->tx_successes = tx_successes_;
  out->tx_failures = tx_failures_;
  out->tx_last_size = tx_last_size_;
  out->tx_last_status = tx_last_status_;
  out->tx_uart_ready = tx_uart_ != nullptr &&
                       tx_uart_->gState == HAL_UART_STATE_READY;
  if (rx_uart_ == nullptr || rx_uart_->hdmarx == nullptr || rx_buf_ == nullptr ||
      rx_buf_size_ == 0) {
    return;
  }

  const uint16_t remaining = __HAL_DMA_GET_COUNTER(rx_uart_->hdmarx);
  out->dma_remaining = remaining;
  if (remaining <= rx_buf_size_) {
    out->write_pos = static_cast<uint16_t>(rx_buf_size_ - remaining);
  }
  const auto *stream = reinterpret_cast<const DMA_Stream_TypeDef *>(
      rx_uart_->hdmarx->Instance);
  out->dma_enabled = (stream->CR & DMA_SxCR_EN) != 0U;
  out->uart_isr = rx_uart_->Instance->ISR;
  std::memcpy(out->rx_preview, rx_buf_, sizeof(out->rx_preview));
}

} // namespace porting
} // namespace auv
