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
bool INS_Porting::serviceRxRecoveryPort(void *ctx,
                                        bool no_valid_frame_timeout) {
  return static_cast<INS_Porting *>(ctx)->serviceRxRecovery(
      no_valid_frame_timeout);
}

bool INS_Porting::isReceiveActive() const {
  if (rx_uart_ == nullptr || rx_uart_->Instance == nullptr ||
      rx_uart_->hdmarx == nullptr || rx_uart_->hdmarx->Instance == nullptr) {
    return false;
  }
  const auto *stream = reinterpret_cast<const DMA_Stream_TypeDef *>(
      rx_uart_->hdmarx->Instance);
  return rx_uart_->RxState == HAL_UART_STATE_BUSY_RX &&
         (rx_uart_->Instance->CR3 & USART_CR3_DMAR) != 0U &&
         (stream->CR & DMA_SxCR_EN) != 0U;
}

bool INS_Porting::startReceive() {
  if (rx_uart_ == nullptr || rx_uart_->Instance == nullptr ||
      rx_uart_->hdmarx == nullptr || rx_uart_->hdmarx->Instance == nullptr ||
      rx_buf_ == nullptr || rx_buf_size_ == 0U) {
    return false;
  }

  // Reconcile HAL/DMA state after an error abort before starting a new ring.
  const HAL_StatusTypeDef abort_status = HAL_UART_AbortReceive(rx_uart_);
  rx_abort_status_ = static_cast<int32_t>(abort_status);
  if (abort_status != HAL_OK) {
    rx_start_status_ = rx_abort_status_;
    return false;
  }

  // HAL_UART_AbortReceive only aborts DMA when UART DMAR is still set. A
  // blocking UART error may already have cleared DMAR while leaving the DMA
  // handle BUSY/ERROR, causing the next HAL_UART_Receive_DMA to fail. Reset
  // and reinitialize the stream unconditionally before restarting the ring.
  const HAL_StatusTypeDef dma_deinit_status = HAL_DMA_DeInit(rx_uart_->hdmarx);
  rx_dma_deinit_status_ = static_cast<int32_t>(dma_deinit_status);
  if (dma_deinit_status != HAL_OK) {
    rx_start_status_ = rx_dma_deinit_status_;
    return false;
  }
  const HAL_StatusTypeDef dma_init_status = HAL_DMA_Init(rx_uart_->hdmarx);
  rx_dma_init_status_ = static_cast<int32_t>(dma_init_status);
  if (dma_init_status != HAL_OK) {
    rx_start_status_ = rx_dma_init_status_;
    return false;
  }

  rx_read_idx_ = 0U;
  std::memset(rx_buf_, 0, rx_buf_size_);
  const HAL_StatusTypeDef start_status =
      HAL_UART_Receive_DMA(rx_uart_, rx_buf_, rx_buf_size_);
  rx_hal_start_status_ = static_cast<int32_t>(start_status);
  rx_start_status_ = static_cast<int32_t>(start_status);
  if (start_status != HAL_OK || !isReceiveActive()) {
    if (start_status == HAL_OK) {
      rx_start_status_ = static_cast<int32_t>(HAL_ERROR);
      (void)HAL_UART_AbortReceive(rx_uart_);
    }
    return false;
  }

  // The ring is polled using NDTR; half/full interrupts are not needed.
  __HAL_DMA_DISABLE_IT(rx_uart_->hdmarx, DMA_IT_HT | DMA_IT_TC);
  return true;
}

bool INS_Porting::init() {
  if (rx_uart_ == nullptr || rx_uart_->hdmarx == nullptr || rx_buf_ == nullptr ||
      rx_buf_size_ == 0) {
    return false;
  }

  uart_error_count_ = 0U;
  last_uart_error_ = 0U;
  rx_recovery_attempts_ = 0U;
  rx_recovery_successes_ = 0U;
  rx_recovery_failures_ = 0U;
  rx_recovery_pending_ = false;
  has_rx_bytes_ = false;
  consecutive_recovery_failures_ = 0U;
  has_recovery_attempted_ = false;
  for (int i = 0; i < 5; i++) {
    read_events_ = 0;
    total_bytes_ = 0;
    last_rx_tick_ = 0;
    tx_calls_ = 0;
    tx_attempts_ = 0;
    tx_successes_ = 0;
    tx_failures_ = 0;
    tx_last_size_ = 0;
    tx_last_status_ = 0;
    if (startReceive()) {
      return true;
    }
    HAL_Delay(10);
  }
  return false;
}

void INS_Porting::onHalError(UART_HandleTypeDef *uart) {
  if (uart == nullptr || uart != rx_uart_) {
    return;
  }
  ++uart_error_count_;
  last_uart_error_ = uart->ErrorCode;
  rx_recovery_pending_ = true;
}

bool INS_Porting::serviceRxRecovery(bool no_valid_frame_timeout) {
  if (rx_uart_ == nullptr || rx_uart_->Instance == nullptr ||
      rx_uart_->hdmarx == nullptr || rx_uart_->hdmarx->Instance == nullptr ||
      rx_buf_ == nullptr || rx_buf_size_ == 0U) {
    return false;
  }

  const bool error_pending = rx_recovery_pending_;
  const uint32_t now_ms = HAL_GetTick();
  static constexpr uint32_t kNoRxProgressTimeoutMs = 1000U;
  const bool no_rx_progress =
      !has_rx_bytes_ || now_ms - last_rx_tick_ >= kNoRxProgressTimeoutMs;
  if (!error_pending && (!no_valid_frame_timeout || !no_rx_progress)) {
    // A parser timeout alone is not evidence that DMA stopped. Keep a healthy
    // stream running; rearm only after UART error or prolonged byte silence.
    return false;
  }

  static constexpr uint32_t kRetryDelaysMs[] = {
      100U, 250U, 500U, 1000U, 2000U};
  uint32_t retry_delay_ms = 0U;
  if (has_recovery_attempted_) {
    if (consecutive_recovery_failures_ == 0U) {
      retry_delay_ms = no_valid_frame_timeout ? 5000U : kRetryDelaysMs[0];
    } else if (consecutive_recovery_failures_ <=
               sizeof(kRetryDelaysMs) / sizeof(kRetryDelaysMs[0])) {
      retry_delay_ms = kRetryDelaysMs[consecutive_recovery_failures_ - 1U];
    } else {
      retry_delay_ms = 5000U;
    }
  }
  if (has_recovery_attempted_ &&
      now_ms - last_recovery_attempt_ms_ < retry_delay_ms) {
    return false;
  }

  // Clear before calling HAL so a new IRQ during recovery remains pending.
  rx_recovery_pending_ = false;
  has_recovery_attempted_ = true;
  last_recovery_attempt_ms_ = now_ms;
  ++rx_recovery_attempts_;
  if (startReceive()) {
    consecutive_recovery_failures_ = 0U;
    ++rx_recovery_successes_;
    return true;
  }

  ++consecutive_recovery_failures_;
  ++rx_recovery_failures_;
  rx_recovery_pending_ = true;
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
  // NDTR can briefly be zero before the circular counter reloads. Both zero
  // and rx_buf_size_ denote ring position zero, never a separate position at
  // rx_buf_size_ that could make consumed data appear available again.
  const uint16_t write_idx = static_cast<uint16_t>(
      (rx_buf_size_ - remaining) % rx_buf_size_);
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
  has_rx_bytes_ = true;
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
  has_rx_bytes_ = true;
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
  out->uart_error_count = uart_error_count_;
  out->last_uart_error = last_uart_error_;
  out->rx_start_status = rx_start_status_;
  out->rx_abort_status = rx_abort_status_;
  out->rx_dma_deinit_status = rx_dma_deinit_status_;
  out->rx_dma_init_status = rx_dma_init_status_;
  out->rx_hal_start_status = rx_hal_start_status_;
  out->rx_recovery_attempts = rx_recovery_attempts_;
  out->rx_recovery_successes = rx_recovery_successes_;
  out->rx_recovery_failures = rx_recovery_failures_;
  out->rx_recovery_pending = rx_recovery_pending_;
  out->tx_uart_ready = tx_uart_ != nullptr &&
                       tx_uart_->gState == HAL_UART_STATE_READY;
  if (rx_uart_ == nullptr || rx_uart_->Instance == nullptr ||
      rx_uart_->hdmarx == nullptr || rx_uart_->hdmarx->Instance == nullptr ||
      rx_buf_ == nullptr || rx_buf_size_ == 0) {
    return;
  }

  const uint16_t remaining = __HAL_DMA_GET_COUNTER(rx_uart_->hdmarx);
  out->dma_remaining = remaining;
  if (remaining <= rx_buf_size_) {
    out->write_pos = static_cast<uint16_t>(
        (rx_buf_size_ - remaining) % rx_buf_size_);
  }
  out->dma_enabled = isReceiveActive();
  out->dma_hal_state = static_cast<uint32_t>(rx_uart_->hdmarx->State);
  out->dma_hal_error = rx_uart_->hdmarx->ErrorCode;
  out->uart_hal_rx_state = static_cast<uint32_t>(rx_uart_->RxState);
  out->uart_isr = rx_uart_->Instance->ISR;
  const uint16_t preview_size =
      static_cast<uint16_t>(sizeof(out->rx_preview));
  for (uint16_t i = 0U; i < preview_size; ++i) {
    const uint16_t distance = static_cast<uint16_t>(preview_size - i);
    const uint16_t index = static_cast<uint16_t>(
        (out->write_pos + rx_buf_size_ - distance) % rx_buf_size_);
    out->rx_preview[i] = rx_buf_[index];
  }
}

} // namespace porting
} // namespace auv
