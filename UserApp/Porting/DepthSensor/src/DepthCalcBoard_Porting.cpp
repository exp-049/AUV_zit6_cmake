#include "DepthCalcBoard_Porting.hpp"
#include "RosLogger.hpp"

#include <cstring>

// DMA 双缓冲（位于 RAM_D2，DMA 可访问）
__attribute__((section(".dma_buffer"))) static uint8_t s_dma_buf_a[256];
__attribute__((section(".dma_buffer"))) static uint8_t s_dma_buf_b[256];

namespace auv {
namespace porting {

DepthCalcBoard_Porting *DepthCalcBoard_Porting::active_instance = nullptr;

DepthCalcBoard_Porting::DepthCalcBoard_Porting(
    UART_HandleTypeDef *huart, auv::peripheral::DepthUartRxSink *backend)
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

bool DepthCalcBoard_Porting::serviceRxRecoveryPort(
    void *ctx, bool no_valid_frame_timeout) {
  if (ctx == nullptr) {
    return false;
  }
  return static_cast<DepthCalcBoard_Porting *>(ctx)->serviceRxRecovery(
      no_valid_frame_timeout);
}

bool DepthCalcBoard_Porting::isReceiveActive() const {
  if (huart_ == nullptr || huart_->Instance == nullptr ||
      huart_->hdmarx == nullptr || huart_->hdmarx->Instance == nullptr) {
    return false;
  }
  const auto *stream =
      static_cast<const DMA_Stream_TypeDef *>(huart_->hdmarx->Instance);
  return huart_->RxState == HAL_UART_STATE_BUSY_RX &&
         (huart_->Instance->CR3 & USART_CR3_DMAR) != 0U &&
         (stream->CR & DMA_SxCR_EN) != 0U;
}

bool DepthCalcBoard_Porting::startReceive() {
  active_buf_ = nullptr;
  dma_pos_ = 0;
  if (huart_ == nullptr || huart_->Instance == nullptr ||
      huart_->hdmarx == nullptr || huart_->hdmarx->Instance == nullptr) {
    rx_start_status_ = HAL_ERROR;
    return false;
  }

  // UART errors may leave the HAL handle and DMA stream in a half-aborted
  // state. Abort and clear the receive path before reusing the circular ring.
  const HAL_StatusTypeDef abort_status = HAL_UART_AbortReceive(huart_);
  rx_abort_status_ = static_cast<int32_t>(abort_status);
  if (abort_status != HAL_OK) {
    rx_start_status_ = rx_abort_status_;
    return false;
  }

  // UART error handling can clear DMAR before HAL_UART_AbortReceive runs. In
  // that case HAL may leave the DMA handle BUSY/ERROR even though RxState was
  // restored to READY. Reset the stream and DMAMUX state before each rearm.
  const HAL_StatusTypeDef dma_deinit_status = HAL_DMA_DeInit(huart_->hdmarx);
  rx_dma_deinit_status_ = static_cast<int32_t>(dma_deinit_status);
  if (dma_deinit_status != HAL_OK) {
    rx_start_status_ = rx_dma_deinit_status_;
    return false;
  }
  const HAL_StatusTypeDef dma_init_status = HAL_DMA_Init(huart_->hdmarx);
  rx_dma_init_status_ = static_cast<int32_t>(dma_init_status);
  if (dma_init_status != HAL_OK) {
    rx_start_status_ = rx_dma_init_status_;
    return false;
  }

  std::memset(dma_buf_a_, 0, kBufSize);
  HAL_StatusTypeDef ret = HAL_UART_Receive_DMA(huart_, dma_buf_a_, kBufSize);
  rx_hal_start_status_ = static_cast<int32_t>(ret);
  rx_start_status_ = static_cast<int32_t>(ret);
  if (ret != HAL_OK || !isReceiveActive()) {
    if (ret == HAL_OK) {
      rx_start_status_ = static_cast<int32_t>(HAL_ERROR);
      (void)HAL_UART_AbortReceive(huart_);
    }
    return false;
  }

  // RX is consumed by polling NDTR, so DMA half/full interrupts add no value.
  __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT | DMA_IT_TC);
  active_buf_ = dma_buf_a_;
  ROS_LOG_DEBUG("[DepthPorting] startRx() -> HAL_UART_Receive_DMA=%d, "
                "buf_a=%p, buf_b=%p, hdmarx=%p",
                (int)rx_start_status_, dma_buf_a_, dma_buf_b_, huart_->hdmarx);
  return true;
}

bool DepthCalcBoard_Porting::startRx() {
  uart_error_count_ = 0U;
  last_uart_error_ = 0U;
  rx_recovery_attempts_ = 0U;
  rx_recovery_successes_ = 0U;
  rx_recovery_failures_ = 0U;
  last_rx_recovery_reason_ = 0U;
  rx_recovery_pending_ = false;
  consecutive_recovery_failures_ = 0U;
  has_recovery_attempted_ = false;
  return startReceive();
}

bool DepthCalcBoard_Porting::serviceRxRecovery(
    bool no_valid_frame_timeout) {
  if (huart_ == nullptr || huart_->Instance == nullptr ||
      huart_->hdmarx == nullptr || huart_->hdmarx->Instance == nullptr) {
    return false;
  }

  const bool error_pending = rx_recovery_pending_;
  if (isReceiveActive() && !error_pending && !no_valid_frame_timeout) {
    return false;
  }
  if (!error_pending && !no_valid_frame_timeout) {
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
  const uint32_t now_ms = HAL_GetTick();
  if (has_recovery_attempted_ &&
      now_ms - last_recovery_attempt_ms_ < retry_delay_ms) {
    return false;
  }

  // Leave a new error raised during the HAL call pending for the next cycle.
  rx_recovery_pending_ = false;
  has_recovery_attempted_ = true;
  last_recovery_attempt_ms_ = now_ms;
  if (!error_pending && no_valid_frame_timeout) {
    last_rx_recovery_reason_ = 2U;
  }
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

void DepthCalcBoard_Porting::getDiagnosticsPort(
    void *ctx, auv::peripheral::DepthDiagnostics &out) {
  if (ctx != nullptr) {
    static_cast<DepthCalcBoard_Porting *>(ctx)->getDiagnostics(out);
  }
}

void DepthCalcBoard_Porting::handleHalError(UART_HandleTypeDef *huart) {
  if (active_instance != nullptr && huart != nullptr &&
      huart == active_instance->huart_) {
    ++active_instance->uart_error_count_;
    active_instance->last_uart_error_ = huart->ErrorCode;
    active_instance->rx_recovery_pending_ = true;
    active_instance->last_rx_recovery_reason_ = 1U;
  }
}

void DepthCalcBoard_Porting::getDiagnostics(
    auv::peripheral::DepthDiagnostics &out) const {
  out = {};
  out.has_uart_diagnostics = true;
  out.rx_start_status = rx_start_status_;
  out.rx_abort_status = rx_abort_status_;
  out.rx_dma_deinit_status = rx_dma_deinit_status_;
  out.rx_dma_init_status = rx_dma_init_status_;
  out.rx_hal_start_status = rx_hal_start_status_;
  out.rx_error_count = uart_error_count_;
  out.last_rx_error = last_uart_error_;
  out.rx_recovery_count = rx_recovery_successes_;
  out.last_rx_recovery_reason = last_rx_recovery_reason_;
  out.rx_recovery_attempts = rx_recovery_attempts_;
  out.rx_recovery_successes = rx_recovery_successes_;
  out.rx_recovery_failures = rx_recovery_failures_;
  out.rx_recovery_pending = rx_recovery_pending_;
  if (huart_ == nullptr || huart_->hdmarx == nullptr ||
      huart_->hdmarx->Instance == nullptr) {
    return;
  }
  const uint16_t remaining = __HAL_DMA_GET_COUNTER(huart_->hdmarx);
  if (remaining <= kBufSize) {
    out.dma_write_pos = (kBufSize - remaining) % kBufSize;
  }
  const auto *stream =
      static_cast<const DMA_Stream_TypeDef *>(huart_->hdmarx->Instance);
  out.rx_dma_active = isReceiveActive();
  out.dma_hal_state = static_cast<uint32_t>(huart_->hdmarx->State);
  out.dma_hal_error = huart_->hdmarx->ErrorCode;
  out.uart_hal_rx_state = static_cast<uint32_t>(huart_->RxState);
  out.last_rx_error |= huart_->ErrorCode;
}

void DepthCalcBoard_Porting::poll() {
  if (!huart_ || !huart_->hdmarx || !huart_->hdmarx->Instance ||
      !backend_ || !active_buf_)
    return;

  // Circular DMA producer position. The low-rate depth protocol is consumed
  // before one full lap, so a single ring buffer is sufficient.
  uint16_t remaining = __HAL_DMA_GET_COUNTER(huart_->hdmarx);
  if (remaining > kBufSize) return;
  // NDTR can briefly be zero before the circular DMA counter reloads.
  // Normalize it to ring position zero, which the consumer can reach.
  uint16_t received = (kBufSize - remaining) % kBufSize;

  while (dma_pos_ != received) {
    backend_->onRxByte(active_buf_[dma_pos_]);
    dma_pos_ = static_cast<uint16_t>((dma_pos_ + 1U) % kBufSize);
  }
}

} // namespace porting
} // namespace auv
