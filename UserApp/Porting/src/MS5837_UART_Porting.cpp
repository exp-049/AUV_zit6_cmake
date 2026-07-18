#include "MS5837_UART_Porting.hpp"

#include "MS5837_LogConfig.hpp"
#include "stm32h7xx_hal_uart_ex.h"

namespace {
__attribute__((section(".dma_buffer"))) uint8_t
    g_ms5837_uart4_dma_buffer[auv::porting::MS5837_UART_Porting::kDmaBufferSize];
}

namespace auv {
namespace porting {

MS5837_UART_Porting *MS5837_UART_Porting::active_instance_ = nullptr;

MS5837_UART_Porting::MS5837_UART_Porting(
    UART_HandleTypeDef *huart, auv::peripheral::UART_MS5837Backend *backend)
    : huart_(huart), backend_(backend), dma_buffer_(g_ms5837_uart4_dma_buffer) {
  MS5837_LOG_DEBUG("[MS5837 UART port] constructed, huart=%p", huart_);
}

bool MS5837_UART_Porting::transmitPort(void *ctx, const uint8_t *data,
                                       uint16_t length) {
  auto *self = static_cast<MS5837_UART_Porting *>(ctx);
  if (self == nullptr || self->huart_ == nullptr || data == nullptr ||
      length == 0U) {
    return false;
  }

  return HAL_UART_Transmit(self->huart_, const_cast<uint8_t *>(data), length,
                           100U) == HAL_OK;
}

void MS5837_UART_Porting::pollPort(void *ctx) {
  if (ctx != nullptr) {
    static_cast<MS5837_UART_Porting *>(ctx)->poll();
  }
}

bool MS5837_UART_Porting::startRxPort(void *ctx) {
  return ctx != nullptr && static_cast<MS5837_UART_Porting *>(ctx)->startRx();
}

uint32_t MS5837_UART_Porting::getTickPort(void *ctx) {
  (void)ctx;
  return HAL_GetTick();
}

uint32_t MS5837_UART_Porting::getRxRecoveryCountPort(void *ctx) {
  if (ctx == nullptr) {
    return 0U;
  }
  return static_cast<MS5837_UART_Porting *>(ctx)->recovery_count_;
}

uint32_t MS5837_UART_Porting::getRxErrorCountPort(void *ctx) {
  return ctx != nullptr
             ? static_cast<MS5837_UART_Porting *>(ctx)->uart_error_count_
             : 0U;
}

uint32_t MS5837_UART_Porting::getLastRxErrorPort(void *ctx) {
  return ctx != nullptr
             ? static_cast<MS5837_UART_Porting *>(ctx)->last_uart_error_
             : 0U;
}

uint32_t MS5837_UART_Porting::getLastRxRecoveryReasonPort(void *ctx) {
  return ctx != nullptr
             ? static_cast<MS5837_UART_Porting *>(ctx)->last_recovery_reason_
             : 0U;
}

uint32_t MS5837_UART_Porting::getRxEventCountPort(void *ctx) {
  return ctx != nullptr
             ? static_cast<MS5837_UART_Porting *>(ctx)->rx_event_count_
             : 0U;
}

uint32_t MS5837_UART_Porting::getDmaWritePosPort(void *ctx) {
  if (ctx == nullptr) {
    return 0U;
  }
  auto *self = static_cast<MS5837_UART_Porting *>(ctx);
  if (self->huart_ == nullptr || self->huart_->hdmarx == nullptr) {
    return 0U;
  }
  const uint16_t remaining = __HAL_DMA_GET_COUNTER(self->huart_->hdmarx);
  return remaining <= kDmaBufferSize ? kDmaBufferSize - remaining : 0U;
}

void MS5837_UART_Porting::handleHalRxEvent(UART_HandleTypeDef *huart,
                                           uint16_t size) {
  if (active_instance_ != nullptr && huart == active_instance_->huart_) {
    active_instance_->onRxEvent(size);
  }
}

void MS5837_UART_Porting::handleHalError(UART_HandleTypeDef *huart) {
  if (active_instance_ != nullptr && huart == active_instance_->huart_) {
    ++active_instance_->uart_error_count_;
    active_instance_->last_uart_error_ = huart->ErrorCode;
  }
}

bool MS5837_UART_Porting::startIdleDma() {
  if (huart_ == nullptr || huart_->hdmarx == nullptr) {
    return false;
  }

  const HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(
      huart_, dma_buffer_, kDmaBufferSize);
  if (status != HAL_OK) {
    MS5837_LOG_DEBUG(
        "[MS5837 UART port] HAL_UARTEx_ReceiveToIdle_DMA failed: %d",
        (int)status);
    return false;
  }

  // IDLE is the packet-boundary notification. Half-transfer notifications
  // are unnecessary because poll() reads the circular DMA write position.
  __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
  dma_pos_ = 0U;
  last_progress_ms_ = HAL_GetTick();
  rx_event_pending_ = false;
  return true;
}

bool MS5837_UART_Porting::startRx() {
  active_instance_ = this;
  dma_pos_ = 0U;
  last_progress_ms_ = HAL_GetTick();
  last_recovery_ms_ = last_progress_ms_;
  recovery_count_ = 0U;
  rx_event_count_ = 0U;
  uart_error_count_ = 0U;
  last_uart_error_ = 0U;
  last_recovery_reason_ = 0U;
  last_polled_event_count_ = 0U;
  last_event_size_ = 0U;
  rx_event_pending_ = false;
  return startIdleDma();
}

void MS5837_UART_Porting::onRxEvent(uint16_t size) {
  // This runs from UART/DMA interrupt context. Do not parse protocol frames
  // or call logging here; only publish a small event for the polling task.
  last_event_size_ = size;
  ++rx_event_count_;
  rx_event_pending_ = true;
}

bool MS5837_UART_Porting::dmaIsActive() const {
  if (huart_ == nullptr || huart_->hdmarx == nullptr ||
      huart_->hdmarx->Instance == nullptr) {
    return false;
  }

  // In ReceiveToIdle circular mode HAL may report RxState=READY while an
  // IDLE/transfer event is being dispatched. DMAR + stream enable are the
  // actual hardware liveness indicators and avoid a false recovery here.
  const bool uart_rx_active =
      (huart_->Instance->CR3 & USART_CR3_DMAR) != 0U;
  const auto *stream =
      static_cast<const DMA_Stream_TypeDef *>(huart_->hdmarx->Instance);
  const bool dma_stream_active = (stream->CR & DMA_SxCR_EN) != 0U;
  return uart_rx_active && dma_stream_active;
}

void MS5837_UART_Porting::recoverRx(uint32_t now_ms, uint32_t reason) {
  if ((uint32_t)(now_ms - last_recovery_ms_) < kRecoveryIntervalMs) {
    return;
  }

  last_recovery_ms_ = now_ms;
  ++recovery_count_;
  last_recovery_reason_ = reason;

  (void)HAL_UART_AbortReceive(huart_);
  if (huart_->hdmarx != nullptr && huart_->hdmarx->Instance != nullptr) {
    const auto *stream =
        static_cast<const DMA_Stream_TypeDef *>(huart_->hdmarx->Instance);
    if ((stream->CR & DMA_SxCR_EN) != 0U) {
      (void)HAL_DMA_Abort(huart_->hdmarx);
    }
  }

  dma_pos_ = 0U;
  last_event_size_ = 0U;
  rx_event_pending_ = false;
  if (backend_ != nullptr) {
    backend_->onRxReset();
  }

  const bool restarted = startIdleDma();
  MS5837_LOG_DEBUG("[MS5837 UART port] RX recovery #%lu: restarted=%d",
                   (unsigned long)recovery_count_, restarted ? 1 : 0);
}

void MS5837_UART_Porting::processAvailable() {
  if (huart_ == nullptr || huart_->hdmarx == nullptr || backend_ == nullptr) {
    return;
  }

  const uint16_t remaining = __HAL_DMA_GET_COUNTER(huart_->hdmarx);
  if (remaining > kDmaBufferSize) {
    return;
  }

  const uint16_t write_pos = (uint16_t)(kDmaBufferSize - remaining);
  if (write_pos == dma_pos_) {
    return;
  }

  if (write_pos > dma_pos_) {
    while (dma_pos_ < write_pos) {
      backend_->onRxByte(dma_buffer_[dma_pos_]);
      ++dma_pos_;
    }
  } else {
    while (dma_pos_ < kDmaBufferSize) {
      backend_->onRxByte(dma_buffer_[dma_pos_]);
      ++dma_pos_;
    }
    dma_pos_ = 0U;
    while (dma_pos_ < write_pos) {
      backend_->onRxByte(dma_buffer_[dma_pos_]);
      ++dma_pos_;
    }
  }

  last_progress_ms_ = HAL_GetTick();
}

void MS5837_UART_Porting::poll() {
  if (huart_ == nullptr || huart_->hdmarx == nullptr || backend_ == nullptr) {
    return;
  }

  const uint32_t now_ms = HAL_GetTick();
  const uint32_t event_count = rx_event_count_;
  const bool new_event = event_count != last_polled_event_count_;
  last_polled_event_count_ = event_count;
  const bool idle_event = rx_event_pending_;
  processAvailable();
  rx_event_pending_ = false;

  // In circular ReceiveToIdle mode, the HAL event is itself proof that bytes
  // reached the UART/DMA path. The DMA counter can be observed at the same
  // instant as an IDLE/TC interrupt and appear unchanged for one poll.
  if (idle_event || new_event) {
    last_progress_ms_ = now_ms;
  }

  // Circular DMA is deliberately allowed to remain idle indefinitely. A
  // missing DATA frame is not evidence that the receiver is broken: the
  // sensor may pause transmission, and ReceiveToIdle callbacks are only
  // notifications, not a liveness guarantee. Recover only when the hardware
  // receive path is actually disabled.
  const bool active = dmaIsActive();
  if (!active) {
    recoverRx(now_ms, 1U);
  }
}

} // namespace porting
} // namespace auv
