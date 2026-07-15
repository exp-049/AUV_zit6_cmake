#include "USBL_Porting.hpp"
#include "USBL_Driver.hpp"

#include <cstring>

namespace auv {
namespace porting {

__attribute__((section(".dma_buffer"))) uint8_t usbl_rx_buffer[512];
USBL_Porting *USBL_Porting::active_instance = nullptr;

USBL_Porting::USBL_Porting(UART_HandleTypeDef *uart, uint8_t *buffer,
                           uint16_t buffer_size)
    : uart_(uart), rx_buffer_(buffer), buffer_size_(buffer_size) {
  active_instance = this;
}

bool USBL_Porting::initPort(void *ctx) {
  return static_cast<USBL_Porting *>(ctx)->init();
}

uint16_t USBL_Porting::readPort(void *ctx, uint8_t *buffer,
                                uint16_t max_len) {
  return static_cast<USBL_Porting *>(ctx)->read(buffer, max_len);
}

void USBL_Porting::diagnosticsPort(
    void *ctx, auv::peripheral::UsblPortDiagnostics *out) {
  static_cast<USBL_Porting *>(ctx)->diagnostics(out);
}

uint32_t USBL_Porting::getTickPort(void *ctx) {
  return static_cast<USBL_Porting *>(ctx)->getTick();
}

bool USBL_Porting::init() {
  if (uart_ == nullptr || uart_->hdmarx == nullptr || rx_buffer_ == nullptr ||
      buffer_size_ == 0) {
    return false;
  }

  read_pos_ = 0;
  events_ = 0;
  invalid_events_ = 0;
  std::memset(rx_buffer_, 0, buffer_size_);

  // USART3 is configured as circular RX DMA in CubeMX.  ReceiveToIdle keeps
  // the IDLE event enabled for low-latency wakeups while the actual consumer
  // uses the DMA counter, so no global byte callback is needed in NORMAL.
  const HAL_StatusTypeDef status =
      HAL_UARTEx_ReceiveToIdle_DMA(uart_, rx_buffer_, buffer_size_);
  if (status != HAL_OK) {
    return false;
  }

  // The 133-byte USBL frame does not need half-transfer notifications.  This
  // avoids an unnecessary interrupt while retaining IDLE and TC handling.
  __HAL_DMA_DISABLE_IT(uart_->hdmarx, DMA_IT_HT);
  return true;
}

uint16_t USBL_Porting::read(uint8_t *buffer, uint16_t max_len) {
  if (buffer == nullptr || max_len == 0 || uart_ == nullptr ||
      uart_->hdmarx == nullptr || buffer_size_ == 0) {
    return 0;
  }

  const uint16_t remaining = __HAL_DMA_GET_COUNTER(uart_->hdmarx);
  if (remaining > buffer_size_) {
    return 0;
  }

  const uint16_t write_pos =
      static_cast<uint16_t>(buffer_size_ - remaining);
  const uint16_t available =
      (write_pos >= read_pos_)
          ? static_cast<uint16_t>(write_pos - read_pos_)
          : static_cast<uint16_t>(buffer_size_ - read_pos_ + write_pos);
  const uint16_t count = (available < max_len) ? available : max_len;

  for (uint16_t i = 0; i < count; ++i) {
    buffer[i] = rx_buffer_[(read_pos_ + i) % buffer_size_];
  }
  read_pos_ = static_cast<uint16_t>((read_pos_ + count) % buffer_size_);
  return count;
}

void USBL_Porting::onRxEvent(uint16_t size) {
  ++events_;
  if (size > buffer_size_) {
    ++invalid_events_;
  }
}

void USBL_Porting::diagnostics(
    auv::peripheral::UsblPortDiagnostics *out) const {
  if (out == nullptr) {
    return;
  }
  out->events = events_;
  out->invalid_events = invalid_events_;
  if (uart_ == nullptr || uart_->hdmarx == nullptr || rx_buffer_ == nullptr) {
    return;
  }
  const uint16_t remaining = __HAL_DMA_GET_COUNTER(uart_->hdmarx);
  out->dma_remaining = remaining;
  if (remaining <= buffer_size_) {
    out->write_pos = static_cast<uint16_t>(buffer_size_ - remaining);
  }
  const auto *stream = reinterpret_cast<const DMA_Stream_TypeDef *>(
      uart_->hdmarx->Instance);
  out->dma_enabled = (stream->CR & DMA_SxCR_EN) != 0;
  out->uart_isr = uart_->Instance->ISR;
  std::memcpy(out->rx_preview, rx_buffer_, sizeof(out->rx_preview));
}

uint32_t USBL_Porting::getTick() const { return HAL_GetTick(); }

} // namespace porting
} // namespace auv

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                             uint16_t size) {
  if (auv::porting::USBL_Porting::active_instance == nullptr ||
      huart != auv::porting::USBL_Porting::active_instance->getUart()) {
    return;
  }
  auv::porting::USBL_Porting::active_instance->onRxEvent(size);
}
