#include "INS_Porting.hpp"

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

bool INS_Porting::init() {
  for (int i = 0; i < 5; i++) {
    rx_write_idx_ = 0;
    rx_read_idx_ = 0;
    if (HAL_UART_Receive_DMA(rx_uart_, rx_buf_, rx_buf_size_) == HAL_OK)
      return true;
    HAL_Delay(10);
  }
  return false;
}

uint16_t INS_Porting::read(uint8_t *buf, uint16_t max_len) {
  uint16_t avail = (rx_write_idx_ >= rx_read_idx_)
                       ? (rx_write_idx_ - rx_read_idx_)
                       : (rx_buf_size_ - rx_read_idx_ + rx_write_idx_);
  if (avail == 0)
    return 0;

  uint16_t to_read = (avail < max_len) ? avail : max_len;
  for (uint16_t i = 0; i < to_read; i++) {
    buf[i] = rx_buf_[(rx_read_idx_ + i) % rx_buf_size_];
  }
  rx_read_idx_ = (rx_read_idx_ + to_read) % rx_buf_size_;
  return to_read;
}

bool INS_Porting::transmit(const uint8_t *data, uint16_t len) {
  for (int retry = 0; retry < 3; retry++) {
    if (HAL_UART_Transmit(tx_uart_, (uint8_t *)data, len, 50) == HAL_OK)
      return true;
  }
  return false;
}

bool INS_Porting::isDataFresh(uint32_t timeout_ms) const {
  return (HAL_GetTick() - last_rx_tick_ < timeout_ms);
}

void INS_Porting::onRxCompleted() {
  // DMA 半满/全满中断：更新写指针
  // HAL 的 HAL_UARTEx_RxEventCallback 或 HAL_UART_RxCpltCallback
  // 会传入实际接收到的数据量
  // 简化处理：使用 DMA 的 NDTR 计算已接收字节
  if (rx_uart_->hdmarx) {
    uint16_t remaining = __HAL_DMA_GET_COUNTER(rx_uart_->hdmarx);
    rx_write_idx_ = rx_buf_size_ - remaining;
  }
  last_rx_tick_ = HAL_GetTick();
}

} // namespace porting
} // namespace auv
