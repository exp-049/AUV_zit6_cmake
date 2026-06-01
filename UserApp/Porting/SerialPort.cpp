#include "SerialPort.hpp"
#include <cstring>


namespace auv {
namespace porting {

SerialPort::SerialPort(UART_HandleTypeDef* huart, uint8_t* ext_rx_buf, uint16_t rx_buf_size)
    : huart_(huart), rx_buffer_ptr_(ext_rx_buf), rx_buf_size_(rx_buf_size) {}

SerialPort::SerialPort(UART_HandleTypeDef* huart, uint16_t rx_buf_size)
    : huart_(huart), rx_buffer_ptr_(rx_buffer_internal_), rx_buf_size_(rx_buf_size > kMaxRxBufferSize ? kMaxRxBufferSize : rx_buf_size) {}

SerialPort::~SerialPort() = default;

bool SerialPort::startReceive() {
    __HAL_UART_CLEAR_FLAG(huart_, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
    return HAL_UART_Receive_DMA(huart_, rx_buffer_ptr_, rx_buf_size_) == HAL_OK;
}

bool SerialPort::transmit(const uint8_t* data, uint16_t len) {
    if (huart_->hdmatx != NULL) {
        DMA_Stream_TypeDef *dma_stream = (DMA_Stream_TypeDef *)huart_->hdmatx->Instance;
        if ((dma_stream->CR & DMA_SxCR_EN) != 0U) {
            return false;
        }
    }
    return HAL_UART_Transmit_DMA(huart_, const_cast<uint8_t*>(data), len) == HAL_OK;
}

uint16_t SerialPort::read(uint8_t* out_buf, uint16_t max_len) {
    // Invalidate D-Cache to ensure the CPU reads the data actually written by DMA to RAM
    SCB_InvalidateDCache_by_Addr((uint32_t*)rx_buffer_ptr_, rx_buf_size_);

    uint16_t current_pos = rx_buf_size_ - __HAL_DMA_GET_COUNTER(huart_->hdmarx);
    uint16_t bytes_to_read = 0;

    if (current_pos != last_rx_pos_) {
        if (current_pos > last_rx_pos_) {
            bytes_to_read = current_pos - last_rx_pos_;
            if (bytes_to_read > max_len) bytes_to_read = max_len;
            std::memcpy(out_buf, rx_buffer_ptr_ + last_rx_pos_, bytes_to_read);
        } else {
            uint16_t first_part = rx_buf_size_ - last_rx_pos_;
            bytes_to_read = first_part + current_pos;
            if (bytes_to_read > max_len) bytes_to_read = max_len;

            if (bytes_to_read <= first_part) {
                std::memcpy(out_buf, rx_buffer_ptr_ + last_rx_pos_, bytes_to_read);
            } else {
                std::memcpy(out_buf, rx_buffer_ptr_ + last_rx_pos_, first_part);
                std::memcpy(out_buf + first_part, rx_buffer_ptr_, bytes_to_read - first_part);
            }
        }
        last_rx_pos_ = current_pos;
    }
    return bytes_to_read;
}

} // namespace porting
} // namespace auv
