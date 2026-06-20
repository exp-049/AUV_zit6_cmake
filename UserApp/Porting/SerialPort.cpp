#include "SerialPort.hpp"
#include <cstring>


namespace auv {
namespace porting {

SerialPort::SerialPort(UART_HandleTypeDef* huart, uint8_t* ext_rx_buf, uint16_t rx_buf_size)
    : huart_(huart), rx_buffer_ptr_(ext_rx_buf), rx_buf_size_(rx_buf_size) {}

SerialPort::SerialPort(UART_HandleTypeDef* huart, uint16_t rx_buf_size)
    : huart_(huart), rx_buffer_ptr_(rx_buffer_internal_), rx_buf_size_(rx_buf_size > kMaxRxBufferSize ? kMaxRxBufferSize : rx_buf_size) {}

SerialPort::~SerialPort() = default;

bool SerialPort::startReceive() noexcept {
    __HAL_UART_CLEAR_FLAG(huart_, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
    return HAL_UART_Receive_DMA(huart_, rx_buffer_ptr_, rx_buf_size_) == HAL_OK;
}

bool SerialPort::transmit(const uint8_t* data, uint16_t len) noexcept {
    if (huart_->hdmatx != NULL) {
        DMA_Stream_TypeDef *dma_stream = (DMA_Stream_TypeDef *)huart_->hdmatx->Instance;
        if ((dma_stream->CR & DMA_SxCR_EN) != 0U) {
            return false;
        }
    }
    return HAL_UART_Transmit_DMA(huart_, const_cast<uint8_t*>(data), len) == HAL_OK;
}

uint16_t SerialPort::read(uint8_t* out_buf, uint16_t max_len) noexcept {
    // Invalidate D-Cache to ensure the CPU reads the data actually written by DMA to RAM
    SCB_InvalidateDCache_by_Addr((uint32_t*)rx_buffer_ptr_, rx_buf_size_);

    const uint16_t current_pos =
        rx_buf_size_ - __HAL_DMA_GET_COUNTER(huart_->hdmarx);

    if (current_pos == last_rx_pos_) {
        return 0;
    }

    uint16_t bytes_to_read;

    if (current_pos > last_rx_pos_) {
        // 正常线性增长：新数据在 [last_rx_pos_, current_pos)
        bytes_to_read = current_pos - last_rx_pos_;
        if (bytes_to_read > max_len) bytes_to_read = max_len;
        std::memcpy(out_buf, rx_buffer_ptr_ + last_rx_pos_, bytes_to_read);
    } else {
        // 环形翻转：新数据分布在 [last_rx_pos_, rx_buf_size_) + [0, current_pos)
        const uint16_t first_part  = rx_buf_size_ - last_rx_pos_;
        const uint16_t second_part = current_pos;
        const uint16_t total_avail = first_part + second_part;
        const uint16_t to_copy     = (total_avail > max_len) ? max_len : total_avail;

        if (to_copy <= first_part) {
            // 只需从尾部读一段
            std::memcpy(out_buf, rx_buffer_ptr_ + last_rx_pos_, to_copy);
        } else {
            // 需分段：尾部 + 头部
            std::memcpy(out_buf, rx_buffer_ptr_ + last_rx_pos_, first_part);
            const uint16_t remain = to_copy - first_part;
            std::memcpy(out_buf + first_part, rx_buffer_ptr_, remain);
        }
        bytes_to_read = to_copy;
    }

    last_rx_pos_ = (last_rx_pos_ + bytes_to_read) % rx_buf_size_;
    return bytes_to_read;
}

} // namespace porting
} // namespace auv
