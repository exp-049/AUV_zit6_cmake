#ifndef __SERIAL_PORT_HPP
#define __SERIAL_PORT_HPP

#include "usart.h"
#include <stdint.h>
#include <string.h>
#include "main.h"
namespace auv {
namespace porting {

class SerialPort {
public:
    enum class Mode {
        RECEIVE_ONLY,
        TRANSMIT_ONLY,
        DUPLEX
    };

    static constexpr uint16_t kMaxRxBufferSize = 512;

    SerialPort(UART_HandleTypeDef* huart, uint8_t* ext_rx_buf, uint16_t rx_buf_size);

    SerialPort(UART_HandleTypeDef* huart, uint16_t rx_buf_size = kMaxRxBufferSize);

    ~SerialPort();

    // 启动 DMA 循环接收
    bool startReceive();

    // 发送数据 (DMA 方式)
    bool transmit(const uint8_t* data, uint16_t len);

    // 调试专用：通过 UART5 使用 DMA 非阻塞发送。
    // 如果 `huart5.gState` 非 `HAL_UART_STATE_READY` 则直接丢弃并返回 false。
    static bool transmitDebug(const uint8_t* data, uint16_t len);

    // 从循环缓冲区读取新数据，返回读取到的字节数
    uint16_t read(uint8_t* out_buf, uint16_t max_len);

private:
    UART_HandleTypeDef* huart_;
    uint8_t* rx_buffer_ptr_;
    alignas(32) uint8_t rx_buffer_internal_[kMaxRxBufferSize] = {0};
    uint16_t rx_buf_size_;
    uint16_t last_rx_pos_ = 0;
};

} // namespace porting
} // namespace auv

#endif
