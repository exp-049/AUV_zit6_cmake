#include <uxr/client/transport.h>

#include <rmw_microxrcedds_c/config.h>

#include "main.h"
#include "cmsis_os.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

// --- micro-ROS Transports Config ---
#define UART_DMA_BUFFER_SIZE 2048

// RX buffer in D2 RAM
uint8_t dma_buffer[UART_DMA_BUFFER_SIZE] __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)));
static size_t dma_head = 0, dma_tail = 0;

// TX Ring Buffer in D2 RAM
static uint8_t tx_ring_buffer[UART_DMA_BUFFER_SIZE] __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)));
static volatile size_t tx_head = 0; // Data entry point
static volatile size_t tx_tail = 0; // Data exit point (DMA start)
static volatile size_t tx_last_len = 0; // Length of the active DMA transfer
static volatile bool tx_busy = false;

// Helper to trigger the next DMA chunk from the ring buffer
static void start_next_tx(UART_HandleTypeDef* huart) {
    if (tx_busy || tx_head == tx_tail) return;

    size_t len;
    if (tx_head > tx_tail) {
        len = tx_head - tx_tail;
    } else {
        len = UART_DMA_BUFFER_SIZE - tx_tail;
    }

    if (len > 0) {
        tx_busy = true;
        tx_last_len = len;
        HAL_UART_Transmit_DMA(huart, &tx_ring_buffer[tx_tail], len);
    }
}

// UART TX Complete Callback
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    // 只有当回调来自于 micro-ROS 使用的串口时才处理
    if (huart->Instance != USART2) return;

    // Update tx_tail using the length that was actually sent
    tx_tail = (tx_tail + tx_last_len) % UART_DMA_BUFFER_SIZE;
    tx_busy = false;

    // Try to send next chunk
    start_next_tx(huart);
}

bool cubemx_transport_open(struct uxrCustomTransport * transport){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
    HAL_UART_Receive_DMA(uart, dma_buffer, UART_DMA_BUFFER_SIZE);
    tx_head = 0;
    tx_tail = 0;
    tx_last_len = 0;
    tx_busy = false;
    dma_head = 0;
    dma_tail = 0;
    return true;
}

bool cubemx_transport_close(struct uxrCustomTransport * transport){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
    HAL_UART_DMAStop(uart);
    return true;
}

size_t cubemx_transport_write(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, uint8_t * err){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;

    size_t free_space;
    size_t total_written = 0;

    // Use a simple loop to copy data into ring buffer
    // To keep it non-blocking, we only write what fits.
    // micro-ROS will retry if we return less than len.
    
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    size_t current_head = tx_head;
    size_t current_tail = tx_tail;

    if (current_head >= current_tail) {
        free_space = UART_DMA_BUFFER_SIZE - (current_head - current_tail) - 1;
    } else {
        free_space = current_tail - current_head - 1;
    }

    size_t to_write = (len < free_space) ? len : free_space;

    for (size_t i = 0; i < to_write; i++) {
        tx_ring_buffer[tx_head] = buf[i];
        tx_head = (tx_head + 1) % UART_DMA_BUFFER_SIZE;
    }

    total_written = to_write;

    /* Defer starting the HAL DMA until after interrupts are restored. */
    bool need_start = false;
    if (!tx_busy && (tx_head != tx_tail)) {
        need_start = true;
    }

    if (primask == 0) __enable_irq();

    if (need_start) {
        start_next_tx(uart);
    }

    return total_written;
}

size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;

    uint32_t start_tick = osKernelGetTickCount();
    
    // Update tail once before checking
    dma_tail = (UART_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(uart->hdmarx)) % UART_DMA_BUFFER_SIZE;

    // Use while loop to avoid mandatory delay when timeout is 0 or data is already here
    while (dma_head == dma_tail && (osKernelGetTickCount() - start_tick) < (uint32_t)timeout) {
        osDelay(1);
        dma_tail = (UART_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(uart->hdmarx)) % UART_DMA_BUFFER_SIZE;
    }
    
    size_t wrote = 0;
    while ((dma_head != dma_tail) && (wrote < len)){
        buf[wrote] = dma_buffer[dma_head];
        dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
        wrote++;
    }
    
    return wrote;
}

#endif //RMW_UXRCE_TRANSPORT_CUSTOM