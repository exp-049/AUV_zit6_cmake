#pragma once

#include <cstdint>

// Minimal UART/DMA register model for testing the real text-depth port.
struct DMA_Stream_TypeDef {
  uint32_t CR = 0U;
  uint32_t NDTR = 0U;
};
struct DMA_HandleTypeDef {
  DMA_Stream_TypeDef *Instance = nullptr;
  uint32_t State = 1U;
  uint32_t ErrorCode = 0U;
};
struct USART_TypeDef {
  uint32_t CR3 = 0U;
};
enum HAL_UART_StateTypeDef {
  HAL_UART_STATE_READY = 0U,
  HAL_UART_STATE_BUSY_RX = 0x22U
};
struct UART_HandleTypeDef {
  USART_TypeDef *Instance = nullptr;
  DMA_HandleTypeDef *hdmarx = nullptr;
  uint32_t ErrorCode = 0U;
  HAL_UART_StateTypeDef RxState = HAL_UART_STATE_READY;
};
enum HAL_StatusTypeDef { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2 };

constexpr uint32_t USART_CR3_DMAR = 1U << 6;
constexpr uint32_t DMA_SxCR_EN = 1U;
constexpr uint32_t DMA_IT_HT = 1U << 1U;
constexpr uint32_t DMA_IT_TC = 1U << 2U;
#define __HAL_DMA_GET_COUNTER(handle) ((handle)->Instance->NDTR)
#define __HAL_DMA_DISABLE_IT(handle, interrupts) ((void)(handle), (void)(interrupts))

uint32_t HAL_GetTick();
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart);
HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *dma);
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *dma);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *uart,
                                       uint8_t *buffer, uint16_t size);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data,
                                    uint16_t size, uint32_t timeout);
