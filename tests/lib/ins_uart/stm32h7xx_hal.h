#pragma once

#include <cstdint>

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
  uint32_t ISR = 0U;
};
enum HAL_StatusTypeDef { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 };
enum HAL_UART_StateTypeDef {
  HAL_UART_STATE_RESET = 0U,
  HAL_UART_STATE_READY = 0x20U,
  HAL_UART_STATE_BUSY_RX = 0x22U
};
struct UART_HandleTypeDef {
  USART_TypeDef *Instance = nullptr;
  DMA_HandleTypeDef *hdmarx = nullptr;
  uint32_t ErrorCode = 0U;
  HAL_UART_StateTypeDef RxState = HAL_UART_STATE_READY;
  HAL_UART_StateTypeDef gState = HAL_UART_STATE_READY;
  uint8_t *pRxBuffPtr = nullptr;
  uint16_t RxXferSize = 0U;
};

constexpr uint32_t USART_CR3_DMAR = 1U << 6U;
constexpr uint32_t DMA_SxCR_EN = 1U;
constexpr uint32_t DMA_IT_HT = 1U << 1U;
constexpr uint32_t DMA_IT_TC = 1U << 2U;

#define __HAL_DMA_GET_COUNTER(handle) ((handle)->Instance->NDTR)
#define __HAL_DMA_DISABLE_IT(handle, interrupts) ((void)(handle), (void)(interrupts))

uint32_t HAL_GetTick();
void HAL_Delay(uint32_t delay_ms);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart);
HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *dma);
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *dma);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *uart,
                                       uint8_t *buffer, uint16_t size);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data,
                                    uint16_t size, uint32_t timeout);
