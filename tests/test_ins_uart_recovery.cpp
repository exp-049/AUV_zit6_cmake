#include "INS_Porting.hpp"
#include "INS_Driver.hpp"

#include <gtest/gtest.h>

namespace {
uint32_t tick_ms = 0U;
HAL_StatusTypeDef receive_status = HAL_OK;
HAL_StatusTypeDef abort_status = HAL_OK;
bool activate_receive = true;
uint32_t receive_calls = 0U;
uint32_t abort_calls = 0U;
uint8_t *active_buffer = nullptr;
uint16_t active_size = 0U;

void disableReceive(UART_HandleTypeDef &uart) {
  uart.Instance->CR3 &= ~USART_CR3_DMAR;
  uart.hdmarx->Instance->CR &= ~DMA_SxCR_EN;
  uart.RxState = HAL_UART_STATE_READY;
}

class InsUartRecoveryTest : public ::testing::Test {
protected:
  void SetUp() override {
    tick_ms = 0U;
    receive_status = HAL_OK;
    abort_status = HAL_OK;
    activate_receive = true;
    receive_calls = 0U;
    abort_calls = 0U;
    active_buffer = nullptr;
    active_size = 0U;
  }

  DMA_Stream_TypeDef stream;
  DMA_HandleTypeDef dma{&stream};
  USART_TypeDef registers;
  UART_HandleTypeDef uart{&registers, &dma};
  uint8_t buffer[512]{};
  auv::porting::INS_Porting port{&uart, &uart, buffer, sizeof(buffer)};
};
} // namespace

uint32_t HAL_GetTick() { return tick_ms; }

void HAL_Delay(uint32_t delay_ms) { tick_ms += delay_ms; }

HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart) {
  ++abort_calls;
  if (abort_status != HAL_OK) {
    return abort_status;
  }
  if ((uart->Instance->CR3 & USART_CR3_DMAR) != 0U) {
    disableReceive(*uart);
    uart->hdmarx->State = 1U;
  } else {
    uart->RxState = HAL_UART_STATE_READY;
  }
  return HAL_OK;
}

HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *dma) {
  dma->Instance->CR = 0U;
  dma->Instance->NDTR = 0U;
  dma->State = 0U;
  dma->ErrorCode = 0U;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *dma) {
  dma->State = 1U;
  dma->ErrorCode = 0U;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *uart,
                                       uint8_t *buffer, uint16_t size) {
  ++receive_calls;
  if (receive_status != HAL_OK) {
    return receive_status;
  }
  if (uart->hdmarx->State != 1U) {
    uart->hdmarx->ErrorCode = 0x10U;
    return HAL_ERROR;
  }
  uart->pRxBuffPtr = buffer;
  uart->RxXferSize = size;
  uart->RxState = HAL_UART_STATE_BUSY_RX;
  uart->hdmarx->State = 2U;
  if (activate_receive) {
    uart->Instance->CR3 |= USART_CR3_DMAR;
    uart->hdmarx->Instance->CR |= DMA_SxCR_EN;
    uart->hdmarx->Instance->NDTR = size;
    active_buffer = buffer;
    active_size = size;
  }
  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t,
                                    uint32_t) {
  return HAL_OK;
}

TEST_F(InsUartRecoveryTest, StartsAndVerifiesCircularReceive) {
  ASSERT_TRUE(port.init());
  EXPECT_EQ(receive_calls, 1U);
  EXPECT_EQ(active_buffer, buffer);
  EXPECT_EQ(active_size, sizeof(buffer));
  auv::peripheral::InsPortDiagnostics diagnostics;
  port.diagnostics(&diagnostics);
  EXPECT_TRUE(diagnostics.dma_enabled);
  EXPECT_EQ(diagnostics.rx_start_status, HAL_OK);
}

TEST_F(InsUartRecoveryTest, DoesNotReadConsumedBytesAgainAtZeroNdtr) {
  ASSERT_TRUE(port.init());
  for (uint16_t i = 0U; i < sizeof(buffer); ++i) {
    buffer[i] = static_cast<uint8_t>(i);
  }
  uint8_t output[256]{};

  stream.NDTR = 256U;
  ASSERT_EQ(port.read(output, sizeof(output)), 256U);
  stream.NDTR = 0U;
  ASSERT_EQ(port.read(output, sizeof(output)), 256U);

  // No further DMA progress after the consumer reaches ring position zero.
  EXPECT_EQ(port.read(output, sizeof(output)), 0U);
  stream.NDTR = sizeof(buffer); // The circular counter reloads to the same pos.
  EXPECT_EQ(port.read(output, sizeof(output)), 0U);

  auv::peripheral::InsPortDiagnostics diagnostics;
  port.diagnostics(&diagnostics);
  EXPECT_EQ(diagnostics.total_bytes, sizeof(buffer));
  EXPECT_EQ(diagnostics.read_events, 2U);
}

TEST_F(InsUartRecoveryTest, ReadsInStreamOrderAcrossRingWrap) {
  ASSERT_TRUE(port.init());
  for (uint16_t i = 0U; i < sizeof(buffer); ++i) {
    buffer[i] = static_cast<uint8_t>(i);
  }
  uint8_t output[512]{};
  stream.NDTR = 12U; // Producer at 500; consume everything up to it.
  ASSERT_EQ(port.read(output, sizeof(output)), 500U);

  for (uint16_t i = 0U; i < 10U; ++i) {
    buffer[i] = static_cast<uint8_t>(0xA0U + i);
  }
  stream.NDTR = 502U; // Producer has wrapped to position 10.
  ASSERT_EQ(port.read(output, sizeof(output)), 22U);
  for (uint16_t i = 0U; i < 12U; ++i) {
    EXPECT_EQ(output[i], static_cast<uint8_t>(500U + i));
  }
  for (uint16_t i = 0U; i < 10U; ++i) {
    EXPECT_EQ(output[12U + i], static_cast<uint8_t>(0xA0U + i));
  }
  EXPECT_EQ(port.read(output, sizeof(output)), 0U);
}

TEST_F(InsUartRecoveryTest, ReportsZeroNdtrAsRingPositionZero) {
  ASSERT_TRUE(port.init());
  stream.NDTR = 0U;
  auv::peripheral::InsPortDiagnostics diagnostics;
  port.diagnostics(&diagnostics);
  EXPECT_EQ(diagnostics.write_pos, 0U);
  EXPECT_EQ(diagnostics.dma_remaining, 0U);
}

TEST_F(InsUartRecoveryTest, KeepsHealthyDmaRunningDuringBriefSignalGap) {
  ASSERT_TRUE(port.init());
  EXPECT_FALSE(port.serviceRxRecovery(false));
  EXPECT_EQ(receive_calls, 1U);
  EXPECT_TRUE((registers.CR3 & USART_CR3_DMAR) != 0U);
  EXPECT_TRUE((stream.CR & DMA_SxCR_EN) != 0U);
}

TEST_F(InsUartRecoveryTest, RearmsEvenIfDmaLooksActiveAfterFrameTimeout) {
  ASSERT_TRUE(port.init());
  EXPECT_TRUE(port.serviceRxRecovery(true));
  EXPECT_EQ(receive_calls, 2U);
  EXPECT_FALSE(port.serviceRxRecovery(true));
  tick_ms = 4999U;
  EXPECT_FALSE(port.serviceRxRecovery(true));
  tick_ms = 5000U;
  EXPECT_TRUE(port.serviceRxRecovery(true));
  EXPECT_EQ(receive_calls, 3U);
}

TEST_F(InsUartRecoveryTest, ReArmsAfterUartErrorAndVerifiesHardwareState) {
  ASSERT_TRUE(port.init());
  uart.ErrorCode = 0x08U; // HAL_UART_ERROR_ORE
  port.onHalError(&uart);
  disableReceive(uart); // HAL aborts DMA before invoking ErrorCallback.

  EXPECT_TRUE(port.serviceRxRecovery(false));
  EXPECT_EQ(receive_calls, 2U);
  EXPECT_TRUE((registers.CR3 & USART_CR3_DMAR) != 0U);
  EXPECT_TRUE((stream.CR & DMA_SxCR_EN) != 0U);
  auv::peripheral::InsPortDiagnostics diagnostics;
  port.diagnostics(&diagnostics);
  EXPECT_EQ(diagnostics.uart_error_count, 1U);
  EXPECT_EQ(diagnostics.last_uart_error, 0x08U);
  EXPECT_EQ(diagnostics.rx_recovery_attempts, 1U);
  EXPECT_EQ(diagnostics.rx_recovery_successes, 1U);
  EXPECT_EQ(diagnostics.rx_recovery_failures, 0U);
  EXPECT_FALSE(diagnostics.rx_recovery_pending);
}

TEST_F(InsUartRecoveryTest, RetriesWithIncreasingBackoffThenCooldown) {
  ASSERT_TRUE(port.init());
  port.onHalError(&uart);
  disableReceive(uart);
  receive_status = HAL_ERROR;

  EXPECT_FALSE(port.serviceRxRecovery(false)); // immediate attempt, fails
  EXPECT_FALSE(port.serviceRxRecovery(false));
  EXPECT_EQ(receive_calls, 2U);

  const uint32_t retry_times[] = {100U, 350U, 850U, 1850U, 3850U, 8850U};
  for (uint32_t retry_time : retry_times) {
    tick_ms = retry_time - 1U;
    EXPECT_FALSE(port.serviceRxRecovery(false));
    tick_ms = retry_time;
    EXPECT_FALSE(port.serviceRxRecovery(false));
  }
  EXPECT_EQ(receive_calls, 8U);
  auv::peripheral::InsPortDiagnostics diagnostics;
  port.diagnostics(&diagnostics);
  EXPECT_EQ(diagnostics.rx_recovery_attempts, 7U);
  EXPECT_EQ(diagnostics.rx_recovery_failures, 7U);
  EXPECT_TRUE(diagnostics.rx_recovery_pending);
}

TEST_F(InsUartRecoveryTest, RejectsHalSuccessWhenDmaDidNotStart) {
  ASSERT_TRUE(port.init());
  port.onHalError(&uart);
  disableReceive(uart);
  activate_receive = false;

  EXPECT_FALSE(port.serviceRxRecovery(false));
  auv::peripheral::InsPortDiagnostics diagnostics;
  port.diagnostics(&diagnostics);
  EXPECT_EQ(diagnostics.rx_start_status, HAL_ERROR);
  EXPECT_EQ(diagnostics.rx_recovery_successes, 0U);
  EXPECT_EQ(diagnostics.rx_recovery_failures, 1U);
  EXPECT_TRUE(diagnostics.rx_recovery_pending);
}

TEST_F(InsUartRecoveryTest, ReArmsAnInactiveDmaAfterValidFrameTimeout) {
  ASSERT_TRUE(port.init());
  disableReceive(uart);
  EXPECT_FALSE(port.serviceRxRecovery(false));
  EXPECT_TRUE(port.serviceRxRecovery(true));
  EXPECT_EQ(receive_calls, 2U);
}
