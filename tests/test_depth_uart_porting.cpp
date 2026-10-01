#include "DepthCalcBoard_Porting.hpp"

#include <gtest/gtest.h>
#include <vector>

namespace {
HAL_StatusTypeDef receive_status = HAL_OK;
HAL_StatusTypeDef abort_status = HAL_OK;
bool activate_receive = true;
uint32_t tick_ms = 0U;
uint32_t receive_calls = 0U;
uint32_t abort_calls = 0U;
uint8_t *receive_buffer = nullptr;
uint16_t receive_size = 0U;

void disableReceive(UART_HandleTypeDef &uart) {
  uart.Instance->CR3 &= ~USART_CR3_DMAR;
  uart.hdmarx->Instance->CR &= ~DMA_SxCR_EN;
  uart.RxState = HAL_UART_STATE_READY;
}

class ByteSink : public auv::peripheral::DepthUartRxSink {
public:
  void onRxByte(uint8_t byte) override { bytes.push_back(byte); }
  std::vector<uint8_t> bytes;
};

class DepthUartPortTest : public ::testing::Test {
protected:
  void SetUp() override {
    receive_status = HAL_OK;
    abort_status = HAL_OK;
    activate_receive = true;
    tick_ms = 0U;
    receive_calls = 0U;
    abort_calls = 0U;
    receive_buffer = nullptr;
    receive_size = 0U;
  }
  void TearDown() override {
    auv::porting::DepthCalcBoard_Porting::active_instance = nullptr;
  }
  USART_TypeDef registers;
  DMA_Stream_TypeDef stream;
  DMA_HandleTypeDef dma{&stream};
  UART_HandleTypeDef uart{&registers, &dma, 0U};
  ByteSink sink;
};
} // namespace

uint32_t HAL_GetTick() { return tick_ms; }

HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart) {
  ++abort_calls;
  if (abort_status == HAL_OK) {
    if ((uart->Instance->CR3 & USART_CR3_DMAR) != 0U) {
      disableReceive(*uart);
      uart->hdmarx->State = 1U;
    } else {
      uart->RxState = HAL_UART_STATE_READY;
    }
  }
  return abort_status;
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
  if (receive_status == HAL_OK) {
    if (uart->hdmarx->State != 1U) {
      uart->hdmarx->ErrorCode = 0x10U;
      return HAL_ERROR;
    }
    receive_buffer = buffer;
    receive_size = size;
    uart->hdmarx->State = 2U;
    uart->RxState = HAL_UART_STATE_BUSY_RX;
    if (activate_receive) {
      uart->Instance->CR3 |= USART_CR3_DMAR;
      uart->hdmarx->Instance->CR |= DMA_SxCR_EN;
      uart->hdmarx->Instance->NDTR = size;
    }
  }
  return receive_status;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t,
                                    uint32_t) {
  return HAL_OK;
}

TEST_F(DepthUartPortTest, ReportsReceiveStartupAndConsumesEachByteOnce) {
  auv::porting::DepthCalcBoard_Porting port(&uart, &sink);
  ASSERT_TRUE(port.startRx());
  ASSERT_EQ(receive_size, 256U);
  for (uint8_t i = 0; i < 3; ++i) {
    receive_buffer[i] = i + 1;
  }
  stream.NDTR = 253U;
  port.poll();
  port.poll();
  EXPECT_EQ(sink.bytes, (std::vector<uint8_t>{1, 2, 3}));
  auv::peripheral::DepthDiagnostics diagnostics;
  port.getDiagnostics(diagnostics);
  EXPECT_TRUE(diagnostics.has_uart_diagnostics);
  EXPECT_TRUE(diagnostics.rx_dma_active);
  EXPECT_EQ(diagnostics.rx_start_status, HAL_OK);
  EXPECT_EQ(diagnostics.dma_write_pos, 3U);
}

TEST_F(DepthUartPortTest, ConsumesTailWhenDmaCounterIsZeroThenContinues) {
  auv::porting::DepthCalcBoard_Porting port(&uart, &sink);
  ASSERT_TRUE(port.startRx());
  for (uint16_t i = 0; i < receive_size; ++i) {
    receive_buffer[i] = static_cast<uint8_t>(i);
  }
  stream.NDTR = 6U;
  port.poll();
  ASSERT_EQ(sink.bytes.size(), 250U);
  stream.NDTR = 0U;
  port.poll(); // Must terminate even before the circular counter reloads.
  ASSERT_EQ(sink.bytes.size(), 256U);
  EXPECT_EQ(sink.bytes.back(), 255U);
  stream.NDTR = 256U;
  port.poll();
  EXPECT_EQ(sink.bytes.size(), 256U);
  receive_buffer[0] = 42U;
  stream.NDTR = 255U;
  port.poll();
  ASSERT_EQ(sink.bytes.size(), 257U);
  EXPECT_EQ(sink.bytes.back(), 42U);
}

TEST_F(DepthUartPortTest, ReportsReceiveFailureWithoutConsumingBuffer) {
  auv::porting::DepthCalcBoard_Porting port(&uart, &sink);
  receive_status = HAL_BUSY;
  EXPECT_FALSE(port.startRx());
  port.poll();
  EXPECT_TRUE(sink.bytes.empty());
  auv::peripheral::DepthDiagnostics diagnostics;
  port.getDiagnostics(diagnostics);
  EXPECT_EQ(diagnostics.rx_start_status, HAL_BUSY);
  EXPECT_FALSE(diagnostics.rx_dma_active);
}

TEST_F(DepthUartPortTest, RecordsErrorsOnlyForItsOwnUart) {
  auv::porting::DepthCalcBoard_Porting port(&uart, &sink);
  ASSERT_TRUE(port.startRx());
  UART_HandleTypeDef other;
  other.ErrorCode = 4U;
  auv::porting::DepthCalcBoard_Porting::handleHalError(&other);
  uart.ErrorCode = 8U;
  auv::porting::DepthCalcBoard_Porting::handleHalError(&uart);
  uart.ErrorCode = 0U; // HAL may clear the error after the interrupt.
  auv::peripheral::DepthDiagnostics diagnostics;
  port.getDiagnostics(diagnostics);
  EXPECT_EQ(diagnostics.rx_error_count, 1U);
  EXPECT_EQ(diagnostics.last_rx_error, 8U);
}

TEST_F(DepthUartPortTest, KeepsHealthyDmaRunningDuringBriefSignalGap) {
  auv::porting::DepthCalcBoard_Porting port(&uart, &sink);
  ASSERT_TRUE(port.startRx());
  EXPECT_FALSE(port.serviceRxRecovery(false));
  EXPECT_EQ(receive_calls, 1U);
  EXPECT_TRUE((registers.CR3 & USART_CR3_DMAR) != 0U);
}

TEST_F(DepthUartPortTest, RearmsAfterUartErrorInTaskContext) {
  auv::porting::DepthCalcBoard_Porting port(&uart, &sink);
  ASSERT_TRUE(port.startRx());
  uart.ErrorCode = 8U;
  auv::porting::DepthCalcBoard_Porting::handleHalError(&uart);
  disableReceive(uart); // HAL aborts DMA before invoking the error callback.

  EXPECT_TRUE(port.serviceRxRecovery(false));
  EXPECT_EQ(receive_calls, 2U);
  EXPECT_TRUE((registers.CR3 & USART_CR3_DMAR) != 0U);
  EXPECT_TRUE((stream.CR & DMA_SxCR_EN) != 0U);
  auv::peripheral::DepthDiagnostics diagnostics;
  port.getDiagnostics(diagnostics);
  EXPECT_EQ(diagnostics.rx_error_count, 1U);
  EXPECT_EQ(diagnostics.last_rx_error, 8U);
  EXPECT_EQ(diagnostics.rx_recovery_attempts, 1U);
  EXPECT_EQ(diagnostics.rx_recovery_successes, 1U);
  EXPECT_EQ(diagnostics.rx_recovery_failures, 0U);
  EXPECT_FALSE(diagnostics.rx_recovery_pending);
  EXPECT_EQ(diagnostics.rx_dma_deinit_status, HAL_OK);
  EXPECT_EQ(diagnostics.rx_dma_init_status, HAL_OK);
}

TEST_F(DepthUartPortTest, RearmsOnValidFrameTimeoutAndLimitsRepeats) {
  auv::porting::DepthCalcBoard_Porting port(&uart, &sink);
  ASSERT_TRUE(port.startRx());
  EXPECT_TRUE(port.serviceRxRecovery(true));
  EXPECT_EQ(receive_calls, 2U);
  EXPECT_FALSE(port.serviceRxRecovery(true));
  tick_ms = 4999U;
  EXPECT_FALSE(port.serviceRxRecovery(true));
  tick_ms = 5000U;
  EXPECT_TRUE(port.serviceRxRecovery(true));
  EXPECT_EQ(receive_calls, 3U);
}

TEST_F(DepthUartPortTest, RetriesFailedRecoveryWithIncreasingBackoff) {
  auv::porting::DepthCalcBoard_Porting port(&uart, &sink);
  ASSERT_TRUE(port.startRx());
  uart.ErrorCode = 8U;
  auv::porting::DepthCalcBoard_Porting::handleHalError(&uart);
  disableReceive(uart);
  receive_status = HAL_BUSY;

  EXPECT_FALSE(port.serviceRxRecovery(false));
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
  auv::peripheral::DepthDiagnostics diagnostics;
  port.getDiagnostics(diagnostics);
  EXPECT_EQ(diagnostics.rx_recovery_attempts, 7U);
  EXPECT_EQ(diagnostics.rx_recovery_failures, 7U);
  EXPECT_TRUE(diagnostics.rx_recovery_pending);
}

TEST_F(DepthUartPortTest, RejectsHalSuccessWhenDmaDidNotStart) {
  auv::porting::DepthCalcBoard_Porting port(&uart, &sink);
  ASSERT_TRUE(port.startRx());
  uart.ErrorCode = 8U;
  auv::porting::DepthCalcBoard_Porting::handleHalError(&uart);
  disableReceive(uart);
  activate_receive = false;

  EXPECT_FALSE(port.serviceRxRecovery(false));
  auv::peripheral::DepthDiagnostics diagnostics;
  port.getDiagnostics(diagnostics);
  EXPECT_EQ(diagnostics.rx_start_status, HAL_ERROR);
  EXPECT_EQ(diagnostics.rx_recovery_successes, 0U);
  EXPECT_EQ(diagnostics.rx_recovery_failures, 1U);
  EXPECT_TRUE(diagnostics.rx_recovery_pending);
}

TEST_F(DepthUartPortTest, HandlesMissingDmaWithoutDereferencingIt) {
  uart.hdmarx = nullptr;
  auv::porting::DepthCalcBoard_Porting port(&uart, &sink);
  EXPECT_FALSE(port.startRx());
  port.poll();
  auv::peripheral::DepthDiagnostics diagnostics;
  port.getDiagnostics(diagnostics);
  EXPECT_EQ(diagnostics.rx_start_status, HAL_ERROR);
  EXPECT_FALSE(diagnostics.rx_dma_active);
}
