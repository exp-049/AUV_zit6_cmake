#pragma once

#include "UART_MS5837Backend.hpp"
#include "stm32h7xx_hal.h"

#include <stdint.h>

namespace auv {
namespace porting {

/** UART4 transport for the protocol-v1 MS5837 backend. */
class MS5837_UART_Porting {
public:
  explicit MS5837_UART_Porting(
      UART_HandleTypeDef *huart,
      auv::peripheral::UART_MS5837Backend *backend = nullptr);

  void setBackend(auv::peripheral::UART_MS5837Backend *backend) {
    backend_ = backend;
  }

  static bool transmitPort(void *ctx, const uint8_t *data, uint16_t length);
  static void pollPort(void *ctx);
  static bool startRxPort(void *ctx);
  static uint32_t getTickPort(void *ctx);
  static uint32_t getRxRecoveryCountPort(void *ctx);
  static uint32_t getRxErrorCountPort(void *ctx);
  static uint32_t getLastRxErrorPort(void *ctx);
  static uint32_t getLastRxRecoveryReasonPort(void *ctx);
  static uint32_t getRxEventCountPort(void *ctx);
  static uint32_t getDmaWritePosPort(void *ctx);
  static void handleHalRxEvent(UART_HandleTypeDef *huart, uint16_t size);
  static void handleHalError(UART_HandleTypeDef *huart);

  bool startRx();
  void poll();

  static constexpr uint16_t kDmaBufferSize = 256U;

private:
  bool startIdleDma();
  bool dmaIsActive() const;
  void recoverRx(uint32_t now_ms, uint32_t reason);
  void onRxEvent(uint16_t size);
  void processAvailable();

  static MS5837_UART_Porting *active_instance_;

  UART_HandleTypeDef *huart_;
  auv::peripheral::UART_MS5837Backend *backend_;
  uint8_t *dma_buffer_;
  uint16_t dma_pos_ = 0U;
  uint32_t last_progress_ms_ = 0U;
  uint32_t last_recovery_ms_ = 0U;
  uint32_t recovery_count_ = 0U;
  volatile uint32_t uart_error_count_ = 0U;
  volatile uint32_t last_uart_error_ = 0U;
  volatile uint32_t last_recovery_reason_ = 0U;
  volatile uint32_t rx_event_count_ = 0U;
  uint32_t last_polled_event_count_ = 0U;
  volatile uint16_t last_event_size_ = 0U;
  volatile bool rx_event_pending_ = false;

  static constexpr uint32_t kRecoveryIntervalMs = 100U;
};

} // namespace porting
} // namespace auv
