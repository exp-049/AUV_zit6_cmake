#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>

namespace auv {
namespace peripheral {
struct UsblPortDiagnostics;
}
} // namespace auv

namespace auv {
namespace porting {

/**
 * @brief USART3 USBL RX circular DMA storage.
 *
 * The buffer is placed in RAM_D2 by the linker and is therefore accessible by
 * the DMA controller.  The normal application consumes it by polling the DMA
 * write position; no byte-by-byte interrupt processing is required.
 */
extern uint8_t usbl_rx_buffer[512];

/**
 * @class USBL_Porting
 * @brief USBL UART hardware adapter used by USBL_Driver in NORMAL.
 */
class USBL_Porting {
public:
  static constexpr uint16_t kBufferSize = 512;

  USBL_Porting(UART_HandleTypeDef *uart, uint8_t *buffer,
               uint16_t buffer_size);

  static bool initPort(void *ctx);
  static uint16_t readPort(void *ctx, uint8_t *buffer, uint16_t max_len);
  static void diagnosticsPort(void *ctx,
                              auv::peripheral::UsblPortDiagnostics *out);
  static uint32_t getTickPort(void *ctx);
  static void handleHalRxEvent(UART_HandleTypeDef *huart, uint16_t size);

  bool init();
  uint16_t read(uint8_t *buffer, uint16_t max_len);
  UART_HandleTypeDef *getUart() const { return uart_; }
  void diagnostics(auv::peripheral::UsblPortDiagnostics *out) const;
  uint32_t getTick() const;

  static USBL_Porting *active_instance;

  void onRxEvent(uint16_t size);

private:
  UART_HandleTypeDef *uart_;
  uint8_t *rx_buffer_;
  uint16_t buffer_size_;
  uint16_t read_pos_ = 0;
  volatile uint32_t events_ = 0;
  volatile uint32_t invalid_events_ = 0;
};

} // namespace porting
} // namespace auv
