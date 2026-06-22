#pragma once

#include "stm32h7xx_hal.h"
#include <stdint.h>

namespace auv {
namespace porting {

class MotionController_Porting {
public:
  MotionController_Porting(UART_HandleTypeDef *huart);

  /** @brief 供 MotorPortOps 使用的静态包装 */
  static bool transmitDMA(void *ctx, const uint8_t *data, uint16_t size);
  static void *getTxPacket(void *ctx);

private:
  UART_HandleTypeDef *huart_;
  static __attribute__((section(".dma_buffer"),
                        aligned(32))) uint8_t tx_packet_buf_[26];
};

} // namespace porting
} // namespace auv
