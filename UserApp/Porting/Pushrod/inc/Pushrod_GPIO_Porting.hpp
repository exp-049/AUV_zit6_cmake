#ifndef PUSHROD_GPIO_PORTING_HPP
#define PUSHROD_GPIO_PORTING_HPP

#include "Pushrod_GPIO_Backend.hpp"
#include "stm32h7xx_hal.h"

#include <stdint.h>

namespace auv {
namespace porting {

/** HAL adapter for the non-PWM PB8/PB7 motor-bridge backend. */
class Pushrod_GPIO_Porting {
public:
  static bool initPort(void *ctx);
  static bool setOutputsPort(void *ctx, bool in1, bool in2);
  static uint32_t getTickPort(void *ctx);

private:
  static void configurePins();
};

} // namespace porting
} // namespace auv

#endif
