#include "Pushrod_GPIO_Porting.hpp"

namespace auv {
namespace porting {

namespace {
constexpr uint16_t kMotorPins = GPIO_PIN_7 | GPIO_PIN_8;
}

void Pushrod_GPIO_Porting::configurePins() {
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {};
  gpio.Pin = kMotorPins;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;

  /* De-energize before changing the pin mux/output mode. */
  HAL_GPIO_WritePin(GPIOB, kMotorPins, GPIO_PIN_RESET);
  HAL_GPIO_Init(GPIOB, &gpio);
}

bool Pushrod_GPIO_Porting::initPort(void *ctx) {
  (void)ctx;
  configurePins();
  return true;
}

bool Pushrod_GPIO_Porting::setOutputsPort(void *ctx, bool in1, bool in2) {
  (void)ctx;

  /*
   * PB8=IN1 and PB7=IN2. Reset both first so reversing direction never
   * briefly drives both bridge inputs high. Then assert only one input.
   */
  HAL_GPIO_WritePin(GPIOB, kMotorPins, GPIO_PIN_RESET);
  if (in1) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
  } else if (in2) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  }
  return true;
}

uint32_t Pushrod_GPIO_Porting::getTickPort(void *ctx) {
  (void)ctx;
  return HAL_GetTick();
}

} // namespace porting
} // namespace auv
