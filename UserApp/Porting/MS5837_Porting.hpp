#pragma once

#include "stm32h7xx_hal.h"
#include <stdint.h>

namespace auv {
namespace porting {

/**
 * @class MS5837_Porting
 * @brief MS5837 深度传感器硬件适配层（I2C1）
 *
 * 封装 I2C 读写和延时操作。
 * 替代原先直接在驱动中调用 HAL_I2C 的模式。
 */
class MS5837_Porting {
public:
  MS5837_Porting(I2C_HandleTypeDef *hi2c, uint8_t addr);

  /** @brief 供 DepthPortOps 使用的静态包装 */
  static bool writePort(void *ctx, uint8_t cmd);
  static bool readPortByte(void *ctx, uint8_t *data);
  static bool readPort(void *ctx, uint8_t *data, uint16_t size);
  static void delayPort(void *ctx, uint32_t ms);

  bool writeByte(uint8_t cmd);
  bool readByte(uint8_t *data);
  bool read(uint8_t *data, uint16_t size);
  void delay(uint32_t ms);

  uint32_t getTick() const;

private:
  I2C_HandleTypeDef *hi2c_;
  uint8_t addr_;
};

} // namespace porting
} // namespace auv
