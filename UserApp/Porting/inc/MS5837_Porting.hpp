#pragma once

#include "Depth_Sensor_Driver.hpp" // DepthPortOps
#include "stm32h7xx_hal.h"
#include <stdint.h>

// 前向声明（完整类型在 I2C_DepthBackend.hpp 中，由 AppContext.cpp 包含）
namespace auv {
namespace peripheral {
class I2C_DepthBackend;
}
} // namespace auv

namespace auv {
namespace porting {

/**
 * @class MS5837_Porting
 * @brief MS5837 深度传感器硬件适配层（I2C1）
 *
 * 封装 I2C 读写操作 + 创建 FreeRTOS 轮询任务。
 */
class MS5837_Porting {
public:
  MS5837_Porting(I2C_HandleTypeDef *hi2c, uint8_t addr,
                 auv::peripheral::I2C_DepthBackend *backend = nullptr);
  void setBackend(auv::peripheral::I2C_DepthBackend *backend) {
    backend_ = backend;
  }

  /** @brief 供 DepthPortOps 使用的静态包装 */
  static bool writePort(void *ctx, uint8_t cmd);
  static bool readPortByte(void *ctx, uint8_t *data);
  static bool readPort(void *ctx, uint8_t *data, uint16_t size);
  static void delayPort(void *ctx, uint32_t ms);
  static void startPort(void *ctx);

  /** @brief 创建 FreeRTOS 轮询任务 */
  void start();

  bool writeByte(uint8_t cmd);
  bool readByte(uint8_t *data);
  bool read(uint8_t *data, uint16_t size);
  void delay(uint32_t ms);

  uint32_t getTick() const;

private:
  I2C_HandleTypeDef *hi2c_;
  uint8_t addr_;
  auv::peripheral::I2C_DepthBackend *backend_;
};

} // namespace porting
} // namespace auv
