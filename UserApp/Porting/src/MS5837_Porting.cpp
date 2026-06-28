#include "MS5837_Porting.hpp"
#include "cmsis_os2.h"

namespace auv {
namespace porting {

MS5837_Porting::MS5837_Porting(I2C_HandleTypeDef *hi2c, uint8_t addr)
    : hi2c_(hi2c), addr_(addr) {}

bool MS5837_Porting::writePort(void *ctx, uint8_t cmd) {
  return static_cast<MS5837_Porting *>(ctx)->writeByte(cmd);
}
bool MS5837_Porting::readPortByte(void *ctx, uint8_t *data) {
  return static_cast<MS5837_Porting *>(ctx)->readByte(data);
}
bool MS5837_Porting::readPort(void *ctx, uint8_t *data, uint16_t size) {
  return static_cast<MS5837_Porting *>(ctx)->read(data, size);
}
void MS5837_Porting::delayPort(void *ctx, uint32_t ms) {
  static_cast<MS5837_Porting *>(ctx)->delay(ms);
}

bool MS5837_Porting::writeByte(uint8_t cmd) {
  return HAL_I2C_Master_Transmit(hi2c_, addr_, &cmd, 1, 100) == HAL_OK;
}

bool MS5837_Porting::readByte(uint8_t *data) {
  return HAL_I2C_Master_Receive(hi2c_, addr_, data, 1, 100) == HAL_OK;
}

bool MS5837_Porting::read(uint8_t *data, uint16_t size) {
  return HAL_I2C_Master_Receive(hi2c_, addr_, data, size, 100) == HAL_OK;
}

void MS5837_Porting::delay(uint32_t ms) { osDelay(ms); }

uint32_t MS5837_Porting::getTick() const { return HAL_GetTick(); }

} // namespace porting
} // namespace auv
