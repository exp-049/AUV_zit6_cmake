// IIC直连方案,载板上不稳定,已弃用
#include "I2C_DepthBackend.hpp"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "main.h" // HAL_GetTick
#include "task.h" // vTaskDelayUntil, xTaskGetTickCount

namespace auv {
namespace peripheral {

I2C_DepthBackend::I2C_DepthBackend(DepthPortOps ops, uint8_t slave_addr)
    : ops_(ops), slave_address_(slave_addr), model_(MS5837_30BA),
      fluidDensity_(1029) {}

// 轮询任务入口（FreeRTOS 任务函数）
// 注意：不在此处调用 init()，init() 已在 Depth_Sensor_Driver::Init() 中完成
void I2C_DepthBackend::pollingTask(void *arg) {
  auto *self = static_cast<I2C_DepthBackend *>(arg);

  TickType_t last_wake = xTaskGetTickCount();

  for (;;) {
    if (self->read()) {
      // 新数据就绪，触发回调
      if (self->cb_.onDepthReady) {
        self->cb_.onDepthReady(self->cb_.ctx, self->depth_, self->temperature_);
      }
    }
    // ~125Hz 循环；两步转换产生 ~62.5Hz 采样率
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(8));
  }
}

// Backend 接口实现

bool I2C_DepthBackend::init() {
  uint8_t reset_cmd = MS5837_RESET;
  if (!transmitByte(&reset_cmd))
    return false;
  osDelay(10);

  for (uint8_t i = 0; i < 7; i++) {
    uint16_t c_val = 0;
    if (read16(MS5837_PROM_READ + (i * 2), c_val)) {
      m_MS5837_values_.C[i] = c_val;
    } else {
      connected_ = false;
      return false;
    }
    osDelay(20);
  }

  uint8_t crcRead = m_MS5837_values_.C[0] >> 12;
  uint8_t crcCalculated = crc4(m_MS5837_values_.C);

  connected_ = (crcCalculated == crcRead);
  return connected_;
}

bool I2C_DepthBackend::read() {
  // Non-blocking state machine: call frequently (e.g., 60Hz) until returns true
  uint32_t now = HAL_GetTick();

  switch (conv_state_) {
  case CS_IDLE: {
    uint8_t cmd = MS5837_CONVERT_D1_8192;
    if (!transmitByte(&cmd))
      return false;
    conv_start_ms_ = now;
    conv_state_ = CS_WAIT_D1;
    return false;
  }

  case CS_WAIT_D1: {
    if ((uint32_t)(now - conv_start_ms_) < conv_delay_ms_)
      return false;
    uint32_t d1_raw = 0;
    if (!read32(MS5837_ADC_READ, d1_raw)) {
      conv_state_ = CS_IDLE;
      return false;
    }
    m_MS5837_values_.D1 = d1_raw >> 8;

    uint8_t cmd = MS5837_CONVERT_D2_8192;
    if (!transmitByte(&cmd)) {
      conv_state_ = CS_IDLE;
      return false;
    }
    conv_start_ms_ = now;
    conv_state_ = CS_WAIT_D2;
    return false;
  }

  case CS_WAIT_D2: {
    if ((uint32_t)(now - conv_start_ms_) < conv_delay_ms_)
      return false;
    uint32_t d2_raw = 0;
    if (!read32(MS5837_ADC_READ, d2_raw)) {
      conv_state_ = CS_IDLE;
      return false;
    }
    m_MS5837_values_.D2 = d2_raw >> 8;

    // We have both D1 and D2 -> compute
    calculate();

    // Start next D1 conversion to pipeline continuous sampling
    uint8_t cmd = MS5837_CONVERT_D1_8192;
    if (!transmitByte(&cmd)) {
      conv_state_ = CS_IDLE;
      return true; // data available but couldn't start next conversion
    }
    conv_start_ms_ = now;
    conv_state_ = CS_WAIT_D1;
    return true; // new sample ready
  }
  }
  return false;
}

// ============================================================================
// I2C 原始读写
// ============================================================================

bool I2C_DepthBackend::transmitByte(uint8_t *pData) {
  return ops_.writeByte && ops_.writeByte(ops_.ctx, *pData);
}

bool I2C_DepthBackend::receiveByte(uint8_t *pData) {
  return ops_.readByte && ops_.readByte(ops_.ctx, pData);
}

bool I2C_DepthBackend::receive(uint8_t *pData, uint16_t Size) {
  return ops_.read && ops_.read(ops_.ctx, pData, Size);
}

inline int8_t I2C_DepthBackend::read8(uint8_t addr) {
  uint8_t data = 0;
  transmitByte(&addr);
  receiveByte(&data);
  return data;
}

inline bool I2C_DepthBackend::read16(uint8_t addr, uint16_t &out_data) {
  uint8_t dataArr[2] = {0, 0};
  if (!transmitByte(&addr))
    return false;
  if (!receive(dataArr, 2))
    return false;
  out_data = (dataArr[0] << 8) | dataArr[1];
  return true;
}

inline bool I2C_DepthBackend::read32(uint8_t addr, uint32_t &out_data) {
  uint8_t dataArr[4] = {0, 0, 0, 0};
  if (!transmitByte(&addr))
    return false;
  if (!receive(dataArr, 4))
    return false;
  out_data =
      (dataArr[0] << 24) | (dataArr[1] << 16) | (dataArr[2] << 8) | dataArr[3];
  return true;
}

// ============================================================================
// MS5837 协议：CRC4 校验
// ============================================================================

uint8_t I2C_DepthBackend::crc4(uint16_t n_prom[]) {
  uint16_t n_rem = 0;

  n_prom[0] = ((n_prom[0]) & 0x0FFF);
  n_prom[7] = 0;

  for (uint8_t i = 0; i < 16; i++) {
    if (i % 2 == 1) {
      n_rem ^= (uint16_t)((n_prom[i >> 1]) & 0x00FF);
    } else {
      n_rem ^= (uint16_t)(n_prom[i >> 1] >> 8);
    }
    for (uint8_t n_bit = 8; n_bit > 0; n_bit--) {
      if (n_rem & 0x8000) {
        n_rem = (n_rem << 1) ^ 0x3000;
      } else {
        n_rem = (n_rem << 1);
      }
    }
  }

  n_rem = ((n_rem >> 12) & 0x000F);
  return n_rem ^ 0x00;
}

// ============================================================================
// MS5837 协议：温度补偿计算
// ============================================================================

void I2C_DepthBackend::calculate() {
  int32_t dT = 0;
  int64_t SENS = 0;
  int64_t OFF = 0;
  int32_t SENSi = 0;
  int32_t OFFi = 0;
  int32_t Ti = 0;
  int64_t OFF2 = 0;
  int64_t SENS2 = 0;

  dT = m_MS5837_values_.D2 - (uint32_t)m_MS5837_values_.C[5] * 256l;

  if (model_) {
    SENS = (int64_t)m_MS5837_values_.C[1] * 65536l +
           ((int64_t)m_MS5837_values_.C[3] * dT) / 128l;
    OFF = (int64_t)m_MS5837_values_.C[2] * 131072l +
          ((int64_t)m_MS5837_values_.C[4] * dT) / 64l;
    m_MS5837_values_.P =
        (m_MS5837_values_.D1 * SENS / (2097152l) - OFF) / (32768l);
  } else {
    SENS = (int64_t)m_MS5837_values_.C[1] * 32768l +
           ((int64_t)m_MS5837_values_.C[3] * dT) / 256l;
    OFF = (int64_t)m_MS5837_values_.C[2] * 65536l +
          ((int64_t)m_MS5837_values_.C[4] * dT) / 128l;
    m_MS5837_values_.P =
        (m_MS5837_values_.D1 * SENS / (2097152l) - OFF) / (8192l);
  }

  m_MS5837_values_.TEMP =
      2000l + (int64_t)dT * m_MS5837_values_.C[6] / 8388608LL;

  if (model_) {
    if ((m_MS5837_values_.TEMP / 100) < 20) {
      Ti = (11 * (int64_t)dT * (int64_t)dT) / (34359738368LL);
      OFFi = (31 * (m_MS5837_values_.TEMP - 2000) *
              (m_MS5837_values_.TEMP - 2000)) /
             8;
      SENSi = (63 * (m_MS5837_values_.TEMP - 2000) *
               (m_MS5837_values_.TEMP - 2000)) /
              32;
    }
  } else {
    if ((m_MS5837_values_.TEMP / 100) < 20) {
      Ti = (3 * (int64_t)dT * (int64_t)dT) / (8589934592LL);
      OFFi = (3 * (m_MS5837_values_.TEMP - 2000) *
              (m_MS5837_values_.TEMP - 2000)) /
             2;
      SENSi = (5 * (m_MS5837_values_.TEMP - 2000) *
               (m_MS5837_values_.TEMP - 2000)) /
              8;
      if ((m_MS5837_values_.TEMP / 100) < -15) {
        OFFi = OFFi + 7 * (m_MS5837_values_.TEMP + 1500l) *
                          (m_MS5837_values_.TEMP + 1500l);
        SENSi = SENSi + 4 * (m_MS5837_values_.TEMP + 1500l) *
                            (m_MS5837_values_.TEMP + 1500l);
      }
    } else if ((m_MS5837_values_.TEMP / 100) >= 20) {
      Ti = 2 * (dT * dT) / (137438953472LL);
      OFFi = (1 * (m_MS5837_values_.TEMP - 2000) *
              (m_MS5837_values_.TEMP - 2000)) /
             16;
      SENSi = 0;
    }
  }

  OFF2 = OFF - OFFi;
  SENS2 = SENS - SENSi;

  if (model_) {
    m_MS5837_values_.TEMP = (m_MS5837_values_.TEMP - Ti);
    m_MS5837_values_.P =
        (((m_MS5837_values_.D1 * SENS2) / 2097152l - OFF2) / 32768l) / 100;
  } else {
    m_MS5837_values_.TEMP = (m_MS5837_values_.TEMP - Ti);
    m_MS5837_values_.P =
        (((m_MS5837_values_.D1 * SENS2) / 2097152l - OFF2) / 8192l) / 10;
  }

  temperature_ = m_MS5837_values_.TEMP / 100.0f;

  float raw_pressure = m_MS5837_values_.P * 1.0f;
#ifdef Pa
  raw_pressure = m_MS5837_values_.P * 100.0f;
#endif
#ifdef bar
  raw_pressure = m_MS5837_values_.P * 0.001f;
#endif

  // 计算深度并更新
  float computed_depth =
      (raw_pressure * 100.0f - 101300.0f) / (fluidDensity_ * 9.80665f);
  if (raw_pressure > 0.0f && computed_depth > -5.0f && computed_depth < 50.0f) {
    depth_ = computed_depth;
  }
}

void I2C_DepthBackend::start() {
  // 委托 Porting 层创建 FreeRTOS 轮询任务
  if (ops_.start) {
    ops_.start(ops_.ctx);
  }
}

} // namespace peripheral
} // namespace auv
