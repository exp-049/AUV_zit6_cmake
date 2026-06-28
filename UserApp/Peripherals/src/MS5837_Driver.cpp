#include "MS5837_Driver.hpp"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"

namespace auv {
namespace peripheral {

bool MS5837_Driver::transmitByte(uint8_t *pData) {
  return ops_.writeByte && ops_.writeByte(ops_.ctx, *pData);
}

bool MS5837_Driver::receiveByte(uint8_t *pData) {
  return ops_.readByte && ops_.readByte(ops_.ctx, pData);
}

bool MS5837_Driver::receive(uint8_t *pData, uint16_t Size) {
  return ops_.read && ops_.read(ops_.ctx, pData, Size);
}

void MS5837_Driver::Init(void) {
  uint8_t reset_cmd = MS5837_RESET;

  transmitByte(&reset_cmd);
  osDelay(10);

  for (uint8_t i = 0; i < 7; i++) {
    uint16_t c_val = 0;
    if (read16(MS5837_PROM_READ + (i * 2), c_val)) {
      m_MS5837_values.C[i] = c_val;
    } else {
      is_connected = false;
      return;
    }
    osDelay(20);
  }
  uint8_t crcRead = m_MS5837_values.C[0] >> 12;
  uint8_t crcCalculated = crc4(m_MS5837_values.C);

  if (crcCalculated != crcRead) {
    is_connected = false;
  } else {
    is_connected = true;
  }
}

inline int8_t MS5837_Driver::read8(uint8_t addr) {
  uint8_t data = 0;
  transmitByte(&addr);
  receiveByte(&data);
  return data;
}

inline bool MS5837_Driver::read16(uint8_t addr, uint16_t &out_data) {
  uint8_t dataArr[2] = {0, 0};
  if (!transmitByte(&addr))
    return false;
  if (!receive(dataArr, 2))
    return false;
  out_data = (dataArr[0] << 8) | dataArr[1];
  return true;
}

inline bool MS5837_Driver::read32(uint8_t addr, uint32_t &out_data) {
  uint8_t dataArr[4] = {0, 0, 0, 0};
  if (!transmitByte(&addr))
    return false;
  if (!receive(dataArr, 4))
    return false;
  out_data =
      (dataArr[0] << 24) | (dataArr[1] << 16) | (dataArr[2] << 8) | dataArr[3];
  return true;
}

uint8_t MS5837_Driver::crc4(uint16_t n_prom[]) {
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

void MS5837_Driver::calculate() {

  int32_t dT = 0;
  int64_t SENS = 0;
  int64_t OFF = 0;
  int32_t SENSi = 0;
  int32_t OFFi = 0;
  int32_t Ti = 0;
  int64_t OFF2 = 0;
  int64_t SENS2 = 0;

  dT = m_MS5837_values.D2 - (uint32_t)m_MS5837_values.C[5] * 256l;
  if (model) {
    SENS = (int64_t)m_MS5837_values.C[1] * 65536l +
           ((int64_t)m_MS5837_values.C[3] * dT) / 128l;
    OFF = (int64_t)m_MS5837_values.C[2] * 131072l +
          ((int64_t)m_MS5837_values.C[4] * dT) / 64l;
    m_MS5837_values.P =
        (m_MS5837_values.D1 * SENS / (2097152l) - OFF) / (32768l);
  } else {
    SENS = (int64_t)m_MS5837_values.C[1] * 32768l +
           ((int64_t)m_MS5837_values.C[3] * dT) / 256l;
    OFF = (int64_t)m_MS5837_values.C[2] * 65536l +
          ((int64_t)m_MS5837_values.C[4] * dT) / 128l;
    m_MS5837_values.P =
        (m_MS5837_values.D1 * SENS / (2097152l) - OFF) / (8192l);
  }

  m_MS5837_values.TEMP = 2000l + (int64_t)dT * m_MS5837_values.C[6] / 8388608LL;

  if (model) {
    if ((m_MS5837_values.TEMP / 100) < 20) {
      Ti = (11 * (int64_t)dT * (int64_t)dT) / (34359738368LL);
      OFFi =
          (31 * (m_MS5837_values.TEMP - 2000) * (m_MS5837_values.TEMP - 2000)) /
          8;
      SENSi =
          (63 * (m_MS5837_values.TEMP - 2000) * (m_MS5837_values.TEMP - 2000)) /
          32;
    }
  } else {
    if ((m_MS5837_values.TEMP / 100) < 20) {
      Ti = (3 * (int64_t)dT * (int64_t)dT) / (8589934592LL);
      OFFi =
          (3 * (m_MS5837_values.TEMP - 2000) * (m_MS5837_values.TEMP - 2000)) /
          2;
      SENSi =
          (5 * (m_MS5837_values.TEMP - 2000) * (m_MS5837_values.TEMP - 2000)) /
          8;
      if ((m_MS5837_values.TEMP / 100) < -15) {
        OFFi = OFFi + 7 * (m_MS5837_values.TEMP + 1500l) *
                          (m_MS5837_values.TEMP + 1500l);
        SENSi = SENSi + 4 * (m_MS5837_values.TEMP + 1500l) *
                            (m_MS5837_values.TEMP + 1500l);
      }
    } else if ((m_MS5837_values.TEMP / 100) >= 20) {
      Ti = 2 * (dT * dT) / (137438953472LL);
      OFFi =
          (1 * (m_MS5837_values.TEMP - 2000) * (m_MS5837_values.TEMP - 2000)) /
          16;
      SENSi = 0;
    }
  }

  OFF2 = OFF - OFFi;
  SENS2 = SENS - SENSi;

  if (model) {
    m_MS5837_values.TEMP = (m_MS5837_values.TEMP - Ti);
    m_MS5837_values.P =
        (((m_MS5837_values.D1 * SENS2) / 2097152l - OFF2) / 32768l) / 100;
  } else {
    m_MS5837_values.TEMP = (m_MS5837_values.TEMP - Ti);
    m_MS5837_values.P =
        (((m_MS5837_values.D1 * SENS2) / 2097152l - OFF2) / 8192l) / 10;
  }

  temperture = m_MS5837_values.TEMP / 100.0f;
  pressure = m_MS5837_values.P * 1.0f;
#ifdef Pa
  pressure = m_MS5837_values.P * 100.0f;
#endif
#ifdef bar
  pressure = m_MS5837_values.P * 0.001f;
#endif
  // Update last valid depth only when reading is reasonable to avoid transient
  // zeros
  {
    float computed_depth =
        (pressure * 100.0f - 101300.0f) / (fluidDensity * 9.80665f);
    bool valid = true;
    if (!(pressure > 0.0f))
      valid = false;
    if (!(computed_depth > -5.0f && computed_depth < 50.0f))
      valid = false;
    if (valid) {
      taskENTER_CRITICAL();
      last_valid_pressure = pressure;
      last_valid_depth = computed_depth;
      has_valid_depth = true;
      taskEXIT_CRITICAL();
    }
  }
}

int MS5837_Driver::Read() {
  // Non-blocking state machine: call frequently (e.g., 60Hz) until it returns 1
  // (new sample)
  uint32_t now = HAL_GetTick();
  switch (conv_state) {
  case CS_IDLE: {
    uint8_t cmd = MS5837_CONVERT_D1_8192;
    if (!transmitByte(&cmd)) {
      return -1; // I2C transmit error
    }
    conv_start_ms = now;
    conv_state = CS_WAIT_D1;
    return 0; // conversion started
  }
  case CS_WAIT_D1: {
    if ((uint32_t)(now - conv_start_ms) < conv_delay_ms)
      return 0;
    uint32_t d1_raw = 0;
    if (!read32(MS5837_ADC_READ, d1_raw)) {
      conv_state = CS_IDLE;
      return -1;
    }
    m_MS5837_values.D1 = d1_raw >> 8;

    uint8_t cmd = MS5837_CONVERT_D2_8192;
    if (!transmitByte(&cmd)) {
      conv_state = CS_IDLE;
      return -1;
    }
    conv_start_ms = now;
    conv_state = CS_WAIT_D2;
    return 0;
  }
  case CS_WAIT_D2: {
    if ((uint32_t)(now - conv_start_ms) < conv_delay_ms)
      return 0;
    uint32_t d2_raw = 0;
    if (!read32(MS5837_ADC_READ, d2_raw)) {
      conv_state = CS_IDLE;
      return -1;
    }
    m_MS5837_values.D2 = d2_raw >> 8;

    // We have both D1 and D2 -> compute
    calculate();

    // Start next D1 conversion to pipeline continuous sampling
    uint8_t cmd = MS5837_CONVERT_D1_8192;
    if (!transmitByte(&cmd)) {
      conv_state = CS_IDLE;
      return 1; // sample available but couldn't start next, still report
                // success
    }
    conv_start_ms = now;
    conv_state = CS_WAIT_D1;
    return 1; // new sample ready
  }
  }
  return 0;
}

void MS5837_Driver::Depth(float *p) {
  if (has_valid_depth) {
    taskENTER_CRITICAL();
    *p = last_valid_depth;
    taskEXIT_CRITICAL();
  } else {
    *p = (pressure * 100.0f - 101300.0f) / (fluidDensity * 9.80665f);
  }
}

inline void MS5837_Driver::altitude(float *p) {
  *p = (1 - pow((pressure / 1013.25), .190284)) * 145366.45 * .3048;
}

MS5837_Driver::MS5837_Driver(DepthPortOps ops, uint8_t SLAVE_ADDRESS,
                             uint16_t MemAddSize)
    : ops_(ops), slave_address(SLAVE_ADDRESS), model(MS5837_30BA),
      fluidDensity(1029) {
  last_valid_pressure = 0.0f;
  last_valid_depth = 0.0f;
  has_valid_depth = false;
}

MS5837_Driver::~MS5837_Driver() {}

float MS5837_Driver::getMS5837Z() {
  float z = 0.0f;
  Depth(&z);
  return z;
}

void MS5837_Driver::setMS5837Z(float z) {
  taskENTER_CRITICAL();
  last_valid_depth = z;
  has_valid_depth = true;
  taskEXIT_CRITICAL();
}

} // namespace peripheral
} // namespace auv
