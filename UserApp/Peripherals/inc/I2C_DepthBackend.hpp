#pragma once

#include "MS5837_Driver.hpp" // DepthBackend, DepthPortOps
#include <stdint.h>

#define MS5837_RESET 0x1E
#define MS5837_ADC_READ 0x00
#define MS5837_PROM_READ 0xA0
#define MS5837_CONVERT_D1_8192 0x4A
#define MS5837_CONVERT_D2_8192 0x5A

// Models:
#define MS5837_30BA 0x00
#define MS5837_02BA 0x01

#define waterDensity 1029

namespace auv {
namespace peripheral {

// Values read from MS5837
typedef struct {
  int32_t TEMP;
  int32_t P;
  uint16_t C[8];
  uint32_t D1;
  uint32_t D2;
} MS5837_values;

/**
 * @class I2C_DepthBackend
 * @brief I2C MS5837 深度传感器后端实现
 *
 * 实现 DepthBackend 接口，封装 MS5837 芯片的完整 I2C 协议：
 * - Reset + PROM 读取 + CRC 校验
 * - D1/D2 非阻塞转换状态机
 * - 温度补偿计算（calculate）
 */
class I2C_DepthBackend : public DepthBackend {
public:
  I2C_DepthBackend(DepthPortOps ops, uint8_t slave_addr);

  /** @name Backend 接口方法 */
  /**@{*/
  bool init();
  void poll() {} // I2C 后端不需要 DMA poll，空操作
  bool read();
  void setCallback(DepthDataReadyCallback cb) { cb_ = cb; }
  void start();
  bool isConnected() const { return connected_; }
  float getDepth() const { return depth_; }
  float getTemperature() const { return temperature_; }
  /**@}*/

  /** @brief 设置转换等待时间（ms），默认 10ms */
  void setConversionDelay(uint16_t ms) { conv_delay_ms_ = ms; }

  /**
   * @brief 轮询任务入口（可用作 FreeRTOS osThreadNew 的入口函数）
   * @param arg 指向 I2C_DepthBackend 实例的指针
   *
   * 调用流程：
   *   1. 调用 init()
   *   2. 循环：read() → 若返回 true → 触发 DataReadyCallback
   *   3. 每次循环后 vTaskDelayUntil 控制频率
   */
  static void pollingTask(void *arg);

private:
  // I2C 原始读写
  bool transmitByte(uint8_t *pData);
  bool receiveByte(uint8_t *pData);
  bool receive(uint8_t *pData, uint16_t Size);

  inline int8_t read8(uint8_t addr);
  inline bool read16(uint8_t addr, uint16_t &out_data);
  inline bool read32(uint8_t addr, uint32_t &out_data);

  // MS5837 协议
  uint8_t crc4(uint16_t n_prom[]);
  void calculate();

  // 硬件操作
  DepthPortOps ops_;
  uint8_t slave_address_;
  uint8_t model_;
  float fluidDensity_;

  // 原始传感器数据
  MS5837_values m_MS5837_values_ = {};

  // 非阻塞状态机
  enum ConvState : uint8_t { CS_IDLE = 0, CS_WAIT_D1 = 1, CS_WAIT_D2 = 2 };
  ConvState conv_state_ = CS_IDLE;
  uint32_t conv_start_ms_ = 0;
  uint16_t conv_delay_ms_ = 10;

  // 数据就绪回调
  DepthDataReadyCallback cb_ = {nullptr, nullptr};

  // 输出数据
  float depth_ = 0.0f;
  float temperature_ = 0.0f;
  bool connected_ = false;
};

} // namespace peripheral
} // namespace auv
