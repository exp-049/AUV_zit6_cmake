#ifndef __MS5837_DRIVER_HPP
#define __MS5837_DRIVER_HPP

#include <math.h>
#include <stdint.h>

namespace auv {
namespace peripheral {

/** @brief 数据就绪回调类型 */
struct DepthDataReadyCallback {
  void (*onDepthReady)(void *ctx, float depth, float temperature);
  void *ctx;
};

/** @brief 深度传感器硬件操作接口（I2C 函数指针表） */
struct DepthPortOps {
  void *ctx;
  bool (*writeByte)(void *ctx, uint8_t cmd);
  bool (*readByte)(void *ctx, uint8_t *data);
  bool (*read)(void *ctx, uint8_t *data, uint16_t size);
  void (*delay)(void *ctx, uint32_t ms);
  void (*start)(void *ctx);
};

struct DepthBackend {
  virtual ~DepthBackend() = default;

  /** @brief 初始化传感器 */
  virtual bool init() = 0;

  /**
   * @brief 轮询数据源（DMA 双缓冲需要先 poll 再 read）
   * I2C 后端此方法为空操作
   */
  virtual void poll() = 0;

  /** @brief 读取数据，返回 true 表示有新数据就绪 */
  virtual bool read() = 0;

  /**
   * @brief 注册数据就绪回调
   * Init 内部会调用此方法将数据管道接至 MS5837_Driver::setMS5837Z()
   */
  virtual void setCallback(DepthDataReadyCallback cb) = 0;

  /**
   * @brief 启动后端工作
   * I2C: 创建 FreeRTOS 轮询任务
   * UART: 启动 DMA 接收
   * 在 Init() 之后调用
   */
  virtual void start() = 0;

  /** @brief 传感器是否已连接 */
  virtual bool isConnected() const = 0;

  /** @brief UART 协议握手是否收到 ACK；非 UART 后端默认不适用 */
  virtual bool isHandshakeAcknowledged() const { return false; }

  /** @brief UART 接收层自恢复次数；非 UART 后端默认返回 0 */
  virtual uint32_t getRxRecoveryCount() const { return 0U; }
  virtual uint32_t getRxErrorCount() const { return 0U; }
  virtual uint32_t getLastRxError() const { return 0U; }
  virtual uint32_t getLastRxRecoveryReason() const { return 0U; }
  virtual uint32_t getRxEventCount() const { return 0U; }
  virtual uint32_t getDmaWritePos() const { return 0U; }

  /** @brief 获取最新深度值（米） */
  virtual float getDepth() const = 0;

  /** @brief 获取最新温度值（摄氏度） */
  virtual float getTemperature() const = 0;
};

/**
 * @class MS5837_Driver
 * @brief 深度传感器驱动（Facade）
 *
 * 对外提供稳定的公开接口（getMS5837Z/setMS5837Z/Depth/Init/Read），
 * 内部通过 DepthBackend* 委托给具体硬件实现（I2C / UART）。
 *
 * 下游（ControlTask 等）通过此接口访问深度，不受后端切换影响。
 */
class MS5837_Driver {
public:
  MS5837_Driver(DepthBackend *backend);
  ~MS5837_Driver();

  /** @name 初始化与读取*/
  /**@{*/
  void Init(void);
  void start();
  int Read();
  /**@}*/

  /** @name 深度数据访问（数据容器） */
  /**@{*/
  void Depth(float *p);
  float getMS5837Z();
  void setMS5837Z(float z);
  inline void altitude(float *p);
  /**@}*/

  /** @brief 传感器连接状态（由 Backend 更新） */
  bool is_connected = false;

  /** @brief UART 协议握手 ACK 状态（非 UART 后端返回 false） */
  bool isHandshakeAcknowledged() const;

  /** @brief UART 接收层自恢复次数（非 UART 后端返回 0） */
  uint32_t getRxRecoveryCount() const;
  uint32_t getRxErrorCount() const;
  uint32_t getLastRxError() const;
  uint32_t getLastRxRecoveryReason() const;
  uint32_t getRxEventCount() const;
  uint32_t getDmaWritePos() const;

  /** @brief 最近一次压力值（mBar），供 Depth() fallback 使用 */
  float pressure = 0;

  /** @brief 最近一次温度值（摄氏度） */
  float temperture = 0;

private:
  DepthBackend *backend_;

  // 深度数据容器（临界区保护）
  float last_valid_depth = 0.0f;
  bool has_valid_depth = false;
};

} // namespace peripheral
} // namespace auv

#endif

// // 1. 触发读取并进行内部计算 (calculate)
// depthSensor.Read();

// // 2. 获取压力和温度
// float currentP = depthSensor.pressure;    // 单位：mbar 或 Pa
// (取决于你的宏定义) float currentT = depthSensor.temperture;  // 单位：摄氏度

// // 3. 计算深度
// float currentDepth = 0;
// depthSensor.Depth(&currentDepth); // 结果存入 currentDepth，单位：米

// // 打印数据调试
// printf("Depth: %.2f m, Temp: %.2f C\r\n", currentDepth, currentT);

// Delay(100); // 控制采样频率
