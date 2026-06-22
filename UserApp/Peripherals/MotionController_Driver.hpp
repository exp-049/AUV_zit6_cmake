#ifndef MOTION_CONTROLLER_DRIVER_HPP
#define MOTION_CONTROLLER_DRIVER_HPP

#include "FreeRTOS.h"
#include "task.h"
#include <cstdint>
#include <cstring>

// SCB cache maintenance is provided by CMSIS headers (core_cm7)

namespace auv {
namespace peripheral {

// 0x01: 推力下发数据包
struct __attribute__((packed)) ThrustPacket {
  uint8_t head[2]; // FA AF
  uint8_t id;      // 0x01
  float Fx, Fy, Fz;
  float Fyaw, Fpitch, Froll;
  uint8_t tail[2]; // FB BF
};

// 0x07: 推力曲线配置包
struct __attribute__((packed)) CurvePacket {
  uint8_t head[2]; // FA AF
  uint8_t id;      // 0x07
  uint8_t mode;    // 0: Read, 1: Write
  uint8_t index;   // Motor Index
  float pwm[4];    // 4 PWM points
  float thrust[4]; // 4 Thrust points
  uint8_t tail[2]; // FB BF
};

// 0x02: 舵机控制包
struct __attribute__((packed)) ServoPacket {
  uint8_t head[2]; // FA AF
  uint8_t id;      // 0x02
  float angle;
  uint8_t tail[2]; // FB BF
};

// 0x03: 灯光控制包
struct __attribute__((packed)) LightPacket {
  uint8_t head[2]; // FA AF
  uint8_t id;      // 0x03
  uint8_t state;   // R/Y/B state
  uint8_t tail[2]; // FB BF
};

/**
 * @struct MotorPortOps
 * @brief 动力控制板硬件操作接口（函数指针表）
 */
struct MotorPortOps {
  void *ctx;        ///< Porting 实例上下文
  bool (*transmitDMA)(void *ctx, const uint8_t *data, uint16_t size);
  void *(*getTxPacket)(void *ctx);
};

/**
 * @class MotionController_Driver
 * @brief 动力控制板驱动类
 */
class MotionController_Driver {
public:
  MotionController_Driver(MotorPortOps ops, ThrustPacket *ext_pkt = nullptr);
  ~MotionController_Driver();

  // 公共接口
  void publishThrust(float fx, float fy, float fz, float fyaw, float fp = 0,
                     float fr = 0);
  void setThrustCurve(uint8_t mode, uint8_t index, const float pwm[4],
                      const float thrust[4]);
  void setServoAngle(float angle);
  void setLightState(uint8_t state);

private:
  template <typename T> void initPacket(T *pkt, uint8_t id) {
    pkt->head[0] = 0xFA;
    pkt->head[1] = 0xAF;
    pkt->id = id;
    pkt->tail[0] = 0xFB;
    pkt->tail[1] = 0xBF;
  }

  void sendThrustPacketDMA();
  void transmitDMA(uint8_t *data, uint16_t size);

  MotorPortOps ops_;         ///< 硬件操作接口
  ThrustPacket *thrust_pkt_ptr_;
  ThrustPacket internal_pkt_;
};

} // namespace peripheral
} // namespace auv

#endif // MOTION_CONTROLLER_DRIVER_HPP
