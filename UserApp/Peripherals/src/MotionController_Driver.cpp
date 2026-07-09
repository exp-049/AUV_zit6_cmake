#include "../inc/MotionController_Driver.hpp"
#include "../HAL/Middlewares/Third_Party/FreeRTOS/Source/include/FreeRTOS.h"
#include "../HAL/Middlewares/Third_Party/FreeRTOS/Source/include/task.h"
#include "stm32h7xx_hal.h"
#include <cstring>

namespace auv {
namespace peripheral {

MotionController_Driver::MotionController_Driver(MotorPortOps ops,
                                                 ThrustPacket *ext_pkt)
    : ops_(ops), thrust_pkt_ptr_(ext_pkt ? ext_pkt
                                         : static_cast<ThrustPacket *>(
                                               ops_.getTxPacket(ops_.ctx))),
      internal_pkt_{} {
  if (!thrust_pkt_ptr_)
    thrust_pkt_ptr_ = &internal_pkt_;
  initPacket(thrust_pkt_ptr_, 0x01);
  thrust_pkt_ptr_->Fx = 0.0f;
  thrust_pkt_ptr_->Fy = 0.0f;
  thrust_pkt_ptr_->Fz = 0.0f;
  thrust_pkt_ptr_->Fyaw = 0.0f;
  thrust_pkt_ptr_->Fpitch = 0.0f;
  thrust_pkt_ptr_->Froll = 0.0f;
}

MotionController_Driver::~MotionController_Driver() = default;

bool MotionController_Driver::publishThrust(float fx, float fy, float fz,
                                            float fyaw, float fp, float fr) {
  taskENTER_CRITICAL();
  thrust_pkt_ptr_->Fx = -fy;
  thrust_pkt_ptr_->Fy = fx;
  thrust_pkt_ptr_->Fz = fz;
  thrust_pkt_ptr_->Fyaw = fyaw;
  thrust_pkt_ptr_->Fpitch = fr;
  thrust_pkt_ptr_->Froll = -fp;
  taskEXIT_CRITICAL();

  return sendThrustPacketDMA();
}

bool MotionController_Driver::setThrustCurve(uint8_t mode, uint8_t index,
                                             const float pwm[4],
                                             const float thrust[4]) {
  static CurvePacket pkt;
  initPacket(&pkt, 0x07);
  pkt.mode = mode;
  pkt.index = index;
  std::memcpy(pkt.pwm, pwm, 4 * sizeof(float));
  std::memcpy(pkt.thrust, thrust, 4 * sizeof(float));
  return transmitDMA(reinterpret_cast<uint8_t *>(&pkt), sizeof(CurvePacket));
}

bool MotionController_Driver::setServoAngle(float angle) {
  static ServoPacket pkt;
  initPacket(&pkt, 0x02);
  pkt.angle = angle;
  return transmitDMA(reinterpret_cast<uint8_t *>(&pkt), sizeof(ServoPacket));
}

bool MotionController_Driver::setLightState(uint8_t state) {
  static LightPacket pkt;
  initPacket(&pkt, 0x03);
  pkt.state = state;
  return transmitDMA(reinterpret_cast<uint8_t *>(&pkt), sizeof(LightPacket));
}

bool MotionController_Driver::sendThrustPacketDMA() {
  static ThrustPacket dma_pkt
      __attribute__((section(".dma_buffer"), aligned(32)));

  taskENTER_CRITICAL();
  std::memcpy(&dma_pkt, thrust_pkt_ptr_, sizeof(ThrustPacket));
  taskEXIT_CRITICAL();

  return transmitDMA(reinterpret_cast<uint8_t *>(&dma_pkt), sizeof(ThrustPacket));
}

bool MotionController_Driver::transmitDMA(uint8_t *data, uint16_t size) {
  if (!ops_.transmitDMA)
    return false;
  return ops_.transmitDMA(ops_.ctx, data, size);
}

} // namespace peripheral
} // namespace auv
