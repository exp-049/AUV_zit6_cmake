#include "MotionController_Driver.hpp"
#include "stm32h7xx_hal.h"
#include <cstring>

namespace auv {
namespace peripheral {

MotionController_Driver::MotionController_Driver(MotorPortOps ops,
                                                 ThrustPacket *ext_pkt)
    : ops_(ops), thrust_pkt_ptr_(ext_pkt ? ext_pkt
                                         : static_cast<ThrustPacket *>(
                                               ops_.getTxPacket(ops_.ctx))) {
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

void MotionController_Driver::publishThrust(float fx, float fy, float fz,
                                            float fyaw, float fp, float fr) {
  taskENTER_CRITICAL();
  thrust_pkt_ptr_->Fx = -fy;
  thrust_pkt_ptr_->Fy = fx;
  thrust_pkt_ptr_->Fz = fz;
  thrust_pkt_ptr_->Fyaw = fyaw;
  thrust_pkt_ptr_->Fpitch = fr;
  thrust_pkt_ptr_->Froll = -fp;
  taskEXIT_CRITICAL();

  sendThrustPacketDMA();
}

void MotionController_Driver::setThrustCurve(uint8_t mode, uint8_t index,
                                             const float pwm[4],
                                             const float thrust[4]) {
  static CurvePacket pkt;
  initPacket(&pkt, 0x07);
  pkt.mode = mode;
  pkt.index = index;
  std::memcpy(pkt.pwm, pwm, 4 * sizeof(float));
  std::memcpy(pkt.thrust, thrust, 4 * sizeof(float));
  transmitDMA((uint8_t *)&pkt, sizeof(CurvePacket));
}

void MotionController_Driver::setServoAngle(float angle) {
  static ServoPacket pkt;
  initPacket(&pkt, 0x02);
  pkt.angle = angle;
  transmitDMA((uint8_t *)&pkt, sizeof(ServoPacket));
}

void MotionController_Driver::setLightState(uint8_t state) {
  static LightPacket pkt;
  initPacket(&pkt, 0x03);
  pkt.state = state;
  transmitDMA((uint8_t *)&pkt, sizeof(LightPacket));
}

void MotionController_Driver::sendThrustPacketDMA() {
  static ThrustPacket dma_pkt
      __attribute__((section(".dma_buffer"), aligned(32)));

  taskENTER_CRITICAL();
  std::memcpy(&dma_pkt, thrust_pkt_ptr_, sizeof(ThrustPacket));
  taskEXIT_CRITICAL();

  transmitDMA((uint8_t *)&dma_pkt, sizeof(ThrustPacket));
}

void MotionController_Driver::transmitDMA(uint8_t *data, uint16_t size) {
  if (ops_.transmitDMA)
    ops_.transmitDMA(ops_.ctx, data, size);
}

} // namespace peripheral
} // namespace auv
