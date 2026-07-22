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
      internal_pkt_{}, handshake_rx_state_(0U), handshake_rx_status_(0U),
      handshake_response_pending_(false) {
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
  // Rebuild the fixed header/tail on every frame.  The external packet buffer
  // is placed in a NOLOAD DMA section, so constructor-time initialization is
  // not sufficient to guarantee that these bytes remain intact.
  initPacket(thrust_pkt_ptr_, 0x01);
  thrust_pkt_ptr_->Fx = fx;
  thrust_pkt_ptr_->Fy = -fy;
  thrust_pkt_ptr_->Fz = fz;
  thrust_pkt_ptr_->Fyaw = fyaw;
  thrust_pkt_ptr_->Fpitch = fp;
  thrust_pkt_ptr_->Froll = fr;
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

bool MotionController_Driver::setThrustMatrix(uint8_t mode, float A_1,
                                              float A_2, float B, float C,
                                              float _2b) {
  static MatrixPacket pkt;
  initPacket(&pkt, 0x10);
  pkt.mode = mode;
  pkt.A_1 = A_1;
  pkt.A_2 = A_2;
  pkt.B = B;
  pkt.C = C;
  pkt._2b = _2b;
  return transmitDMA(reinterpret_cast<uint8_t *>(&pkt), sizeof(MatrixPacket));
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

bool MotionController_Driver::sendHandshake() {
  static HandshakePacket pkt;
  initPacket(&pkt, 0x04);
  return transmitDMA(reinterpret_cast<uint8_t *>(&pkt), sizeof(pkt));
}

void MotionController_Driver::onRxByte(uint8_t byte) {
  switch (handshake_rx_state_) {
  case 0U:
    handshake_rx_state_ = byte == 0xFAU ? 1U : 0U;
    break;
  case 1U:
    if (byte == 0xAFU) {
      handshake_rx_state_ = 2U;
    } else {
      handshake_rx_state_ = byte == 0xFAU ? 1U : 0U;
    }
    break;
  case 2U:
    handshake_rx_state_ = byte == 0x04U ? 3U : (byte == 0xFAU ? 1U : 0U);
    break;
  case 3U:
    handshake_rx_status_ = byte;
    handshake_rx_state_ = 4U;
    break;
  case 4U:
    handshake_rx_state_ = byte == 0xFBU ? 5U : (byte == 0xFAU ? 1U : 0U);
    break;
  case 5U:
    if (byte == 0xBFU) {
      handshake_response_pending_ = true;
    }
    handshake_rx_state_ = byte == 0xFAU ? 1U : 0U;
    break;
  default:
    handshake_rx_state_ = 0U;
    break;
  }
}

bool MotionController_Driver::takeHandshakeResponse(uint8_t &status) {
  if (!handshake_response_pending_) {
    return false;
  }

  status = handshake_rx_status_;
  handshake_response_pending_ = false;
  return true;
}

bool MotionController_Driver::sendThrustPacketDMA() {
  static ThrustPacket dma_pkt
      __attribute__((section(".dma_buffer"), aligned(32)));

  taskENTER_CRITICAL();
  std::memcpy(&dma_pkt, thrust_pkt_ptr_, sizeof(ThrustPacket));
  taskEXIT_CRITICAL();

  return transmitDMA(reinterpret_cast<uint8_t *>(&dma_pkt),
                     sizeof(ThrustPacket));
}

bool MotionController_Driver::transmitDMA(uint8_t *data, uint16_t size) {
  if (!ops_.transmitDMA)
    return false;
  return ops_.transmitDMA(ops_.ctx, data, size);
}

} // namespace peripheral
} // namespace auv
