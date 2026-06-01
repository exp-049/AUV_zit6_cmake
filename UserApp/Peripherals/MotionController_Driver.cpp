#include "MotionController_Driver.hpp"
#include "stm32h7xx_hal.h"
#include <cstring>

namespace auv {
namespace device {

MotionController_Driver::MotionController_Driver(UART_HandleTypeDef *huart,
                                                 ThrustPacket *ext_pkt)
    : huart_(huart), thrust_pkt_ptr_(ext_pkt) {
  initPacket(thrust_pkt_ptr_, 0x01);
  thrust_pkt_ptr_->Fx = 0.0f;
  thrust_pkt_ptr_->Fy = 0.0f;
  thrust_pkt_ptr_->Fz = 0.0f;
  thrust_pkt_ptr_->Fyaw = 0.0f;
  thrust_pkt_ptr_->Fpitch = 0.0f;
  thrust_pkt_ptr_->Froll = 0.0f;
}

MotionController_Driver::MotionController_Driver(UART_HandleTypeDef *huart)
    : huart_(huart), thrust_pkt_ptr_(&thrust_pkt_internal_) {
  initPacket(thrust_pkt_ptr_, 0x01);
  thrust_pkt_ptr_->Fx = 0.0f;
  thrust_pkt_ptr_->Fy = 0.0f;
  thrust_pkt_ptr_->Fz = 0.0f;
  thrust_pkt_ptr_->Fyaw = 0.0f;
  thrust_pkt_ptr_->Fpitch = 0.0f;
  thrust_pkt_ptr_->Froll = 0.0f;
}

void MotionController_Driver::publishThrust(float fx, float fy, float fz,
                                            float fyaw, float fp, float fr) {
  taskENTER_CRITICAL();
  thrust_pkt_ptr_->Fx = -fy;    // Left = Right
  thrust_pkt_ptr_->Fy = fx;     // Front = Front
  thrust_pkt_ptr_->Fz = fz;     // Down = Down
  thrust_pkt_ptr_->Fyaw = fyaw; // Yaw = Yaw
  thrust_pkt_ptr_->Fpitch = fr; // Pitch (around Front) = Roll
  thrust_pkt_ptr_->Froll = -fp; // Roll (around Left) = -Pitch
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
  bool can_start = false;
  taskENTER_CRITICAL();
  if (huart_->gState == HAL_UART_STATE_READY) {
    can_start = true;
  }
  taskEXIT_CRITICAL();

  if (!can_start)
    return;

  if (huart_->hdmatx != NULL) {
    DMA_Stream_TypeDef *dma_stream =
        (DMA_Stream_TypeDef *)huart_->hdmatx->Instance;
    int retries = 3;
    while (retries-- > 0) {
      if ((dma_stream->CR & DMA_SxCR_EN) == 0U)
        break;
      for (volatile int i = 0; i < 100; ++i)
        __asm__ volatile("nop");
    }
    if ((dma_stream->CR & DMA_SxCR_EN) != 0U) {
      return;
    }
  }

  const uintptr_t dcache_line = 32u;
  uintptr_t addr = (uintptr_t)data;
  uintptr_t aligned_addr = addr & ~(dcache_line - 1);
  uintptr_t end = (addr + size + dcache_line - 1) & ~(dcache_line - 1);
  int32_t clean_size = (int32_t)(end - aligned_addr);
  SCB_CleanDCache_by_Addr((uint32_t *)aligned_addr, clean_size);

  HAL_StatusTypeDef res = HAL_UART_Transmit_DMA(huart_, data, size);
  (void)res;
}

} // namespace device
} // namespace auv
