#include "MS5837_Driver.hpp"
#include "FreeRTOS.h"
#include "task.h"

namespace auv {
namespace peripheral {

MS5837_Driver::MS5837_Driver(DepthBackend *backend) : backend_(backend) {
  last_valid_depth = 0.0f;
  has_valid_depth = false;
}

MS5837_Driver::~MS5837_Driver() {}

void MS5837_Driver::Init(void) {
  if (backend_) {
    // 注册数据就绪回调：backend 新数据 → setMS5837Z()
    backend_->setCallback(DepthDataReadyCallback{
        .onDepthReady =
            [](void *ctx, float d, float t) {
              static_cast<MS5837_Driver *>(ctx)->setMS5837Z(d);
            },
        .ctx = this,
    });
    is_connected = backend_->init();
  }
}

void MS5837_Driver::start() {
  if (backend_) {
    backend_->start();
  }
}

int MS5837_Driver::Read() {
  if (backend_) {
    backend_->poll(); // 喂 DMA 数据（UART）或无操作（I2C）
    if (backend_->read()) {
      // Backend 有新数据，同步到数据容器
      temperture = backend_->getTemperature();
      // pressure 字段在 I2C 模式下由 calculate 计算得出，
      // UART 模式下可能没有压力值；保留供 Depth() fallback 使用
      if (backend_->isConnected()) {
        is_connected = true;
      }
      return 1;
    }
  }
  return 0;
}

void MS5837_Driver::Depth(float *p) {
  if (has_valid_depth) {
    taskENTER_CRITICAL();
    *p = last_valid_depth;
    taskEXIT_CRITICAL();
  } else if (backend_) {
    *p = backend_->getDepth();
  } else {
    *p = 0.0f;
  }
}

inline void MS5837_Driver::altitude(float *p) {
  // 基于压力计算海拔（使用内部 pressure 字段）
  *p = (1 - pow((pressure / 1013.25), .190284)) * 145366.45 * .3048;
}

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
