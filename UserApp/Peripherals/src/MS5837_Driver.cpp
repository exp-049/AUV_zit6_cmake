#include "MS5837_Driver.hpp"
#include "FreeRTOS.h"
#include "MS5837_LogConfig.hpp"
#include "task.h"

namespace auv {
namespace peripheral {

MS5837_Driver::MS5837_Driver(DepthBackend *backend) : backend_(backend) {
  last_valid_depth = 0.0f;
  has_valid_depth = false;
  MS5837_LOG_DEBUG("[MS5837] Driver constructed, backend=%p", backend_);
}

MS5837_Driver::~MS5837_Driver() {}

void MS5837_Driver::Init(void) {
  MS5837_LOG_DEBUG("[MS5837] Init() enter, backend=%p", backend_);
  if (backend_) {
    // 注册数据就绪回调：backend 新数据 → setMS5837Z()
    backend_->setCallback(DepthDataReadyCallback{
        .onDepthReady =
            [](void *ctx, float d, float t) {
              MS5837_LOG_DEBUG(
                  "[MS5837] callback onDepthReady(d=%.4f, t=%.2f)", d, t);
              static_cast<MS5837_Driver *>(ctx)->setMS5837Z(d);
            },
        .ctx = this,
    });
    // init() only proves that the backend transport initialized. A UART
    // protocol backend cannot report the sensor as connected until it has
    // received a valid DATA frame.
    const bool init_ok = backend_->init();
    is_connected = init_ok && backend_->isConnected();
    MS5837_LOG_DEBUG("[MS5837] Init() backend->init -> connected=%d",
                     is_connected);
  } else {
    MS5837_LOG_DEBUG("[MS5837] Init() FAIL: backend_ is NULL!");
  }
}

void MS5837_Driver::start() {
  MS5837_LOG_DEBUG("[MS5837] start()");
  if (backend_) {
    backend_->start();
  } else {
    MS5837_LOG_DEBUG("[MS5837] start() FAIL: backend_ is NULL!");
  }
}

int MS5837_Driver::Read() {
  if (backend_) {
    backend_->poll(); // 喂 DMA 数据（UART）或无操作（I2C）
    // UART V1 的握手 ACK 也能证明设备在线，即使下一条 DATA 还未到达。
    is_connected = backend_->isConnected();
    if (backend_->read()) {
      // Backend 有新数据，同步到数据容器
      temperture = backend_->getTemperature();
      // pressure 字段在 I2C 模式下由 calculate 计算得出，
      // UART 模式下可能没有压力值；保留供 Depth() fallback 使用
      MS5837_LOG_DEBUG("[MS5837] Read() got new data: depth=%.4f, temp=%.2f",
                       backend_->getDepth(), temperture);
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

bool MS5837_Driver::isHandshakeAcknowledged() const {
  return backend_ != nullptr && backend_->isHandshakeAcknowledged();
}

uint32_t MS5837_Driver::getRxRecoveryCount() const {
  return backend_ != nullptr ? backend_->getRxRecoveryCount() : 0U;
}

uint32_t MS5837_Driver::getRxErrorCount() const {
  return backend_ != nullptr ? backend_->getRxErrorCount() : 0U;
}

uint32_t MS5837_Driver::getLastRxError() const {
  return backend_ != nullptr ? backend_->getLastRxError() : 0U;
}

uint32_t MS5837_Driver::getLastRxRecoveryReason() const {
  return backend_ != nullptr ? backend_->getLastRxRecoveryReason() : 0U;
}

uint32_t MS5837_Driver::getRxEventCount() const {
  return backend_ != nullptr ? backend_->getRxEventCount() : 0U;
}

uint32_t MS5837_Driver::getDmaWritePos() const {
  return backend_ != nullptr ? backend_->getDmaWritePos() : 0U;
}

void MS5837_Driver::setMS5837Z(float z) {
  taskENTER_CRITICAL();
  last_valid_depth = z;
  has_valid_depth = true;
  taskEXIT_CRITICAL();
  MS5837_LOG_DEBUG("[MS5837] setMS5837Z(%.4f) -> cache updated", z);
}

} // namespace peripheral
} // namespace auv
