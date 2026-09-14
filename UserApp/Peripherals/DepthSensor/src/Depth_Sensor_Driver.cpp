#include "Depth_Sensor_Driver.hpp"

#include "FreeRTOS.h"
#include "MS5837_LogConfig.hpp"
#include "math.h"
#include "task.h"

namespace auv {
namespace peripheral {

Depth_Sensor_Driver::Depth_Sensor_Driver(DepthBackend *backend)
    : backend_(backend) {
  MS5837_LOG_DEBUG("[Depth] Driver constructed, backend=%p", backend_);
}

Depth_Sensor_Driver::~Depth_Sensor_Driver() {}

void Depth_Sensor_Driver::Init(void) {
  MS5837_LOG_DEBUG("[Depth] Init() enter, backend=%p", backend_);
  if (backend_ == nullptr) {
    is_connected = false;
    return;
  }

  backend_->setCallback(DepthDataReadyCallback{
      .onDepthReady = [](void *ctx, float depth, float temperature) {
        auto *self = static_cast<Depth_Sensor_Driver *>(ctx);
        self->temperture = temperature;
        self->setMS5837Z(depth);
      },
      .ctx = this,
  });

  const bool init_ok = backend_->init();
  is_connected = init_ok && backend_->isConnected();
  MS5837_LOG_DEBUG("[Depth] Init() connected=%d", is_connected ? 1 : 0);
}

void Depth_Sensor_Driver::start() {
  if (backend_ != nullptr) {
    backend_->start();
  }
}

int Depth_Sensor_Driver::Read() {
  if (backend_ == nullptr) {
    is_connected = false;
    return 0;
  }

  backend_->poll();
  is_connected = backend_->isConnected();
  if (!backend_->read()) {
    return 0;
  }

  // Keep the facade cache authoritative even if a backend implementation
  // delivers its callback asynchronously or loses the callback during a
  // transport recovery. A successful backend read already means that its
  // current depth is valid.
  const float depth = backend_->getDepth();
  temperture = backend_->getTemperature();
  setMS5837Z(depth);
  return 1;
}

void Depth_Sensor_Driver::Depth(float *p) {
  if (p == nullptr) {
    return;
  }

  if (has_valid_depth) {
    taskENTER_CRITICAL();
    *p = last_valid_depth;
    taskEXIT_CRITICAL();
  } else if (backend_ != nullptr) {
    *p = backend_->getDepth();
  } else {
    *p = 0.0f;
  }
}

inline void Depth_Sensor_Driver::altitude(float *p) {
  if (p != nullptr) {
    *p = (1 - pow((pressure / 1013.25), .190284)) * 145366.45 * .3048;
  }
}

float Depth_Sensor_Driver::getMS5837Z() {
  float z = 0.0f;
  Depth(&z);
  return z;
}

void Depth_Sensor_Driver::setMS5837Z(float z) {
  taskENTER_CRITICAL();
  last_valid_depth = z;
  has_valid_depth = true;
  taskEXIT_CRITICAL();
}

void Depth_Sensor_Driver::getDiagnostics(DepthDiagnostics &out) const {
  out = {};
  out.connected = is_connected;
  if (backend_ != nullptr) {
    backend_->getDiagnostics(out);
    out.connected = is_connected;
  }
}

bool Depth_Sensor_Driver::isHandshakeAcknowledged() const {
  DepthDiagnostics diagnostics;
  getDiagnostics(diagnostics);
  return diagnostics.handshake_acknowledged;
}

uint32_t Depth_Sensor_Driver::getRxRecoveryCount() const {
  DepthDiagnostics diagnostics;
  getDiagnostics(diagnostics);
  return diagnostics.rx_recovery_count;
}

uint32_t Depth_Sensor_Driver::getRxErrorCount() const {
  DepthDiagnostics diagnostics;
  getDiagnostics(diagnostics);
  return diagnostics.rx_error_count;
}

uint32_t Depth_Sensor_Driver::getLastRxError() const {
  DepthDiagnostics diagnostics;
  getDiagnostics(diagnostics);
  return diagnostics.last_rx_error;
}

uint32_t Depth_Sensor_Driver::getLastRxRecoveryReason() const {
  DepthDiagnostics diagnostics;
  getDiagnostics(diagnostics);
  return diagnostics.last_rx_recovery_reason;
}

uint32_t Depth_Sensor_Driver::getRxEventCount() const {
  DepthDiagnostics diagnostics;
  getDiagnostics(diagnostics);
  return diagnostics.rx_event_count;
}

uint32_t Depth_Sensor_Driver::getDmaWritePos() const {
  DepthDiagnostics diagnostics;
  getDiagnostics(diagnostics);
  return diagnostics.dma_write_pos;
}

} // namespace peripheral
} // namespace auv
