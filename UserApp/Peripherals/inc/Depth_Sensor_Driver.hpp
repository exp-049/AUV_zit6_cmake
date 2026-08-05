#ifndef DEPTH_SENSOR_DRIVER_HPP
#define DEPTH_SENSOR_DRIVER_HPP

#include "Pushrod_Backend.hpp"

#include <stdint.h>

namespace auv {
namespace peripheral {

struct DepthDataReadyCallback {
  void (*onDepthReady)(void *ctx, float depth, float temperature);
  void *ctx;
};

struct DepthDiagnostics {
  bool connected = false;
  bool handshake_acknowledged = false;
  uint32_t rx_recovery_count = 0U;
  uint32_t rx_error_count = 0U;
  uint32_t last_rx_error = 0U;
  uint32_t last_rx_recovery_reason = 0U;
  uint32_t rx_event_count = 0U;
  uint32_t dma_write_pos = 0U;
};

/** Hardware operations used by the direct-I2C MS5837 backend. */
struct DepthPortOps {
  void *ctx;
  bool (*writeByte)(void *ctx, uint8_t cmd);
  bool (*readByte)(void *ctx, uint8_t *data);
  bool (*read)(void *ctx, uint8_t *data, uint16_t size);
  void (*delay)(void *ctx, uint32_t ms);
  void (*start)(void *ctx);
};

/** Depth-only backend contract shared by all sensor implementations. */
struct DepthBackend {
  virtual ~DepthBackend() = default;

  virtual bool init() = 0;
  virtual void poll() = 0;
  virtual bool read() = 0;
  virtual void setCallback(DepthDataReadyCallback cb) = 0;
  virtual void start() = 0;
  virtual bool isConnected() const = 0;
  virtual float getDepth() const = 0;
  virtual float getTemperature() const = 0;

  virtual void getDiagnostics(DepthDiagnostics &out) const {
    out = {};
    out.connected = isConnected();
  }
};

/**
 * Stable depth-sensor facade. Application code depends on this class rather
 * than on a concrete UART or I2C implementation.
 */
class Depth_Sensor_Driver {
public:
  explicit Depth_Sensor_Driver(DepthBackend *backend,
                               PushrodBackend *legacy_pushrod = nullptr);
  ~Depth_Sensor_Driver();

  void Init(void);
  void start();
  int Read();

  void Depth(float *p);
  float getMS5837Z();
  float getTemperature() const { return temperture; }
  void setMS5837Z(float z);
  inline void altitude(float *p);

  // Deprecated source-compatibility bridge. New code uses Pushrod_Driver.
  bool sendPushrodTask(const PushrodTask &task);
  bool sendPushrodTask(uint32_t task_id, int16_t power_x1000,
                       uint32_t duration_ms);
  bool readPushrodAck(PushrodAck *ack);

  bool isConnected() const { return is_connected; }
  void getDiagnostics(DepthDiagnostics &out) const;

  // Compatibility diagnostics retained for existing telemetry/debug callers.
  bool isHandshakeAcknowledged() const;
  uint32_t getRxRecoveryCount() const;
  uint32_t getRxErrorCount() const;
  uint32_t getLastRxError() const;
  uint32_t getLastRxRecoveryReason() const;
  uint32_t getRxEventCount() const;
  uint32_t getDmaWritePos() const;

  bool is_connected = false;
  float pressure = 0.0f;
  float temperture = 0.0f;

private:
  DepthBackend *backend_;
  PushrodBackend *legacy_pushrod_backend_;
  float last_valid_depth = 0.0f;
  bool has_valid_depth = false;
};

} // namespace peripheral
} // namespace auv

#endif
