#ifndef PUSHROD_GPIO_BACKEND_HPP
#define PUSHROD_GPIO_BACKEND_HPP

#include "Pushrod_Backend.hpp"

#include <stdint.h>

namespace auv {
namespace peripheral {

/** Hardware hooks supplied by the PB8/PB7 GPIO porting layer. */
struct PushrodGpioPortOps {
  void *ctx;
  bool (*init)(void *ctx);
  bool (*setOutputs)(void *ctx, bool in1, bool in2);
  uint32_t (*getTickMs)(void *ctx);
};

/**
 * @brief Non-PWM pushrod backend using a two-input motor bridge.
 *
 * PB8 is IN1 and PB7 is IN2. The magnitude of power_x1000 is deliberately
 * ignored because this backend has no PWM capability:
 *
 *   power > 0: IN1=1, IN2=0
 *   power < 0: IN1=0, IN2=1
 *   power = 0: IN1=0, IN2=0 (safe stop)
 *
 * Duration is still honored by poll(). A non-zero task produces the same
 * application-level ACK consumed by MicroRosSubscriber only after the timed
 * action has stopped, so the ROS topic and queue behavior remain safe.
 */
class Pushrod_GPIO_Backend final : public PushrodBackend {
public:
  explicit Pushrod_GPIO_Backend(PushrodGpioPortOps ops);

  bool init() override;
  void start() override;
  void poll(uint32_t now_ms) override;
  void stop() override;
  bool sendTask(const PushrodTask &task) override;
  bool readAck(PushrodAck *ack) override;
  bool isSupported() const override;

  bool isActive() const { return active_; }
  bool isInitialized() const { return initialized_; }

private:
  bool setOutputs(bool in1, bool in2);
  void queueAck(const PushrodTask &task);
  bool finishActiveTask();

  PushrodGpioPortOps ops_{};
  bool initialized_ = false;
  bool active_ = false;
  PushrodTask active_task_{};
  uint32_t active_task_id_ = 0U;
  uint32_t active_started_ms_ = 0U;
  uint32_t active_duration_ms_ = 0U;
  PushrodAck ack_{};
  bool ack_ready_ = false;
};

} // namespace peripheral
} // namespace auv

#endif
