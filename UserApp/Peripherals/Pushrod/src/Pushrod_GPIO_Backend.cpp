#include "Pushrod_GPIO_Backend.hpp"

namespace auv {
namespace peripheral {

namespace {
constexpr int16_t kPowerMin = -1000;
constexpr int16_t kPowerMax = 1000;
}

Pushrod_GPIO_Backend::Pushrod_GPIO_Backend(PushrodGpioPortOps ops)
    : ops_(ops) {}

bool Pushrod_GPIO_Backend::init() {
  initialized_ = false;
  active_ = false;
  ack_ = {};
  ack_ready_ = false;

  if (ops_.init == nullptr || ops_.setOutputs == nullptr) {
    return false;
  }
  if (!ops_.init(ops_.ctx)) {
    return false;
  }
  initialized_ = setOutputs(false, false);
  return initialized_;
}

void Pushrod_GPIO_Backend::start() {
  if (initialized_) {
    stop();
  }
}

void Pushrod_GPIO_Backend::poll(uint32_t now_ms) {
  if (!initialized_ || !active_) {
    return;
  }

  // A caller can legitimately pass a tick captured just before sendTask().
  // Treat a small backwards jump as "not elapsed" instead of allowing the
  // unsigned subtraction below to wrap and finish the task immediately.
  if (static_cast<int32_t>(now_ms - active_started_ms_) < 0) {
    return;
  }

  if (static_cast<uint32_t>(now_ms - active_started_ms_) >=
      active_duration_ms_) {
    (void)finishActiveTask();
  }
}

void Pushrod_GPIO_Backend::stop() {
  active_ = false;
  active_duration_ms_ = 0U;
  if (initialized_) {
    (void)setOutputs(false, false);
  }
}

bool Pushrod_GPIO_Backend::setOutputs(bool in1, bool in2) {
  if (ops_.setOutputs == nullptr) {
    return false;
  }

  /* The porting implementation makes the transition break-before-make. */
  return ops_.setOutputs(ops_.ctx, in1, in2);
}

bool Pushrod_GPIO_Backend::finishActiveTask() {
  if (!active_) {
    return false;
  }

  /* A GPIO ACK means the timed motor action has safely ended. */
  if (!setOutputs(false, false)) {
    return false;
  }

  active_ = false;
  active_duration_ms_ = 0U;
  queueAck(active_task_);
  return true;
}

void Pushrod_GPIO_Backend::queueAck(const PushrodTask &task) {
  ack_.task_id = task.task_id;
  ack_.result = pushrod::kOk;
  ack_.queue_count = 0U;
  ack_.ready = initialized_ ? 1U : 0U;
  ack_ready_ = true;
}

bool Pushrod_GPIO_Backend::sendTask(const PushrodTask &task) {
  if (!initialized_ || task.duration_ms == 0U ||
      task.power_x1000 < kPowerMin || task.power_x1000 > kPowerMax) {
    return false;
  }

  /* Retries for the same task are idempotent; a different task waits. */
  if (active_) {
    return task.task_id == active_task_id_;
  }
  if (ack_ready_) {
    return false;
  }

  const bool in1 = task.power_x1000 > 0;
  const bool in2 = task.power_x1000 < 0;
  if (!setOutputs(in1, in2)) {
    return false;
  }

  active_task_id_ = task.task_id;
  active_task_ = task;
  active_started_ms_ = ops_.getTickMs != nullptr
                           ? ops_.getTickMs(ops_.ctx)
                           : 0U;
  active_duration_ms_ = task.duration_ms;
  active_ = in1 || in2;

  /* Zero speed is a valid safe-stop command and completes immediately. */
  if (!active_) {
    active_duration_ms_ = 0U;
  }

  if (!active_) {
    queueAck(task);
  }
  return true;
}

bool Pushrod_GPIO_Backend::readAck(PushrodAck *ack) {
  if (ack == nullptr || !ack_ready_) {
    return false;
  }
  *ack = ack_;
  ack_ready_ = false;
  return true;
}

bool Pushrod_GPIO_Backend::isSupported() const {
  return ops_.init != nullptr && ops_.setOutputs != nullptr;
}

} // namespace peripheral
} // namespace auv
