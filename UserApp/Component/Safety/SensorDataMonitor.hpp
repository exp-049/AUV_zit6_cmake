#pragma once

#include <cstdint>

namespace auv::component {

/** Tracks valid-frame arrivals independently of a driver's connected flag. */
class SensorDataMonitor {
public:
  struct Result {
    bool timed_out = false;
    bool alarm_due = false;
    bool recovered = false;
    uint32_t age_ms = 0U;
  };

  static constexpr uint32_t kAlarmIntervalMs = 5000U;

  void reset(uint32_t now_ms) {
    last_frame_ms_ = now_ms;
    last_alarm_ms_ = now_ms;
    timed_out_ = false;
  }

  Result update(uint32_t now_ms, bool received_valid_frame,
                uint32_t timeout_ms) {
    const bool was_timed_out = timed_out_;
    if (received_valid_frame) {
      last_frame_ms_ = now_ms;
    }
    const uint32_t age_ms = now_ms - last_frame_ms_;
    timed_out_ = !received_valid_frame && age_ms >= timeout_ms;

    const bool alarm_due = timed_out_ &&
        (!was_timed_out || now_ms - last_alarm_ms_ >= kAlarmIntervalMs);
    if (alarm_due) {
      last_alarm_ms_ = now_ms;
    }
    return {timed_out_, alarm_due,
            was_timed_out && received_valid_frame, age_ms};
  }

private:
  uint32_t last_frame_ms_ = 0U;
  uint32_t last_alarm_ms_ = 0U;
  bool timed_out_ = false;
};

} // namespace auv::component
