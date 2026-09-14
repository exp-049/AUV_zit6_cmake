#include "SoftWatchdog.hpp"
#include "stm32h7xx_hal.h"

namespace auv {
namespace component {

void SoftWatchdog::init(const auv::config::SoftWatchdogConfig &config) {
  config_ = config;
  uint32_t now = HAL_GetTick();
  for (int i = 0; i < (int)Component::COUNT; i++) {
    last_feed_ms_[i] = now;
  }
}

void SoftWatchdog::feed(Component component) {
  last_feed_ms_[(int)component] = HAL_GetTick();
}

bool SoftWatchdog::check() {
  uint32_t now = HAL_GetTick();

  if (config_.check_microros) {
    if (now - last_feed_ms_[(int)Component::MICROROS] > config_.timeout_ms)
      return false;
  }

  if (config_.check_ins) {
    if (now - last_feed_ms_[(int)Component::INS] > config_.timeout_ms)
      return false;
  }

  if (config_.check_depth) {
    if (now - last_feed_ms_[(int)Component::DEPTH] > config_.timeout_ms)
      return false;
  }

  return true;
}

} // namespace component
} // namespace auv
