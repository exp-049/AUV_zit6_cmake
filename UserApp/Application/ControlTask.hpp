#ifndef __CONTROL_TASK_HPP
#define __CONTROL_TASK_HPP

#include "AppContext.hpp"
#include "FreeRTOS.h"
#include "MotionContext.hpp"
#include <stdint.h>

class ControlTask {
public:
  ControlTask(auv::system::AppContext *ctx) : ctx_(ctx) {}
  void run();

private:
  auv::system::AppContext *ctx_;
  static constexpr uint32_t kLoopPeriodMs = 10;

  TickType_t last_wake_time_ = 0;
  uint32_t last_tick_ = 0;

  /** SITL 模式下无新数据时保持的上次有效导航状态 */
  auv::motion::NavState last_sitl_state_{};

  void init();
  void updateNavigation();
  void computeAndPublish();
};

#endif // __CONTROL_TASK_HPP
