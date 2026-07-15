#ifndef __CONTROL_TASK_HPP
#define __CONTROL_TASK_HPP

#include "../Common/AppContext.hpp"
#include "../Common/MotionContext.hpp"
#include "../Peripherals/inc/USBL_Driver.hpp"
#include <cstdint>

class ControlTask {
public:
  explicit ControlTask(auv::system::AppContext *ctx) : ctx_(ctx) {}
  void run();

private:
  auv::system::AppContext *ctx_;
  static constexpr uint32_t kLoopPeriodMs = 10;

  uint32_t last_wake_time_ = 0;
  uint32_t last_tick_ = 0;
  uint32_t overrun_count_ = 0;
  bool first_cycle_ = true;

  /** SITL 模式下无新数据时保持的上次有效导航状态 */
  auv::motion::NavState last_sitl_state_{};
  auv::peripheral::UsblState usbl_state_{};

  void init();
  void updateNavigation();
  void computeAndPublish();
};

#endif // __CONTROL_TASK_HPP
