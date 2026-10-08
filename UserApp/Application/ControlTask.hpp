#ifndef __CONTROL_TASK_HPP
#define __CONTROL_TASK_HPP

#include "../Common/AppContext.hpp"
#include "../Common/DepthFusion.hpp"
#include "../Common/MotionContext.hpp"
#include "USBL_Driver.hpp"
#include "SensorDataMonitor.hpp"
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
  uint32_t last_nav_sample_ms_ = 0;
  uint32_t last_depth_sample_ms_ = 0;
  uint32_t last_pressure_alarm_ms_ = 0;
  bool have_nav_sample_ = false;
  bool have_depth_sample_ = false;
  bool have_pressure_mode_z_ = false;
  float last_pressure_mode_z_ = 0.0f;
  auv::motion::DepthFusion depth_fusion_{};
  auv::peripheral::UsblState usbl_state_{};
  auv::component::SensorDataMonitor ins_data_monitor_;
  auv::component::SensorDataMonitor depth_data_monitor_;

  void init();
  void updateNavigation();
  void computeAndPublish();
  bool monitorSensorReception(uint32_t now_ms, bool ins_frame_ready,
                              int depth_frame_ready, float z);
  void logInsDiagnostics();
  void logDepthDiagnostics(int frame_ready, float z);
};

#endif // __CONTROL_TASK_HPP
