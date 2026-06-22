#pragma once

// ChassisManager 桩 — 用于主机端测试
// 实现 SafetyMonitor + ConfigService 需要的接口

#include <cstdint>
#include "MotionContext.hpp"
#include "SystemConfig.hpp"

namespace auv {
namespace component {

class ChassisManager {
public:
  void setControlLevel(auv::motion::ControlLevel level) { level_ = level; }
  auv::motion::ControlLevel getControlLevel() const { return level_; }

  void applyConfig(const auv::config::ChassisConfig &cfg) { config_ = cfg; }
  void configurePID(int axis, bool is_pos_ring, float kp, float ki, float kd,
                    float i_limit, float out_limit) {}
  void configureProfile(int axis, float max_v, float max_a) {}

  // 测试辅助
  void setMockLevel(auv::motion::ControlLevel level) { level_ = level; }

private:
  auv::motion::ControlLevel level_ = auv::motion::ControlLevel::NONE;
  auv::config::ChassisConfig config_;
};

} // namespace component
} // namespace auv
