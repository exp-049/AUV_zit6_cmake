#pragma once

#include "CascadeController.hpp"
#include "SetpointRouter.hpp"
#include "SystemConfig.hpp"
#include <array>

namespace auv {
namespace component {

/**
 * @class ChassisManager
 * @brief 整合 SetpointRouter（协议路由）+ CascadeController（数值控制）
 */
class ChassisManager {
public:
  ChassisManager();
  ChassisManager(const auv::config::ChassisConfig &cfg);

  void applyConfig(const auv::config::ChassisConfig &cfg);
  auv::motion::ControlLevel getControlLevel() const;
  std::array<float, 6> update();

  void updateSetpoint(auv::motion::ControlLevel new_level, const float val[6],
                      uint32_t mask, bool is_body, bool is_inc);
  void setControlLevel(auv::motion::ControlLevel new_level);

  void configurePID(int axis, bool is_pos_ring, float kp, float ki, float kd,
                    float i_limit, float out_limit);
  PID_Controller::Config getPIDConfig(int axis, bool is_pos_ring) const;
  void getProfileLimits(int axis, float &max_v, float &max_a) const;
  void configureProfile(int axis, float max_v, float max_a);

private:
  SetpointRouter router_;
  CascadeController controller_;
};

} // namespace component
} // namespace auv
