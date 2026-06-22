#include "PID_Controller.hpp"

namespace auv {
namespace algorithm {
namespace control {

PID_Controller::PID_Controller(const Config &config) : cfg_(config) {}

void PID_Controller::setConfig(const Config &config) { cfg_ = config; }

float PID_Controller::compute(float error, float dt, float derivative) {
  if (dt <= 0.0f)
    return 0.0f;

  // 1. 比例项
  float p_out = cfg_.kp * error;

  // 2. 积分项 (采用矩形积分，并带有抗饱和限幅)
  integral_ += cfg_.ki * error * dt;
  integral_ = std::max(-cfg_.i_limit, std::min(cfg_.i_limit, integral_));

  // 3. 微分项 (直接使用传入的测量值导数，避免对误差求导产生的噪声)
  float d_out = cfg_.kd * derivative;

  // 4. 总输出限幅
  float total = p_out + integral_ + d_out;
  return std::max(-cfg_.output_limit, std::min(cfg_.output_limit, total));
}

void PID_Controller::reset_i() { integral_ = 0.0f; }

void PID_Controller::setIntegral(float val) {
  integral_ = std::max(-cfg_.i_limit, std::min(cfg_.i_limit, val));
}

float PID_Controller::getIntegral() const { return integral_; }

} // namespace control
} // namespace algorithm
} // namespace auv
