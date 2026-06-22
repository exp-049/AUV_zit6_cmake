/**
 * @file PID_Controller.hpp
 * @brief 通用 PID 控制器算法类
 *
 * 功能特性：
 * 1. 经典的增量式积分项计算。
 * 2. 内置抗积分饱和 (Anti-windup) 限幅，防止在大误差下积分器过度膨胀。
 * 3. 结果总输出限幅，保护执行器不超限。
 * 4. 支持积分项手动设置，用于控制权切换时的“状态继承”。
 */

#pragma once

#include <algorithm>

namespace auv {
namespace algorithm {
namespace control {

/**
 * @class PID_Controller
 * @brief 标准 PID 控制实现
 */
class PID_Controller {
public:
  /**
   * @struct Config
   * @brief PID 配置参数结构体
   */
  struct Config {
    float kp = 0.0f;           ///< 比例增益
    float ki = 0.0f;           ///< 积分增益
    float kd = 0.0f;           ///< 微分增益
    float i_limit = 1.0f;      ///< 积分项幅度限制 (Anti-windup)
    float output_limit = 1.0f; ///< 最终总输出限制
    float dt = 0.01f;          ///< 控制周期 (秒)
  };

  PID_Controller() = default;
  explicit PID_Controller(const Config &config);

  /**
   * @brief 更新配置参数
   */
  void setConfig(const Config &config);

  /**
   * @brief 获取当前配置参数
   */
  const Config &getConfig() const { return cfg_; }

  /**
   * @brief 执行一次 PID 计算
   * @param error 当前残差 (Setpoint - Actual)
   * @param dt 实际经过的时间步长 (秒)
   * @param derivative 外部传入的微分项（可选，通常为 -v）
   * @return 经过限幅后的控制输出
   */
  float compute(float error, float dt, float derivative = 0.0f);

  /**
   * @brief 复位积分状态
   */
  void reset_i();

  /**
   * @brief 手动设置积分值
   * @note 常用于深度模式切换时的“推力继承”逻辑
   */
  void setIntegral(float val);

  /**
   * @brief 获取当前积分器的累积值
   */
  float getIntegral() const;

private:
  Config cfg_;
  float integral_ = 0.0f; ///< 积分累加器
};

} // namespace control
} // namespace algorithm
} // namespace auv
