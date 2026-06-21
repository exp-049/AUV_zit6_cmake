#ifndef __MICROROS_TASK_HPP
#define __MICROROS_TASK_HPP

#include "MicroRosPublisher.hpp"
#include "MicroRosService.hpp"
#include "MicroRosSubscriber.hpp"
#include "MicroRosTransport.hpp"

#include <stdint.h>

/**
 * @class MicroRosTask
 * @brief micro-ROS 主控协调器
 *
 * 职责：协调 Transport / Publisher / Subscriber / Service
 * 四个子模块的生命周期。 主循环：
 *   1. WAITING_AGENT: Transport 探测 Agent → 初始化各模块
 *   2. AGENT_CONNECTED: spin executor + Publisher 定时发布
 *   3. Agent 断联 → cleanup 各模块 → 回到 WAITING_AGENT
 *
 * 原有业务回调已拆分到 MicroRosSubscriber / MicroRosService 中。
 */
class MicroRosTask {
public:
  MicroRosTask() = default;
  void run();

private:
  // 四个子模块（按依赖顺序声明：Transport 最先构造/最后析构）
  MicroRosTransport transport_;
  MicroRosPublisher publisher_;
  MicroRosSubscriber subscriber_;
  MicroRosService service_;

  // 状态枚举
  enum class State : uint8_t { WAITING_AGENT, AGENT_CONNECTED };
  State state_ = State::WAITING_AGENT;

  // Agent ping 节流
  uint32_t last_ping_ms_ = 0;

  void connectAgent();
  void disconnectAgent();
};

#endif // __MICROROS_TASK_HPP
