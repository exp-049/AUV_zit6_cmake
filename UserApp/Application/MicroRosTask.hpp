#ifndef __MICROROS_TASK_HPP
#define __MICROROS_TASK_HPP

#include "AppContext.hpp"
#include "FreeRTOS.h"
#include "MicroRosPublisher.hpp"
#include "MicroRosService.hpp"
#include "MicroRosSubscriber.hpp"
#include "MicroRosTransport.hpp"

class MicroRosTask {
public:
  MicroRosTask(auv::system::AppContext *ctx) : ctx_(ctx) {}
  void run();

private:
  auv::system::AppContext *ctx_;

  MicroRosTransport transport_;
  MicroRosPublisher publisher_{ctx_};
  MicroRosSubscriber subscriber_{ctx_};
  MicroRosService service_{ctx_};

  enum class State : uint8_t { WAITING_AGENT, AGENT_CONNECTED };
  State state_ = State::WAITING_AGENT;

  static constexpr uint32_t kLoopPeriodMs = 2;
  static constexpr uint32_t kAgentPingWatchdogTimeoutMs = 5000;

  uint32_t last_ping_ms_ = 0;
  // 最近一次成功 ping Agent 的时间。初始值由 run() 设置，用于给启动阶段
  // 保留一个 5 秒窗口；超过该窗口未成功 ping 时停止喂 micro-ROS 软件狗。
  uint32_t last_agent_ping_ok_ms_ = 0;
  TickType_t last_wake_time_ = 0;

  void connectAgent();
  void disconnectAgent();
};

#endif // __MICROROS_TASK_HPP
