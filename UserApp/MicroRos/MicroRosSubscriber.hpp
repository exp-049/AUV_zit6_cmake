#ifndef __MICROROS_SUBSCRIBER_HPP
#define __MICROROS_SUBSCRIBER_HPP

#include "AppContext.hpp"
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <stdint.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/u_int8.h>
#include <zit6_interfaces/msg/zit_setpoint.h>

/**
 * @class MicroRosSubscriber
 * @brief 命令接收与分发模块
 *
 * 订阅话题：
 * - /zit6/cmd/setpoint — 控制设定点
 * - /zit6/cmd/agxhbt   — 上位机心跳（解锁）
 * - /zit6/cmd/ins      — 惯导指令
 * - /zit6/cmd/servo    — 舵机角度
 * - /zit6/cmd/light    — LED 灯控
 * - /zit6/sim/nav      — SITL 仿真导航状态（12 floats: 6 pos + 6 vel）
 */
class MicroRosSubscriber {
public:
  MicroRosSubscriber(auv::system::AppContext *ctx) : ctx_(ctx) {}

  /**
   * @brief 创建所有订阅并注册到 executor
   * @param node     rcl 节点指针
   * @param executor rclc executor 指针
   * @return true 全部成功
   */
  bool init(rcl_node_t *node, rclc_executor_t *executor);

  /**
   * @brief 销毁所有订阅
   * @param node rcl 节点指针
   */
  void cleanup(rcl_node_t *node);

private:
  // ---------- 订阅句柄 ----------
  rcl_subscription_t setpoint_sub_, arm_sub_, ins_cmd_sub_, servo_sub_,
      led_sub_, sim_nav_sub_;

  // ---------- 消息缓冲区 ----------
  zit6_interfaces__msg__ZitSetpoint setpoint_msg_;
  std_msgs__msg__UInt32 arm_msg_;
  std_msgs__msg__UInt8 ins_cmd_msg_;
  std_msgs__msg__Float32 servo_msg_;
  std_msgs__msg__UInt8 led_msg_;
  std_msgs__msg__Float32MultiArray sim_nav_msg_;

  // SITL 12 元素缓冲区（6 pos + 6 vel）
  float sim_nav_buf_[12] = {0};

  auv::system::AppContext *ctx_;

  // ---------- 实例指针（用于静态回调转发） ----------
  static MicroRosSubscriber *instance_;

  // ---------- 回调处理函数 ----------
  void onSetpoint(const void *msgin);
  void onArmHeartbeat(const void *msgin);
  void onInsCommand(const void *msgin);
  void onServoCmd(const void *msgin);
  void onLedCmd(const void *msgin);
  void onSimNav(const void *msgin);

  // ---------- 静态回调封装 ----------
  static void setpointCb(const void *msgin) {
    if (instance_)
      instance_->onSetpoint(msgin);
  }
  static void armCb(const void *msgin) {
    if (instance_)
      instance_->onArmHeartbeat(msgin);
  }
  static void insCmdCb(const void *msgin) {
    if (instance_)
      instance_->onInsCommand(msgin);
  }
  static void servoCb(const void *msgin) {
    if (instance_)
      instance_->onServoCmd(msgin);
  }
  static void ledCb(const void *msgin) {
    if (instance_)
      instance_->onLedCmd(msgin);
  }
  static void simNavCb(const void *msgin) {
    if (instance_)
      instance_->onSimNav(msgin);
  }
};

#endif // __MICROROS_SUBSCRIBER_HPP
