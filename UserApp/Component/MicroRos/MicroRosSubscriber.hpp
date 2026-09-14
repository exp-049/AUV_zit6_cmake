#ifndef __MICROROS_SUBSCRIBER_HPP
#define __MICROROS_SUBSCRIBER_HPP

#include "AppContext.hpp"
#include "Pushrod_Backend.hpp"
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <stdint.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/u_int8.h>
#include <zit6_interfaces/msg/zit_pushrod.h>
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
 * - /zit6/cmd/pushrod  — 推杆任务（speed: -1.0～1.0，duration_ms: > 0）
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

  /**
   * @brief 驱动推杆任务发送、ACK 处理和超时重发
   *
   * 在 micro-ROS 任务上下文中周期调用，不阻塞等待深度解算板响应。
   */
  void update(uint32_t now_ms);

private:
  // ---------- 订阅句柄 ----------
  rcl_subscription_t setpoint_sub_{};
  rcl_subscription_t arm_sub_{};
  rcl_subscription_t ins_cmd_sub_{};
  rcl_subscription_t servo_sub_{};
  rcl_subscription_t led_sub_{};
  rcl_subscription_t sim_nav_sub_{};
  rcl_subscription_t pushrod_sub_{};

  // rclc may fail after only part of the subscription set was created. Keep
  // explicit ownership bits so reconnect cleanup never finalizes garbage.
  bool setpoint_sub_initialized_ = false;
  bool arm_sub_initialized_ = false;
  bool ins_cmd_sub_initialized_ = false;
  bool servo_sub_initialized_ = false;
  bool led_sub_initialized_ = false;
  bool sim_nav_sub_initialized_ = false;
  bool pushrod_sub_initialized_ = false;
  bool pushrod_sub_registered_ = false;

  // ---------- 消息缓冲区 ----------
  zit6_interfaces__msg__ZitSetpoint setpoint_msg_;
  std_msgs__msg__UInt32 arm_msg_;
  std_msgs__msg__UInt8 ins_cmd_msg_;
  std_msgs__msg__Float32 servo_msg_;
  std_msgs__msg__UInt8 led_msg_;
  std_msgs__msg__Float32MultiArray sim_nav_msg_;
  zit6_interfaces__msg__ZitPushrod pushrod_msg_;

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
  void onPushrodCmd(const void *msgin);

  struct PushrodCommand {
    int16_t power_x1000;
    uint32_t duration_ms;
  };

  static constexpr uint8_t kPushrodQueueCapacity = 4U;
  static constexpr uint32_t kPushrodAckTimeoutMs = 100U;

  PushrodCommand pushrod_queue_[kPushrodQueueCapacity] = {};
  uint8_t pushrod_queue_head_ = 0U;
  uint8_t pushrod_queue_tail_ = 0U;
  uint8_t pushrod_queue_count_ = 0U;
  uint32_t pushrod_next_task_id_ = 0U;
  auv::peripheral::PushrodTask pushrod_pending_task_{};
  bool pushrod_pending_ = false;
  uint32_t pushrod_last_send_ms_ = 0U;

  void updatePushrod(uint32_t now_ms);
  void sendOrRetryPushrod(uint32_t now_ms);
  void popPushrodCommand();

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
  static void pushrodCb(const void *msgin) {
    if (instance_)
      instance_->onPushrodCmd(msgin);
  }
};

#endif // __MICROROS_SUBSCRIBER_HPP
