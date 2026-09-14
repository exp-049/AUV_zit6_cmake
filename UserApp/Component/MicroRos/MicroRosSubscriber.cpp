#include "MicroRosSubscriber.hpp"
#include "FreeRTOS.h"
#include "MotionContext.hpp"
#include "RosLogger.hpp"
#include "Pushrod_Driver.hpp"
#include "SystemConfig.hpp"
#include "SystemContext.hpp"
#include "main.h" // HAL_GetTick
#include "task.h"
#include <cmath>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>

// 实例指针定义
MicroRosSubscriber *MicroRosSubscriber::instance_ = nullptr;

// ============================================================================
// 初始化
// ============================================================================

bool MicroRosSubscriber::init(rcl_node_t *node, rclc_executor_t *executor) {
  instance_ = this;

  // init() may run again after an Agent reconnect. cleanup() owns the old
  // handles; these flags describe only the new connection.
  setpoint_sub_initialized_ = false;
  arm_sub_initialized_ = false;
  ins_cmd_sub_initialized_ = false;
  servo_sub_initialized_ = false;
  led_sub_initialized_ = false;
  sim_nav_sub_initialized_ = false;
  pushrod_sub_initialized_ = false;
  pushrod_sub_registered_ = false;

  // 初始化消息
  std_msgs__msg__UInt32__init(&arm_msg_);
  std_msgs__msg__UInt8__init(&ins_cmd_msg_);
  std_msgs__msg__UInt8__init(&led_msg_);
  std_msgs__msg__Float32__init(&servo_msg_);
  zit6_interfaces__msg__ZitSetpoint__init(&setpoint_msg_);
  zit6_interfaces__msg__ZitPushrod__init(&pushrod_msg_);

  std_msgs__msg__Float32MultiArray__init(&sim_nav_msg_);
  sim_nav_msg_.data.data = sim_nav_buf_;
  sim_nav_msg_.data.size = 12;
  sim_nav_msg_.data.capacity = 12;

  // 创建订阅。记录具体失败点；此前这里的错误被吞掉，导致上层只看到
  // publisher 不存在，无法判断是 Agent、接口还是 executor 出错。
  auto logFailure = [](const char *stage, rcl_ret_t ret) {
    ROS_LOG_ERROR("micro-ROS subscriber %s failed, rc=%d", stage,
                  static_cast<int>(ret));
    rcl_reset_error();
  };
  auto ok = [&](const char *stage, rcl_ret_t ret) {
    if (ret == RCL_RET_OK)
      return true;
    logFailure(stage, ret);
    return false;
  };

  rcl_ret_t rc = rclc_subscription_init_default(
      &led_sub_, node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
      "/zit6/cmd/light");
  if (!ok("/zit6/cmd/light init", rc))
    return false;
  led_sub_initialized_ = true;

  rc = rclc_subscription_init_default(
      &servo_sub_, node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/zit6/cmd/servo");
  if (!ok("/zit6/cmd/servo init", rc))
    return false;
  servo_sub_initialized_ = true;

  rc = rclc_subscription_init_default(
      &setpoint_sub_, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(zit6_interfaces, msg, ZitSetpoint),
      "/zit6/cmd/setpoint");
  if (!ok("/zit6/cmd/setpoint init", rc))
    return false;
  setpoint_sub_initialized_ = true;

  rc = rclc_subscription_init_default(
      &sim_nav_sub_, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "/zit6/sim/nav");
  if (!ok("/zit6/sim/nav init", rc))
    return false;
  sim_nav_sub_initialized_ = true;

  rc = rclc_subscription_init_default(
      &arm_sub_, node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
      "/zit6/cmd/agxhbt");
  if (!ok("/zit6/cmd/agxhbt init", rc))
    return false;
  arm_sub_initialized_ = true;

  rc = rclc_subscription_init_default(
      &ins_cmd_sub_, node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
      "/zit6/cmd/ins");
  if (!ok("/zit6/cmd/ins init", rc))
    return false;
  ins_cmd_sub_initialized_ = true;

  // 推杆消息是 7/22 固件之后加入的。它不能阻断核心状态发布，否则
  // Agent/接口包版本不一致时会同时丢失 /zit6/cmd/pushrod 和 /state/pos。
  // 若初始化成功，仍按正常路径注册；失败则保留错误码并让 NORMAL 继续。
  rc = rclc_subscription_init_default(
      &pushrod_sub_, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(zit6_interfaces, msg, ZitPushrod),
      "/zit6/cmd/pushrod");
  if (rc == RCL_RET_OK) {
    pushrod_sub_initialized_ = true;
  } else {
    logFailure("/zit6/cmd/pushrod init (optional)", rc);
  }

  auto add_subscription = [&](const char *stage,
                              rcl_subscription_t *subscription,
                              bool initialized, void *message,
                              rclc_subscription_callback_t callback) {
    if (!initialized)
      return false;
    return ok(stage, rclc_executor_add_subscription(
                         executor, subscription, message, callback,
                         ON_NEW_DATA));
  };
  if (!add_subscription("setpoint executor", &setpoint_sub_,
                        setpoint_sub_initialized_, &setpoint_msg_,
                        &MicroRosSubscriber::setpointCb) ||
      !add_subscription("arm executor", &arm_sub_, arm_sub_initialized_,
                        &arm_msg_, &MicroRosSubscriber::armCb) ||
      !add_subscription("ins executor", &ins_cmd_sub_,
                        ins_cmd_sub_initialized_, &ins_cmd_msg_,
                        &MicroRosSubscriber::insCmdCb) ||
      !add_subscription("servo executor", &servo_sub_, servo_sub_initialized_,
                        &servo_msg_, &MicroRosSubscriber::servoCb) ||
      !add_subscription("light executor", &led_sub_, led_sub_initialized_,
                        &led_msg_, &MicroRosSubscriber::ledCb) ||
      !add_subscription("sim_nav executor", &sim_nav_sub_,
                        sim_nav_sub_initialized_, &sim_nav_msg_,
                        &MicroRosSubscriber::simNavCb))
    return false;

  if (pushrod_sub_initialized_) {
    rc = rclc_executor_add_subscription(
        executor, &pushrod_sub_, &pushrod_msg_,
        &MicroRosSubscriber::pushrodCb, ON_NEW_DATA);
    if (rc == RCL_RET_OK) {
      pushrod_sub_registered_ = true;
    } else {
      logFailure("pushrod executor (optional)", rc);
    }
  }

  return true;
}

// ============================================================================
// 销毁
// ============================================================================

void MicroRosSubscriber::cleanup(rcl_node_t *node) {
  if (setpoint_sub_initialized_)
    rcl_subscription_fini(&setpoint_sub_, node);
  if (arm_sub_initialized_)
    rcl_subscription_fini(&arm_sub_, node);
  if (ins_cmd_sub_initialized_)
    rcl_subscription_fini(&ins_cmd_sub_, node);
  if (servo_sub_initialized_)
    rcl_subscription_fini(&servo_sub_, node);
  if (led_sub_initialized_)
    rcl_subscription_fini(&led_sub_, node);
  if (sim_nav_sub_initialized_)
    rcl_subscription_fini(&sim_nav_sub_, node);
  if (pushrod_sub_initialized_)
    rcl_subscription_fini(&pushrod_sub_, node);

  setpoint_sub_initialized_ = false;
  arm_sub_initialized_ = false;
  ins_cmd_sub_initialized_ = false;
  servo_sub_initialized_ = false;
  led_sub_initialized_ = false;
  sim_nav_sub_initialized_ = false;
  pushrod_sub_initialized_ = false;
  pushrod_sub_registered_ = false;
  if (instance_ == this)
    instance_ = nullptr;
}

void MicroRosSubscriber::update(uint32_t now_ms) { updatePushrod(now_ms); }

// ============================================================================
// 回调实现
// ============================================================================

void MicroRosSubscriber::onSetpoint(const void *msgin) {
  const auto *msg = (const zit6_interfaces__msg__ZitSetpoint *)msgin;
  auv::motion::motion_context.last_received_seq_.set(msg->seq);
  // All six setpoint fields are forwarded through the control chain. Roll/Pitch
  // are bypassed only at the MotionController_Driver/VIT6 packet boundary.
  if (!std::isfinite(msg->x) || !std::isfinite(msg->y) ||
      !std::isfinite(msg->z) || !std::isfinite(msg->roll) ||
      !std::isfinite(msg->pitch) || !std::isfinite(msg->yaw))
    return;
  if (!auv::system::system_context.arm_state_.get().is_armed)
    return;

  uint32_t level_idx = msg->control_key & 0x03;
  if (level_idx >= 3)
    return;

  auv::motion::ControlLevel new_level;
  switch (level_idx) {
  case 0:
    new_level = auv::motion::ControlLevel::POSITION;
    break;
  case 1:
    new_level = auv::motion::ControlLevel::VELOCITY;
    break;
  case 2:
    new_level = auv::motion::ControlLevel::ACTUATOR;
    break;
  default:
    return;
  }

  bool is_body = (msg->control_key & 0x10) != 0;
  bool is_inc = (msg->control_key & 0x20) != 0;
  uint32_t mask = msg->type_mask;
  float val[6] = {msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw};

  bool sim_mode = auv::config::sys_config.simulation.hitl_enabled ||
                  auv::config::sys_config.simulation.sitl_enabled;
  bool nav_valid = auv::system::system_context.getNavigationValid() || sim_mode;
  if ((new_level == auv::motion::ControlLevel::POSITION ||
       new_level == auv::motion::ControlLevel::VELOCITY) &&
      !nav_valid)
    return;

  taskENTER_CRITICAL();
  ctx_->chassis->updateSetpoint(new_level, val, mask, is_body, is_inc);
  taskEXIT_CRITICAL();

  ROS_LOG_INFO("Setpoint rec: seq=%lu x=%.2f y=%.2f z=%.2f "
               "roll=%.2f pitch=%.2f yaw=%.2f "
               "(roll/pitch bypassed at driver)",
               (unsigned long)msg->seq, msg->x, msg->y, msg->z, msg->roll,
               msg->pitch, msg->yaw);
}

void MicroRosSubscriber::onArmHeartbeat(const void *msgin) {
  const auto *msg = (const std_msgs__msg__UInt32 *)msgin;
  taskENTER_CRITICAL();
  auto a = auv::system::system_context.arm_state_.get();
  a.last_heartbeat_ms = HAL_GetTick();
  a.last_heartbeat_data = msg->data;
  if (!a.is_armed) {
    if (a.heartbeat_count == 0)
      a.start_ms = a.last_heartbeat_ms;
    a.heartbeat_count++;
  }
  auv::system::system_context.arm_state_.set(a);
  taskEXIT_CRITICAL();
}

void MicroRosSubscriber::onInsCommand(const void *msgin) {
  const auto *message = static_cast<const std_msgs__msg__UInt8 *>(msgin);
  if (message == nullptr)
    return;
  switch (message->data) {
  case 1: {
    const bool sent = ctx_->ins_driver->setDvlPower(true);
    ROS_LOG_INFO("INS Cmd: DVL Power ON, tx=%s", sent ? "ok" : "failed");
    break;
  }
  case 2: {
    const bool sent = ctx_->ins_driver->setDvlPower(false);
    ROS_LOG_INFO("INS Cmd: DVL Power OFF, tx=%s", sent ? "ok" : "failed");
    break;
  }
  case 3: {
    const bool sent = ctx_->ins_driver->restart();
    ROS_LOG_INFO("INS Cmd: INS Restart, tx=%s", sent ? "ok" : "failed");
    break;
  }
  case 4: {
    const bool sent = ctx_->ins_driver->resetPosition();
    ROS_LOG_INFO("INS Cmd: Reset Position, tx=%s", sent ? "ok" : "failed");
    break;
  }
  case 5: {
    const bool sent = ctx_->ins_driver->setInitialPosition(
        auv::config::sys_config.ins.init_lat,
        auv::config::sys_config.ins.init_lon);
    ROS_LOG_INFO("INS Cmd: Set Initial Position, tx=%s",
                 sent ? "ok" : "failed");
    break;
  }
  default:
    ROS_LOG_WARN("INS Cmd: Unknown cmd %d", message->data);
    break;
  }
}

void MicroRosSubscriber::onServoCmd(const void *msgin) {
  const auto *msg = (const std_msgs__msg__Float32 *)msgin;
  ctx_->motor_driver->setServoAngle(msg->data);
  ROS_LOG_INFO("Servo Cmd: angle=%.2f", msg->data);
}

void MicroRosSubscriber::onLedCmd(const void *msgin) {
  const auto *msg = (const std_msgs__msg__UInt8 *)msgin;
  ctx_->motor_driver->setLightState(msg->data);
  ROS_LOG_INFO("LED Cmd: state=%d", msg->data);
}

void MicroRosSubscriber::onSimNav(const void *msgin) {
  const auto *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  if (msg->data.size < 12)
    return;
  auv::motion::NavState state;
  for (int i = 0; i < 6; ++i) {
    state.pos_world[i] = msg->data.data[i];
    state.vel_body[i] = msg->data.data[i + 6];
  }
  /* 推入队列，非阻塞；队列满则丢弃旧帧（FIFO，消费者 drain 到最新） */
  xQueueSend(auv::motion::motion_context.sitl_nav_queue, &state, 0);
}

void MicroRosSubscriber::onPushrodCmd(const void *msgin) {
  const auto *msg = static_cast<const zit6_interfaces__msg__ZitPushrod *>(msgin);
  if (msg == nullptr)
    return;

  if (!std::isfinite(msg->speed) || msg->speed < -1.0f ||
      msg->speed > 1.0f) {
    ROS_LOG_WARN("Pushrod Cmd: speed must be finite and in [-1, 1]");
    return;
  }
  if (msg->duration_ms == 0U) {
    ROS_LOG_WARN("Pushrod Cmd: duration_ms must be greater than zero");
    return;
  }
  if (!auv::system::system_context.arm_state_.get().is_armed) {
    ROS_LOG_WARN("Pushrod Cmd: rejected while disarmed");
    return;
  }
  if (pushrod_queue_count_ >= kPushrodQueueCapacity) {
    ROS_LOG_WARN("Pushrod Cmd: local queue full");
    return;
  }

  const long scaled_power = std::lround(msg->speed * 1000.0f);
  PushrodCommand &command = pushrod_queue_[pushrod_queue_tail_];
  command.power_x1000 = static_cast<int16_t>(scaled_power);
  command.duration_ms = msg->duration_ms;
  pushrod_queue_tail_ =
      static_cast<uint8_t>((pushrod_queue_tail_ + 1U) % kPushrodQueueCapacity);
  ++pushrod_queue_count_;

  ROS_LOG_INFO("Pushrod Cmd: speed=%.3f power=%ld duration=%lu ms",
               msg->speed, scaled_power, (unsigned long)msg->duration_ms);
  updatePushrod(HAL_GetTick());
}

void MicroRosSubscriber::popPushrodCommand() {
  if (pushrod_queue_count_ == 0U)
    return;
  pushrod_queue_head_ = static_cast<uint8_t>(
      (pushrod_queue_head_ + 1U) % kPushrodQueueCapacity);
  --pushrod_queue_count_;
}

void MicroRosSubscriber::sendOrRetryPushrod(uint32_t now_ms) {
  if (ctx_ == nullptr || ctx_->pushrod_driver == nullptr ||
      !pushrod_pending_)
    return;

  if (ctx_->pushrod_driver->sendTask(pushrod_pending_task_)) {
    pushrod_last_send_ms_ = now_ms;
    ROS_LOG_INFO("Pushrod task sent: id=%lu power=%d duration=%lu ms",
                 (unsigned long)pushrod_pending_task_.task_id,
                 pushrod_pending_task_.power_x1000,
                 (unsigned long)pushrod_pending_task_.duration_ms);
  } else {
    // Keep the same task_id. The next update will retry without advancing it.
    pushrod_last_send_ms_ = now_ms;
    ROS_LOG_WARN("Pushrod task transmit failed: id=%lu",
                 (unsigned long)pushrod_pending_task_.task_id);
  }
}

void MicroRosSubscriber::updatePushrod(uint32_t now_ms) {
  if (ctx_ == nullptr || ctx_->pushrod_driver == nullptr)
    return;

  const bool armed = auv::system::system_context.arm_state_.get().is_armed;

  // Local GPIO backends use this hook for duration expiry. UART backends
  // intentionally implement it as a no-op because Depth_Sensor_Driver owns
  // the shared UART4 polling path.
  ctx_->pushrod_driver->poll(now_ms);
  if (!armed) {
    // For the GPIO backend this immediately de-energizes PB7/PB8. The shared
    // UART backend has no cancel frame and keeps its existing no-op behavior.
    ctx_->pushrod_driver->stop();
  }

  auv::peripheral::PushrodAck ack{};
  while (ctx_->pushrod_driver->readAck(&ack)) {
    if (!pushrod_pending_ || ack.task_id != pushrod_pending_task_.task_id) {
      ROS_LOG_WARN("Pushrod ACK ignored: id=%lu result=%u",
                   (unsigned long)ack.task_id, (unsigned)ack.result);
      continue;
    }

    if (ack.result == auv::peripheral::pushrod::kOk) {
      const uint32_t completed_id = pushrod_pending_task_.task_id;
      pushrod_pending_ = false;
      popPushrodCommand();
      pushrod_next_task_id_ = completed_id + 1U;
      ROS_LOG_INFO("Pushrod task accepted: id=%lu queue=%u ready=%u",
                   (unsigned long)completed_id, (unsigned)ack.queue_count,
                   (unsigned)ack.ready);
    } else if (ack.result == auv::peripheral::pushrod::kQueueFull ||
               ack.result == auv::peripheral::pushrod::kNotInitialized) {
      // The task was not accepted. Retain the same task ID and retry later.
      pushrod_last_send_ms_ = now_ms;
      ROS_LOG_WARN("Pushrod task deferred: id=%lu result=%u queue=%u",
                   (unsigned long)ack.task_id, (unsigned)ack.result,
                   (unsigned)ack.queue_count);
    } else {
      // The task was not accepted. Drop only this command, but do not advance
      // the ID; the next valid task may reuse it according to the protocol.
      ROS_LOG_WARN("Pushrod task rejected: id=%lu result=%u",
                   (unsigned long)ack.task_id, (unsigned)ack.result);
      pushrod_pending_ = false;
      popPushrodCommand();
    }
  }

  // Do not transmit queued commands after disarm. For the V1 UART protocol an
  // already transmitted task is left intact because there is no cancel frame;
  // the GPIO backend has already been forced to its safe 00 output above.
  if (!armed) {
    if (!pushrod_pending_) {
      pushrod_queue_head_ = 0U;
      pushrod_queue_tail_ = 0U;
      pushrod_queue_count_ = 0U;
    }
    return;
  }

  if (!pushrod_pending_ && pushrod_queue_count_ > 0U) {
    const PushrodCommand &command = pushrod_queue_[pushrod_queue_head_];
    pushrod_pending_task_.task_id = pushrod_next_task_id_;
    pushrod_pending_task_.power_x1000 = command.power_x1000;
    pushrod_pending_task_.duration_ms = command.duration_ms;
    pushrod_pending_ = true;
    sendOrRetryPushrod(now_ms);
    return;
  }

  if (pushrod_pending_ &&
      (uint32_t)(now_ms - pushrod_last_send_ms_) >=
          kPushrodAckTimeoutMs) {
    sendOrRetryPushrod(now_ms);
  }
}
