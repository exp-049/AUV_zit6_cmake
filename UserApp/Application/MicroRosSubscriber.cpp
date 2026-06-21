#include "MicroRosSubscriber.hpp"
#include "ChassisManager.hpp"
#include "FreeRTOS.h"
#include "INS_Driver.hpp"
#include "MotionContext.hpp"
#include "MotionController_Driver.hpp"
#include "RosLogger.hpp"
#include "SystemConfig.hpp"
#include "SystemContext.hpp"
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

  // 初始化消息
  std_msgs__msg__UInt32__init(&arm_msg_);
  std_msgs__msg__UInt8__init(&ins_cmd_msg_);
  std_msgs__msg__UInt8__init(&led_msg_);
  std_msgs__msg__Float32__init(&servo_msg_);
  zit6_interfaces__msg__ZitSetpoint__init(&setpoint_msg_);

  std_msgs__msg__Float32MultiArray__init(&sim_pos_msg_);
  sim_pos_msg_.data.data = sim_pos_buf_;
  sim_pos_msg_.data.size = 6;
  sim_pos_msg_.data.capacity = 6;

  std_msgs__msg__Float32MultiArray__init(&sim_vel_msg_);
  sim_vel_msg_.data.data = sim_vel_buf_;
  sim_vel_msg_.data.size = 6;
  sim_vel_msg_.data.capacity = 6;

  // 创建订阅
  auto ok = [](rcl_ret_t ret) { return ret == RCL_RET_OK; };

  if (!ok(rclc_subscription_init_default(
          &led_sub_, node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
          "/zit6/cmd/light")))
    return false;
  if (!ok(rclc_subscription_init_default(
          &servo_sub_, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
          "/zit6/cmd/servo")))
    return false;
  if (!ok(rclc_subscription_init_default(
          &setpoint_sub_, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(zit6_interfaces, msg, ZitSetpoint),
          "/zit6/cmd/setpoint")))
    return false;
  if (!ok(rclc_subscription_init_default(
          &sim_pos_sub_, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
          "/zit6/sim/pos")))
    return false;
  if (!ok(rclc_subscription_init_default(
          &sim_vel_sub_, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
          "/zit6/sim/vel")))
    return false;
  if (!ok(rclc_subscription_init_default(
          &arm_sub_, node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
          "/zit6/cmd/agxhbt")))
    return false;
  if (!ok(rclc_subscription_init_default(
          &ins_cmd_sub_, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "/zit6/cmd/ins")))
    return false;

  // 注册到 executor
  rclc_executor_add_subscription(executor, &setpoint_sub_, &setpoint_msg_,
                                 &MicroRosSubscriber::setpointCb, ON_NEW_DATA);
  rclc_executor_add_subscription(executor, &arm_sub_, &arm_msg_,
                                 &MicroRosSubscriber::armCb, ON_NEW_DATA);
  rclc_executor_add_subscription(executor, &ins_cmd_sub_, &ins_cmd_msg_,
                                 &MicroRosSubscriber::insCmdCb, ON_NEW_DATA);
  rclc_executor_add_subscription(executor, &servo_sub_, &servo_msg_,
                                 &MicroRosSubscriber::servoCb, ON_NEW_DATA);
  rclc_executor_add_subscription(executor, &led_sub_, &led_msg_,
                                 &MicroRosSubscriber::ledCb, ON_NEW_DATA);
  rclc_executor_add_subscription(executor, &sim_pos_sub_, &sim_pos_msg_,
                                 &MicroRosSubscriber::simPosCb, ON_NEW_DATA);
  rclc_executor_add_subscription(executor, &sim_vel_sub_, &sim_vel_msg_,
                                 &MicroRosSubscriber::simVelCb, ON_NEW_DATA);

  return true;
}

// ============================================================================
// 销毁
// ============================================================================

void MicroRosSubscriber::cleanup(rcl_node_t *node) {
  rcl_subscription_fini(&setpoint_sub_, node);
  rcl_subscription_fini(&arm_sub_, node);
  rcl_subscription_fini(&ins_cmd_sub_, node);
  rcl_subscription_fini(&servo_sub_, node);
  rcl_subscription_fini(&led_sub_, node);
  rcl_subscription_fini(&sim_pos_sub_, node);
  rcl_subscription_fini(&sim_vel_sub_, node);
}

// ============================================================================
// 回调实现
// ============================================================================

void MicroRosSubscriber::onSetpoint(const void *msgin) {
  const auto *msg = (const zit6_interfaces__msg__ZitSetpoint *)msgin;
  auv::motion::motion_context.setLastReceivedSeq(msg->seq);
  if (!std::isfinite(msg->x) || !std::isfinite(msg->y) ||
      !std::isfinite(msg->z) || !std::isfinite(msg->yaw))
    return;
  if (!auv::system::system_context.is_system_armed)
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
  float val[4] = {msg->x, msg->y, msg->z, msg->yaw};

  bool sim_mode = auv::config::sys_config.simulation.hitl_enabled ||
                  auv::config::sys_config.simulation.sitl_enabled;
  bool nav_valid = auv::system::system_context.getNavigationValid() || sim_mode;
  if ((new_level == auv::motion::ControlLevel::POSITION ||
       new_level == auv::motion::ControlLevel::VELOCITY) &&
      !nav_valid)
    return;

  taskENTER_CRITICAL();

  // 1. 记录原始 AGX 设定值快照
  auv::motion::RawSetpoint raw_sp;
  raw_sp.level = new_level;
  raw_sp.data[0] = val[0];
  raw_sp.data[1] = val[1];
  raw_sp.data[2] = val[2];
  raw_sp.data[3] = val[3];
  raw_sp.type_mask = mask;
  raw_sp.is_body = is_body;
  raw_sp.is_incremental = is_inc;
  auv::motion::motion_context.setRawSetpoint(raw_sp);

  // 2. 更新底盘目标设定值并切换控制层级
  auv::control::chassis.updateSetpoint(new_level, val, mask, is_body, is_inc);
  taskEXIT_CRITICAL();

  ROS_LOG_INFO("Setpoint rec: seq=%lu x=%.2f y=%.2f z=%.2f yaw=%.2f",
               (unsigned long)msg->seq, msg->x, msg->y, msg->z, msg->yaw);
}

void MicroRosSubscriber::onArmHeartbeat(const void *msgin) {
  const auto *msg = (const std_msgs__msg__UInt32 *)msgin;
  taskENTER_CRITICAL();
  auv::system::system_context.last_arm_heartbeat_ms = HAL_GetTick();
  auv::system::system_context.last_arm_heartbeat_data = msg->data;
  if (!auv::system::system_context.is_system_armed) {
    if (auv::system::system_context.arm_heartbeat_count == 0)
      auv::system::system_context.arm_start_ms =
          auv::system::system_context.last_arm_heartbeat_ms;
    auv::system::system_context.arm_heartbeat_count++;
  }
  taskEXIT_CRITICAL();
}

void MicroRosSubscriber::onInsCommand(const void *msgin) {
  const auto *message = static_cast<const std_msgs__msg__UInt8 *>(msgin);
  if (message == nullptr)
    return;
  switch (message->data) {
  case 1:
    auv::device::ins_driver.setDvlPower(true);
    ROS_LOG_INFO("INS Cmd: DVL Power ON");
    break;
  case 2:
    auv::device::ins_driver.setDvlPower(false);
    ROS_LOG_INFO("INS Cmd: DVL Power OFF");
    break;
  case 3:
    auv::device::ins_driver.restart();
    ROS_LOG_INFO("INS Cmd: INS Restart");
    break;
  case 4:
    auv::device::ins_driver.resetPosition();
    ROS_LOG_INFO("INS Cmd: Reset Position");
    break;
  case 5:
    auv::device::ins_driver.setInitialPosition(
        auv::config::sys_config.ins.init_lat,
        auv::config::sys_config.ins.init_lon);
    ROS_LOG_INFO("INS Cmd: Set Initial Position");
    break;
  default:
    ROS_LOG_WARN("INS Cmd: Unknown cmd %d", message->data);
    break;
  }
}

void MicroRosSubscriber::onServoCmd(const void *msgin) {
  const auto *msg = (const std_msgs__msg__Float32 *)msgin;
  auv::device::motor_driver.setServoAngle(msg->data);
  ROS_LOG_INFO("Servo Cmd: angle=%.2f", msg->data);
}

void MicroRosSubscriber::onLedCmd(const void *msgin) {
  const auto *msg = (const std_msgs__msg__UInt8 *)msgin;
  auv::device::motor_driver.setLightState(msg->data);
  ROS_LOG_INFO("LED Cmd: state=%d", msg->data);
}

void MicroRosSubscriber::onSimPos(const void *msgin) {
  const auto *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  if (msg->data.size < 6)
    return;

  auv::motion::NavState state;
  for (int i = 0; i < 6; ++i) {
    state.pos_world[i] = msg->data.data[i];
  }
  auto cur = auv::motion::motion_context.getSimNavState();
  for (int i = 0; i < 6; ++i) {
    state.vel_body[i] = cur.vel_body[i];
  }
  auv::motion::motion_context.setSimNavState(state);
}

void MicroRosSubscriber::onSimVel(const void *msgin) {
  const auto *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  if (msg->data.size < 6)
    return;

  auv::motion::NavState state;
  for (int i = 0; i < 6; ++i) {
    state.vel_body[i] = msg->data.data[i];
  }
  auto cur = auv::motion::motion_context.getSimNavState();
  for (int i = 0; i < 6; ++i) {
    state.pos_world[i] = cur.pos_world[i];
  }
  auv::motion::motion_context.setSimNavState(state);
}
