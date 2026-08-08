#include "MicroRosPublisher.hpp"
#include "FreeRTOS.h"
#include "MotionContext.hpp"
#include "RosLogger.hpp"
#include "SystemContext.hpp"
#include "USBL_Driver.hpp"
#include "task.h"
#include <cstring>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>

bool MicroRosPublisher::init(rcl_node_t *node) {
  pos_pub_initialized_ = false;
  vel_pub_initialized_ = false;
  thr_pub_initialized_ = false;
  zithbt_pub_initialized_ = false;
  status_pub_initialized_ = false;
  usbl_pub_initialized_ = false;
  log_pub_initialized_ = false;

  // 初始化零初始化消息
  std_msgs__msg__Float32MultiArray__init(&pos_fb_msg_);
  pos_fb_msg_.data.data = pos_buf_;
  pos_fb_msg_.data.size = 6;
  pos_fb_msg_.data.capacity = 6;

  std_msgs__msg__Float32MultiArray__init(&vel_fb_msg_);
  vel_fb_msg_.data.data = vel_buf_;
  vel_fb_msg_.data.size = 6;
  vel_fb_msg_.data.capacity = 6;

  std_msgs__msg__Float32MultiArray__init(&thr_fb_msg_);
  thr_fb_msg_.data.data = thr_buf_;
  thr_fb_msg_.data.size = 6;
  thr_fb_msg_.data.capacity = 6;

  std_msgs__msg__UInt32__init(&node_heartbeat_msg_);
  zit6_interfaces__msg__ZitStatus__init(&status_msg_);
  zit6_interfaces__msg__ZitUsbl__init(&usbl_msg_);

  rcl_interfaces__msg__Log__init(&log_msg_);
  log_msg_.msg.data = log_msg_buf_;
  log_msg_.msg.capacity = sizeof(log_msg_buf_);
  log_msg_.msg.size = 0;
  log_msg_.name.data = const_cast<char *>("zit6_node");
  log_msg_.name.size = 9;
  log_msg_.name.capacity = 0;
  log_msg_.file.data = const_cast<char *>("");
  log_msg_.file.size = 0;
  log_msg_.file.capacity = 0;
  log_msg_.function.data = const_cast<char *>("");
  log_msg_.function.size = 0;
  log_msg_.function.capacity = 0;
  log_msg_.line = 0;

  // 创建发布器
  auto ok = [&](const char *stage, rcl_ret_t ret) {
    if (ret == RCL_RET_OK)
      return true;
    ROS_LOG_ERROR("micro-ROS publisher %s failed, rc=%d", stage,
                  static_cast<int>(ret));
    rcl_reset_error();
    return false;
  };

  if (!ok("/zit6/state/pos init", rclc_publisher_init_default(
          &pos_pub_, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
          "/zit6/state/pos")))
    return false;
  pos_pub_initialized_ = true;
  if (!ok("/zit6/state/vel init", rclc_publisher_init_default(
          &vel_pub_, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
          "/zit6/state/vel")))
    return false;
  vel_pub_initialized_ = true;
  if (!ok("/zit6/state/thr init", rclc_publisher_init_default(
          &thr_pub_, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
          "/zit6/state/thr")))
    return false;
  thr_pub_initialized_ = true;
  if (!ok("/zit6/state/zithbt init", rclc_publisher_init_default(
          &zithbt_pub_, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
          "/zit6/state/zithbt")))
    return false;
  zithbt_pub_initialized_ = true;
  if (!ok("/zit6/state/status init", rclc_publisher_init_default(
          &status_pub_, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(zit6_interfaces, msg, ZitStatus),
          "/zit6/state/status")))
    return false;
  status_pub_initialized_ = true;
  if (!ok("/zit6/state/USBL init", rclc_publisher_init_default(
          &usbl_pub_, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(zit6_interfaces, msg, ZitUsbl),
          "/zit6/state/USBL")))
    return false;
  usbl_pub_initialized_ = true;
  if (!ok("/zit6/log init", rclc_publisher_init_default(
          &log_pub_, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log), "/zit6/log")))
    return false;
  log_pub_initialized_ = true;

  return true;
}

void MicroRosPublisher::publish(uint32_t now_ms) {
  // 1. 日志发布（事件驱动，每次 pop 一条）
  auv::component::RosLogger::LogEntry log_entry;
  if (ctx_->logger->popLog(log_entry)) {
    std::strncpy(log_msg_.msg.data, log_entry.msg, log_msg_.msg.capacity - 1);
    log_msg_.msg.data[log_msg_.msg.capacity - 1] = '\0';
    log_msg_.msg.size = std::strlen(log_msg_.msg.data);
    log_msg_.level = log_entry.level;
    log_msg_.stamp.sec = now_ms / 1000;
    log_msg_.stamp.nanosec = (now_ms % 1000) * 1000000;
    rcl_publish(&log_pub_, &log_msg_, NULL);
  }

  // 2. USBL 有效帧：按 ControlTask 入队顺序 FIFO 发布。
  auv::motion::UsblTopicSample usbl_sample{};
  while (auv::motion::motion_context.usbl_topic_queue != nullptr &&
         xQueueReceive(auv::motion::motion_context.usbl_topic_queue,
                       &usbl_sample, 0) == pdPASS) {
    const auto &state = usbl_sample.state;
    usbl_msg_.timestamp_ms = state.timestamp;
    usbl_msg_.frame_number = usbl_sample.frame_number;
    usbl_msg_.roll_deg = state.roll;
    usbl_msg_.pitch_deg = state.pitch;
    usbl_msg_.yaw_deg = state.yaw;
    usbl_msg_.pressure_m = state.pressure;
    std::memcpy(usbl_msg_.slant_range_m, state.slant_range,
                sizeof(state.slant_range));
    usbl_msg_.latitude_deg = state.latitude;
    usbl_msg_.longitude_deg = state.longitude;
    std::memcpy(usbl_msg_.time_diff, state.time_diff, sizeof(state.time_diff));
    std::memcpy(usbl_msg_.passive_attitude, state.passive_attitude,
                sizeof(state.passive_attitude));
    std::memcpy(usbl_msg_.signal_strength, state.signal_strength,
                sizeof(state.signal_strength));
    std::memcpy(usbl_msg_.energy, state.energy, sizeof(state.energy));
    usbl_msg_.signal = state.signal;
    usbl_msg_.gain = state.gain;
    usbl_msg_.beacon_north_m = state.beacon_north;
    usbl_msg_.beacon_east_m = state.beacon_east;
    usbl_msg_.beacon_depth_m = state.beacon_depth;
    usbl_msg_.sensor_status = state.sensor_status;
    usbl_msg_.year = state.year;
    usbl_msg_.month = state.month;
    usbl_msg_.day = state.day;
    usbl_msg_.hour = state.hour;
    usbl_msg_.minute = state.minute;
    usbl_msg_.second = state.second;
    usbl_msg_.navigation_status = state.nav_mode;
    usbl_msg_.checksum = state.checksum;
    (void)rcl_publish(&usbl_pub_, &usbl_msg_, NULL);
  }

  // 3. 心跳（1Hz）
  if (now_ms - last_hbt_pub_tick_ >= 1000) {
    last_hbt_pub_tick_ = now_ms;
    node_heartbeat_msg_.data = now_ms;
    rcl_publish(&zithbt_pub_, &node_heartbeat_msg_, NULL);
  }

  // 4. 速度反馈（~50Hz）
  if (now_ms - last_vel_pub_tick_ >= 20) {
    last_vel_pub_tick_ = now_ms;
    auto nav = auv::motion::motion_context.nav_state_.get();
    vel_buf_[0] = nav.vel_body[0];
    vel_buf_[1] = nav.vel_body[1];
    vel_buf_[2] = nav.vel_body[2];
    vel_buf_[3] = nav.vel_body[3];
    vel_buf_[4] = nav.vel_body[4];
    vel_buf_[5] = nav.vel_body[5];
    rcl_publish(&vel_pub_, &vel_fb_msg_, NULL);
  }

  // 5. 推力反馈（~30Hz）
  if (now_ms - last_thr_pub_tick_ >= 33) {
    last_thr_pub_tick_ = now_ms;
    auto forces = auv::motion::motion_context.last_output_forces_.get();
    thr_buf_[0] = forces[0];
    thr_buf_[1] = forces[1];
    thr_buf_[2] = forces[2];
    thr_buf_[3] = forces[3];
    thr_buf_[4] = forces[4];
    thr_buf_[5] = forces[5];
    rcl_publish(&thr_pub_, &thr_fb_msg_, NULL);
  }

  // 6. 位置反馈（~30Hz）
  if (now_ms - last_pos_pub_tick_ >= 33) {
    last_pos_pub_tick_ = now_ms;
    auto nav = auv::motion::motion_context.nav_state_.get();
    pos_buf_[0] = nav.pos_world[0];
    pos_buf_[1] = nav.pos_world[1];
    pos_buf_[2] = nav.pos_world[2];
    pos_buf_[3] = nav.pos_world[3];
    pos_buf_[4] = nav.pos_world[4];
    pos_buf_[5] = nav.pos_world[5];
    rcl_publish(&pos_pub_, &pos_fb_msg_, NULL);
  }

  // 7. 状态汇总（10Hz）
  if (now_ms - last_status_pub_tick_ >= 100) {
    last_status_pub_tick_ = now_ms;
    auto forces = auv::motion::motion_context.last_output_forces_.get();
    const float cycle_time = auv::motion::motion_context.last_dt_ms_.get();
    const float exec_time = auv::motion::motion_context.last_exec_ms_.get();
    const uint32_t overrun_count =
        auv::motion::motion_context.control_overrun_count_.get();
    const uint32_t thrust_tx_fail_count =
        auv::motion::motion_context.thrust_tx_fail_count_.get();

    auto arm = auv::system::system_context.arm_state_.get();
    auto nav = auv::system::system_context.nav_status_.get();
    bool nav_valid = auv::system::system_context.getNavigationValid();

    status_msg_.is_armed = arm.is_armed;
    status_msg_.arm_mode = (uint8_t)arm.last_heartbeat_data;
    status_msg_.control_level = (uint8_t)ctx_->chassis->getControlLevel();
    status_msg_.ins_state = nav.imu_state;
    status_msg_.navigation_ready = nav_valid;
    for (int i = 0; i < 6; ++i)
      status_msg_.forces[i] = forces[i];
    status_msg_.cycle_time_ms = cycle_time;
    status_msg_.battery_voltage = 0.0f;
    status_msg_.error_flags = 0;
    status_msg_.control_overrun_count = overrun_count;
    status_msg_.thrust_tx_fail_count = thrust_tx_fail_count;
    (void)exec_time;

    rcl_publish(&status_pub_, &status_msg_, NULL);
  }
}

void MicroRosPublisher::cleanup(rcl_node_t *node) {
  if (pos_pub_initialized_)
    rcl_publisher_fini(&pos_pub_, node);
  if (vel_pub_initialized_)
    rcl_publisher_fini(&vel_pub_, node);
  if (thr_pub_initialized_)
    rcl_publisher_fini(&thr_pub_, node);
  if (zithbt_pub_initialized_)
    rcl_publisher_fini(&zithbt_pub_, node);
  if (status_pub_initialized_)
    rcl_publisher_fini(&status_pub_, node);
  if (usbl_pub_initialized_)
    rcl_publisher_fini(&usbl_pub_, node);
  if (log_pub_initialized_)
    rcl_publisher_fini(&log_pub_, node);

  pos_pub_initialized_ = false;
  vel_pub_initialized_ = false;
  thr_pub_initialized_ = false;
  zithbt_pub_initialized_ = false;
  status_pub_initialized_ = false;
  usbl_pub_initialized_ = false;
  log_pub_initialized_ = false;

  // 重置节流定时器
  last_hbt_pub_tick_ = 0;
  last_vel_pub_tick_ = 0;
  last_thr_pub_tick_ = 0;
  last_pos_pub_tick_ = 0;
  last_status_pub_tick_ = 0;
}
