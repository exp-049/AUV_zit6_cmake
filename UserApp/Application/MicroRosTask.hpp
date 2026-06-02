#ifndef __MICROROS_TASK_HPP
#define __MICROROS_TASK_HPP

#include "FreeRTOS.h"
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rcutils/allocator.h>
#include <stdint.h>

#include "SystemConfig.hpp"
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/u_int8.h>
#include <zit6_interfaces/msg/zit_setpoint.h>
#include <zit6_interfaces/msg/zit_status.h>
#include <zit6_interfaces/srv/get_params.h>
#include <zit6_interfaces/srv/update_params.h>
#include <rcl_interfaces/msg/log.h>

class MicroRosTask {
public:
  MicroRosTask() = default;
  void run();

private:
  static MicroRosTask *instance_;

  // micro-ROS 实体
  rclc_support_t support_;
  rcl_node_t node_;
  rclc_executor_t executor_;
  rcl_subscription_t setpoint_sub_, arm_sub_, ins_cmd_sub_, servo_sub_,
      led_sub_;
  rcl_publisher_t pos_pub_, vel_pub_, thr_pub_, zithbt_pub_, status_pub_, log_pub_;

  // 消息 + 缓冲区
  std_msgs__msg__Float32 servo_msg_;
  std_msgs__msg__UInt8 led_msg_;
  std_msgs__msg__Float32MultiArray pos_fb_msg_, vel_fb_msg_, thr_fb_msg_;
  zit6_interfaces__msg__ZitSetpoint setpoint_msg_;
  zit6_interfaces__msg__ZitStatus status_msg_;
  std_msgs__msg__UInt8 ins_cmd_msg_;
  std_msgs__msg__UInt32 arm_msg_, node_heartbeat_msg_;
  rcl_interfaces__msg__Log log_msg_;

  // services
  rcl_service_t update_params_srv_, get_params_srv_;
  zit6_interfaces__srv__UpdateParams_Request update_req_;
  zit6_interfaces__srv__UpdateParams_Response update_res_;
  zit6_interfaces__srv__GetParams_Request get_req_;
  zit6_interfaces__srv__GetParams_Response get_res_;

  float pos_buf_[4], vel_buf_[4], thr_buf_[4];

  // 处理函数（原来位于匿名命名空间）
  void onZitSetpoint(const void *msgin);
  void onArmHeartbeat(const void *msgin);
  void onInsCommand(const void *msgin);
  void onServoCmd(const void *msgin);
  void onLedCmd(const void *msgin);
  void onUpdateParams(const void *req, rmw_request_id_t *req_id, void *res);
  void onGetParams(const void *req, rmw_request_id_t *req_id, void *res);
  void cleanupMicroRos();

  // 静态回调封装（传入 rclc）
  static void setpointCb(const void *msgin) {
    if (instance_)
      instance_->onZitSetpoint(msgin);
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
  static void updateParamsCb(const void *req, rmw_request_id_t *req_id,
                             void *res) {
    if (instance_)
      instance_->onUpdateParams(req, req_id, res);
  }
  static void getParamsCb(const void *req, rmw_request_id_t *req_id,
                          void *res) {
    if (instance_)
      instance_->onGetParams(req, req_id, res);
  }
};

#endif // __MICROROS_TASK_HPP
