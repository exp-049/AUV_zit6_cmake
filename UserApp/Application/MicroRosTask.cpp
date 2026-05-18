#include "MicroRosTask.hpp"
#include "AppMain.hpp"
#include "FreeRTOS.h"
#include "GlobalContext.hpp"
#include "SoftWatchdog.hpp"
#include "task.h"
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rcutils/allocator.h>
#include <rmw_microros/rmw_microros.h>

#include "ConfigService.hpp"
#include "cJSON.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/u_int8.h>
#include <zit6_interfaces/msg/zit_pid_status.h>
#include <zit6_interfaces/msg/zit_setpoint.h>
#include <zit6_interfaces/msg/zit_status.h>
#include <zit6_interfaces/srv/get_params.h>
#include <zit6_interfaces/srv/update_params.h>

extern "C" {
bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport,
                              const uint8_t *buf, size_t len, uint8_t *errcode);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf,
                             size_t len, int timeout, uint8_t *errcode);
void *microros_allocate(size_t size, void *state);
void microros_deallocate(void *ptr, void *state);
void *microros_reallocate(void *ptr, size_t new_size, void *state);
void *microros_zero_allocate(size_t number_of_elements,
                             size_t size_t_of_element, void *state);
}

static void *cjson_malloc(size_t size) { return microros_allocate(size, NULL); }
static void cjson_free(void *ptr) { microros_deallocate(ptr, NULL); }

// 实例指针定义
MicroRosTask *MicroRosTask::instance_ = nullptr;

// (on-target debug UART removed — no debug pins available)

// --- 成员回调实现 ---
void MicroRosTask::onZitSetpoint(const void *msgin) {
  const auto *msg = (const zit6_interfaces__msg__ZitSetpoint *)msgin;
  last_received_seq = msg->seq;
  if (!std::isfinite(msg->x) || !std::isfinite(msg->y) ||
      !std::isfinite(msg->z) || !std::isfinite(msg->yaw))
    return;
  if (!is_system_armed)
    return;

  uint32_t level_idx = msg->control_key & 0x03;
  if (level_idx >= 3)
    return;

  auv::common::ControlLevel new_level;
  if (level_idx == 0)
    new_level = auv::common::ControlLevel::POSITION;
  else if (level_idx == 1)
    new_level = auv::common::ControlLevel::VELOCITY;
  else
    new_level = auv::common::ControlLevel::ACTUATOR;

  bool is_body = (msg->control_key & 0x10) != 0;
  bool is_inc = (msg->control_key & 0x20) != 0;
  uint32_t mask = msg->type_mask;
  float val[4] = {msg->x, msg->y, msg->z, msg->yaw};

  auto nav = auv::shared::snapshotNavState();
  bool nav_valid = auv::shared::isNavigationValid(nav) ||
                   auv::config::sys_config.simulation.hitl_enabled;
  if ((new_level == auv::common::ControlLevel::POSITION ||
       new_level == auv::common::ControlLevel::VELOCITY) &&
      !nav_valid)
    return;

  taskENTER_CRITICAL();

  // 1. 模式切换对齐 (Bumpless Transition / Anti-Leakage)
  if (new_level != auv::control::chassis.getControlLevel()) {
    if (new_level == auv::common::ControlLevel::POSITION) {
      target_p[0] = nav.x;
      target_p[1] = nav.y;
      target_p[2] = nav.z;
      target_p[3] = nav.yaw;
    } else if (new_level == auv::common::ControlLevel::VELOCITY) {
      if (is_body) {
        target_p[0] = nav.vx;
        target_p[1] = nav.vy;
        target_p[2] = nav.vz;
        target_p[3] = nav.vyaw;
      } else {
        float wx, wy;
        auv::control::CoordinateManager::bodyToWorld(nav.yaw, nav.vx, nav.vy,
                                                     wx, wy);
        target_p[0] = wx;
        target_p[1] = wy;
        target_p[2] = nav.vz;
        target_p[3] = nav.vyaw;
      }
      auv::control::chassis.setVelocityTargetFrame(is_body);
    }
  }

  // 2. 执行具体指令逻辑
  if (new_level == auv::common::ControlLevel::ACTUATOR) {
    float fx, fy;
    if (is_body) {
      fx = val[0];
      fy = val[1];
    } else {
      auv::control::CoordinateManager::worldToBody(nav.yaw, val[0], val[1], fx,
                                                   fy);
    }

    static float last_forces[4] = {0};
    static auv::common::ControlLevel last_l = auv::common::ControlLevel::NONE;

    // 如果是刚切入 ACTUATOR 模式，清空之前的手动推力缓存
    if (last_l != auv::common::ControlLevel::ACTUATOR) {
      for (int i = 0; i < 4; i++)
        last_forces[i] = 0.0f;
    }
    last_l = auv::common::ControlLevel::ACTUATOR;

    if (!(mask & 1))
      last_forces[0] = fx;
    if (!(mask & 2))
      last_forces[1] = fy;
    if (!(mask & 4))
      last_forces[2] = val[2];
    if (!(mask & 8))
      last_forces[3] = val[3];

    auv::control::chassis.setActuatorForces(last_forces);
  } else if (new_level == auv::common::ControlLevel::VELOCITY) {
    // 确保下次切回 ACTUATOR 时重置
    static auv::common::ControlLevel last_l = auv::common::ControlLevel::NONE;
    last_l = new_level;

    auv::control::chassis.setVelocityTargetFrame(is_body);
    for (int i = 0; i < 4; i++) {
      if (!(mask & (1 << i))) {
        if (is_inc) {
          // 速度增量模式：在当前目标速度基础上叠加
          target_p[i] += val[i];
        } else {
          target_p[i] = val[i];
        }
      }
    }
  } else if (new_level == auv::common::ControlLevel::POSITION) {
    // 确保下次切回 ACTUATOR 时重置
    static auv::common::ControlLevel last_l = auv::common::ControlLevel::NONE;
    last_l = new_level;

    // 预处理机体系分量：X/Y 必须作为矢量整体旋转
    float dx_world = 0.0f, dy_world = 0.0f;
    if (is_body) {
      float bx = (mask & 1) ? 0.0f : val[0];
      float by = (mask & 2) ? 0.0f : val[1];
      auv::control::CoordinateManager::bodyToWorld(nav.yaw, bx, by, dx_world,
                                                   dy_world);
    }

    for (int i = 0; i < 4; i++) {
      if (!(mask & (1 << i))) {
        if (is_inc) {
          if (is_body && i < 2) {
            // 机体系位置增量：使用预计算好的世界系分量
            if (i == 0)
              target_p[0] += dx_world;
            if (i == 1)
              target_p[1] += dy_world;
          } else {
            // 世界系位置/深度/偏航增量：直接累加到当前目标上
            target_p[i] += val[i];
          }
        } else {
          // 非增量模式
          if (is_body) {
            if (i < 2) {
              // 机体系绝对位置：相对于当前“实时位置”的偏移 (Snapshot Relative)
              if (i == 0)
                target_p[0] = nav.x + dx_world;
              if (i == 1)
                target_p[1] = nav.y + dy_world;
            } else {
              // Z 和 Yaw 同样相对于当前实时位置
              target_p[i] = (i == 2 ? nav.z : nav.yaw) + val[i];
            }
          } else {
            // 世界系绝对位置
            target_p[i] = val[i];
          }
        }
      }
    }
  }

  float actual_p[4] = {nav.x, nav.y, nav.z, nav.yaw};
  float actual_v[4] = {nav.vx, nav.vy, nav.vz, nav.vyaw};
  auv::control::chassis.setControlLevel(new_level, actual_p, actual_v);
  taskEXIT_CRITICAL();
}

void MicroRosTask::onArmHeartbeat(const void *msgin) {
  const auto *msg = (const std_msgs__msg__UInt32 *)msgin;
  taskENTER_CRITICAL();
  last_arm_heartbeat_ms = HAL_GetTick();
  last_arm_heartbeat_data = msg->data;
  if (!is_system_armed) {
    if (arm_heartbeat_count == 0)
      arm_start_ms = last_arm_heartbeat_ms;
    arm_heartbeat_count++;
  }
  taskEXIT_CRITICAL();
}

void MicroRosTask::onInsCommand(const void *msgin) {
  const auto *message = static_cast<const std_msgs__msg__UInt8 *>(msgin);
  if (message == nullptr)
    return;
  switch (message->data) {
  case 1:
    auv::device::ins_driver.setDvlPower(true);
    break;
  case 2:
    auv::device::ins_driver.setDvlPower(false);
    break;
  case 3:
    auv::device::ins_driver.restart();
    break;
  case 4:
    auv::device::ins_driver.resetPosition();
    break;
  case 5:
    auv::device::ins_driver.setInitialPosition(
        auv::config::sys_config.ins.init_lat,
        auv::config::sys_config.ins.init_lon);
    break;
  }
}

void MicroRosTask::onServoCmd(const void *msgin) {
  const auto *msg = (const std_msgs__msg__Float32 *)msgin;
  auv::device::motor_driver.setServoAngle(msg->data);
}

void MicroRosTask::onLedCmd(const void *msgin) {
  const auto *msg = (const std_msgs__msg__UInt8 *)msgin;
  auv::device::motor_driver.setLightState(msg->data);
}

void MicroRosTask::onUpdateParams(const void *reqin, rmw_request_id_t *req_id,
                                  void *resin) {
  (void)req_id;
  auto *res = static_cast<zit6_interfaces__srv__UpdateParams_Response *>(resin);
  const auto *req =
      static_cast<const zit6_interfaces__srv__UpdateParams_Request *>(reqin);
  if (!res || !req)
    return;

  const char *json_ptr = (req->json.size > 0) ? req->json.data : nullptr;
  const char *paths[16] = {nullptr};
  const char *values[16] = {nullptr};
  size_t count = (req->paths.size < 16) ? req->paths.size : 16;
  for (size_t i = 0; i < count; ++i) {
    paths[i] = req->paths.data[i].data;
    values[i] = req->values.data[i].data;
  }

  char out_buf[64] = {0};
  res->success = auv::service::ConfigService::updateParams(
      json_ptr, paths, values, count, out_buf, 64);

  // 如果参数中有 PID 相关修改，同步到控制算法
  if (res->success) {
    auv::control::chassis.applyConfig(auv::config::sys_config.chassis);
  }

  rosidl_runtime_c__String__assign(&res->message, out_buf);
}

void MicroRosTask::onGetParams(const void *reqin, rmw_request_id_t *req_id,
                               void *resin) {
  (void)req_id;
  auto *res = static_cast<zit6_interfaces__srv__GetParams_Response *>(resin);
  const auto *req =
      static_cast<const zit6_interfaces__srv__GetParams_Request *>(reqin);
  if (!res)
    return;

  const char *paths[16] = {nullptr};
  size_t count = (req && req->paths.data)
                     ? ((req->paths.size < 16) ? req->paths.size : 16)
                     : 0;
  for (size_t i = 0; i < count; ++i) {
    paths[i] = req->paths.data[i].data;
  }

  const char *json_res =
      auv::service::ConfigService::getParamsJson(paths, count);
  res->success = true;
  rosidl_runtime_c__String__assign(&res->config_json, json_res);
  rosidl_runtime_c__String__assign(&res->message, "ok");
}

void MicroRosTask::cleanupMicroRos() {
  rclc_executor_fini(&executor_);
  // free pre-allocated request/response buffers
  if (get_req_.paths.data) {
    for (size_t _i = 0; _i < get_req_.paths.capacity; ++_i) {
      if (get_req_.paths.data[_i].data) {
        microros_deallocate(get_req_.paths.data[_i].data, NULL);
        get_req_.paths.data[_i].data = NULL;
        get_req_.paths.data[_i].capacity = 0;
        get_req_.paths.data[_i].size = 0;
      }
    }
    rosidl_runtime_c__String__Sequence__fini(&get_req_.paths);
  }
  if (update_req_.json.data) {
    microros_deallocate(update_req_.json.data, NULL);
    update_req_.json.data = NULL;
    update_req_.json.capacity = 0;
    update_req_.json.size = 0;
  }
  if (get_res_.config_json.data) {
    microros_deallocate(get_res_.config_json.data, NULL);
    get_res_.config_json.data = NULL;
    get_res_.config_json.capacity = 0;
    get_res_.config_json.size = 0;
  }
  rcl_service_fini(&update_params_srv_, &node_);
  rcl_service_fini(&get_params_srv_, &node_);
  rcl_publisher_fini(&pos_pub_, &node_);
  rcl_publisher_fini(&vel_pub_, &node_);
  rcl_publisher_fini(&thr_pub_, &node_);
  rcl_publisher_fini(&zithbt_pub_, &node_);
  rcl_publisher_fini(&status_pub_, &node_);
  rcl_publisher_fini(&pid_status_pub_, &node_);
  rcl_subscription_fini(&setpoint_sub_, &node_);
  rcl_subscription_fini(&arm_sub_, &node_);
  rcl_subscription_fini(&ins_cmd_sub_, &node_);
  rcl_subscription_fini(&servo_sub_, &node_);
  rcl_subscription_fini(&led_sub_, &node_);
  rcl_node_fini(&node_);
  rclc_support_fini(&support_);
  memset(&support_, 0, sizeof(support_));
  memset(&node_, 0, sizeof(node_));
  memset(&executor_, 0, sizeof(executor_));
}

void MicroRosTask::run() {
  MicroRosTask::instance_ = this;
  rmw_uros_set_custom_transport(true, (void *)&huart2, cubemx_transport_open,
                                cubemx_transport_close, cubemx_transport_write,
                                cubemx_transport_read);

  // Initialize cJSON with micro-ROS allocators
  cJSON_Hooks hooks;
  hooks.malloc_fn = cjson_malloc;
  hooks.free_fn = cjson_free;
  cJSON_InitHooks(&hooks);

  rcutils_allocator_t allocator = rcutils_get_zero_initialized_allocator();
  allocator.allocate = microros_allocate;
  allocator.deallocate = microros_deallocate;
  allocator.reallocate = microros_reallocate;
  allocator.zero_allocate = microros_zero_allocate;
  rcutils_set_default_allocator(&allocator);
  rcl_allocator_t rcl_allocator = rcl_get_default_allocator();

  uint32_t last_hbt_pub_tick = 0, last_vel_pub_tick = 0, last_thr_pub_tick = 0,
           last_pos_pub_tick = 0, last_status_pub_tick = 0;
  enum uros_state { WAITING_AGENT, AGENT_CONNECTED } state = WAITING_AGENT;

  for (;;) {
    uint32_t now_ms = HAL_GetTick();
    if (state == WAITING_AGENT) {
      if (RCL_RET_OK == rmw_uros_ping_agent(200, 1)) {
        if (RCL_RET_OK ==
            rclc_support_init(&support_, 0, NULL, &rcl_allocator)) {
          rmw_uros_sync_session(100);
          rclc_node_init_default(&node_, "zit6_node", "", &support_);
          std_msgs__msg__Float32MultiArray__init(&pos_fb_msg_);
          pos_fb_msg_.data.data = pos_buf_;
          pos_fb_msg_.data.size = 4;
          pos_fb_msg_.data.capacity = 4;
          std_msgs__msg__Float32MultiArray__init(&vel_fb_msg_);
          vel_fb_msg_.data.data = vel_buf_;
          vel_fb_msg_.data.size = 4;
          vel_fb_msg_.data.capacity = 4;
          std_msgs__msg__Float32MultiArray__init(&thr_fb_msg_);
          thr_fb_msg_.data.data = thr_buf_;
          thr_fb_msg_.data.size = 4;
          thr_fb_msg_.data.capacity = 4;
          std_msgs__msg__UInt32__init(&node_heartbeat_msg_);
          std_msgs__msg__UInt32__init(&arm_msg_);
          std_msgs__msg__UInt8__init(&ins_cmd_msg_);
          std_msgs__msg__UInt8__init(&led_msg_);
          std_msgs__msg__Float32__init(&servo_msg_);
          zit6_interfaces__msg__ZitStatus__init(&status_msg_);

          rclc_publisher_init_default(
              &pos_pub_, &node_,
              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
              "/zit6/state/pos");
          rclc_publisher_init_default(
              &vel_pub_, &node_,
              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
              "/zit6/state/vel");
          rclc_publisher_init_default(
              &thr_pub_, &node_,
              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
              "/zit6/state/thr");
          rclc_publisher_init_default(
              &zithbt_pub_, &node_,
              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
              "/zit6/state/zithbt");
          rclc_publisher_init_default(
              &status_pub_, &node_,
              ROSIDL_GET_MSG_TYPE_SUPPORT(zit6_interfaces, msg, ZitStatus),
              "/zit6/state/status");
          rclc_publisher_init_default(
              &pid_status_pub_, &node_,
              ROSIDL_GET_MSG_TYPE_SUPPORT(zit6_interfaces, msg, ZitPidStatus),
              "/zit6/state/pid_status");

          rclc_subscription_init_default(
              &led_sub_, &node_,
              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
              "/zit6/cmd/light");
          rclc_subscription_init_default(
              &servo_sub_, &node_,
              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
              "/zit6/cmd/servo");
          rclc_subscription_init_default(
              &setpoint_sub_, &node_,
              ROSIDL_GET_MSG_TYPE_SUPPORT(zit6_interfaces, msg, ZitSetpoint),
              "/zit6/cmd/setpoint");
          rclc_subscription_init_default(
              &arm_sub_, &node_,
              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
              "/zit6/cmd/agxhbt");
          rclc_subscription_init_default(
              &ins_cmd_sub_, &node_,
              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
              "/zit6/cmd/ins");

          rclc_executor_init(&executor_, &support_.context, 18, &rcl_allocator);
          rclc_executor_add_subscription(
              &executor_, &setpoint_sub_, &setpoint_msg_,
              &MicroRosTask::setpointCb, ON_NEW_DATA);
          rclc_executor_add_subscription(&executor_, &arm_sub_, &arm_msg_,
                                         &MicroRosTask::armCb, ON_NEW_DATA);
          rclc_executor_add_subscription(&executor_, &ins_cmd_sub_,
                                         &ins_cmd_msg_, &MicroRosTask::insCmdCb,
                                         ON_NEW_DATA);
          rclc_executor_add_subscription(&executor_, &servo_sub_, &servo_msg_,
                                         &MicroRosTask::servoCb, ON_NEW_DATA);
          rclc_executor_add_subscription(&executor_, &led_sub_, &led_msg_,
                                         &MicroRosTask::ledCb, ON_NEW_DATA);
          // Initialize services for parameter update and query
          // Initialize update_params service and check return codes
          {
            rcl_ret_t rc = rclc_service_init_default(
                &update_params_srv_, &node_,
                ROSIDL_GET_SRV_TYPE_SUPPORT(zit6_interfaces, srv, UpdateParams),
                "/zit6/update_params");
            (void)rc;
            zit6_interfaces__srv__UpdateParams_Request__init(&update_req_);
            // Pre-allocate buffer for incoming JSON string
            update_req_.json.data = (char *)microros_allocate(1024, NULL);
            update_req_.json.capacity = 1024;
            update_req_.json.size = 0;

            // 重要修复：预分配 paths 和 values 序列内存
            rosidl_runtime_c__String__Sequence__init(&update_req_.paths, 16);
            rosidl_runtime_c__String__Sequence__init(&update_req_.values, 16);
            for (size_t _i = 0; _i < 16; ++_i) {
              update_req_.paths.data[_i].data =
                  (char *)microros_allocate(64, NULL);
              update_req_.paths.data[_i].capacity = 64;
              update_req_.values.data[_i].data =
                  (char *)microros_allocate(64, NULL);
              update_req_.values.data[_i].capacity = 64;
            }

            zit6_interfaces__srv__UpdateParams_Response__init(&update_res_);
            rc = rclc_executor_add_service_with_request_id(
                &executor_, &update_params_srv_, &update_req_, &update_res_,
                &MicroRosTask::updateParamsCb);
            (void)rc;
          }

          // Initialize get_params service and check return codes
          {
            rcl_ret_t rc = rclc_service_init_default(
                &get_params_srv_, &node_,
                ROSIDL_GET_SRV_TYPE_SUPPORT(zit6_interfaces, srv, GetParams),
                "/zit6/get_params");
            (void)rc;
            zit6_interfaces__srv__GetParams_Request__init(&get_req_);
            // pre-initialize paths sequence to avoid NULL data pointer on
            // incoming requests
            rosidl_runtime_c__String__Sequence__init(&get_req_.paths, 8);
            // allocate per-element string buffers to give rmw a place to write
            // incoming path strings
            if (get_req_.paths.data) {
              for (size_t _i = 0; _i < get_req_.paths.capacity; ++_i) {
                get_req_.paths.data[_i].data =
                    (char *)microros_allocate(64, NULL);
                get_req_.paths.data[_i].capacity = 64;
                get_req_.paths.data[_i].size = 0;
              }
            }
            zit6_interfaces__srv__GetParams_Response__init(&get_res_);
            // Pre-allocate buffer for outgoing JSON string
            get_res_.config_json.data = (char *)microros_allocate(1024, NULL);
            get_res_.config_json.capacity = 1024;
            get_res_.config_json.size = 0;
            rc = rclc_executor_add_service_with_request_id(
                &executor_, &get_params_srv_, &get_req_, &get_res_,
                &MicroRosTask::getParamsCb);
            (void)rc;
          }
          state = AGENT_CONNECTED;
        }
      } else
        vTaskDelay(pdMS_TO_TICKS(100));
    } else {
      static uint32_t last_pid_pub_tick = 0;
      // Robust ping: 500ms timeout, 3 failures
      if (RCL_RET_OK != rmw_uros_ping_agent(500, 3)) {
        cleanupMicroRos();
        state = WAITING_AGENT;
      } else {
        auv::device::SoftWatchdog::getInstance().feed(
            auv::device::SoftWatchdog::Component::MICROROS);
        rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(1));
        if (now_ms - last_hbt_pub_tick >= 1000) {
          last_hbt_pub_tick = now_ms;
          node_heartbeat_msg_.data = now_ms;
          rcl_publish(&zithbt_pub_, &node_heartbeat_msg_, NULL);
        }
        if (now_ms - last_vel_pub_tick >= 20) {
          last_vel_pub_tick = now_ms;
          auto nav = auv::shared::snapshotNavState();
          vel_buf_[0] = nav.vx;
          vel_buf_[1] = nav.vy;
          vel_buf_[2] = nav.vz;
          vel_buf_[3] = nav.vyaw;
          rcl_publish(&vel_pub_, &vel_fb_msg_, NULL);
        }
        if (now_ms - last_thr_pub_tick >= 33) {
          last_thr_pub_tick = now_ms;
          taskENTER_CRITICAL();
          for (int i = 0; i < 4; i++)
            thr_buf_[i] = last_output_forces[i];
          taskEXIT_CRITICAL();
          rcl_publish(&thr_pub_, &thr_fb_msg_, NULL);
        }
        if (now_ms - last_pos_pub_tick >= 33) {
          last_pos_pub_tick = now_ms;
          auto nav = auv::shared::snapshotNavState();
          pos_buf_[0] = nav.x;
          pos_buf_[1] = nav.y;
          pos_buf_[2] = nav.z;
          pos_buf_[3] = nav.yaw;
          rcl_publish(&pos_pub_, &pos_fb_msg_, NULL);
        }
        if (now_ms - last_status_pub_tick >= 100) {
          last_status_pub_tick = now_ms;
          auv::common::NavState nav = auv::shared::snapshotNavState();
          taskENTER_CRITICAL();
          status_msg_.is_armed = is_system_armed;
          status_msg_.arm_mode = (uint8_t)last_arm_heartbeat_data;
          status_msg_.control_level =
              (uint8_t)auv::control::chassis.getControlLevel();
          status_msg_.ins_state = nav.imu_state;
          status_msg_.navigation_ready = auv::shared::isNavigationValid(nav);
          for (int i = 0; i < 4; i++)
            status_msg_.forces[i] = last_output_forces[i];
          status_msg_.cycle_time_ms = (float)last_dt_ms;
          status_msg_.battery_voltage = 0.0f;
          status_msg_.error_flags = 0;
          taskEXIT_CRITICAL();
          rcl_publish(&status_pub_, &status_msg_, NULL);
        }
        if (now_ms - last_pid_pub_tick >= 1000) {
          last_pid_pub_tick = now_ms;
          for (int i = 0; i < 4; i++) {
            auto p_cfg = auv::control::chassis.getPIDConfig(i, true);
            pid_status_msg_.pos_kp[i] = p_cfg.kp;
            auv::control::chassis.getProfileLimits(
                i, pid_status_msg_.pos_max_v[i], pid_status_msg_.pos_max_a[i]);
            pid_status_msg_.pos_out_limit[i] = p_cfg.output_limit;
            auto v_cfg = auv::control::chassis.getPIDConfig(i, false);
            pid_status_msg_.vel_kp[i] = v_cfg.kp;
            pid_status_msg_.vel_ki[i] = v_cfg.ki;
            pid_status_msg_.vel_kd[i] = v_cfg.kd;
            pid_status_msg_.vel_i_limit[i] = v_cfg.i_limit;
            pid_status_msg_.vel_out_limit[i] = v_cfg.output_limit;
          }
          rcl_publish(&pid_status_pub_, &pid_status_msg_, NULL);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void UserApp_MicroRosTask(void *argument) {
  (void)argument;
  MicroRosTask runner;
  runner.run();
}
