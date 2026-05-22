#include "MicroRosTask.hpp"
#include "AppMain.hpp"
#include "MotionContext.hpp"
#include "SystemContext.hpp"
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
  auv::motion::motion_context.last_received_seq = msg->seq;
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

  auto nav = auv::motion::motion_context.getNavState();
  bool nav_valid = auv::system::system_context.getNavigationValid() ||
                   auv::config::sys_config.simulation.hitl_enabled;
  if ((new_level == auv::motion::ControlLevel::POSITION ||
       new_level == auv::motion::ControlLevel::VELOCITY) &&
      !nav_valid)
    return;

  taskENTER_CRITICAL();

  // 1. 记录原始 AGX 设定值快照
  auv::motion::motion_context.raw_setpoint.level = new_level;
  auv::motion::motion_context.raw_setpoint.data[0] = val[0];
  auv::motion::motion_context.raw_setpoint.data[1] = val[1];
  auv::motion::motion_context.raw_setpoint.data[2] = val[2];
  auv::motion::motion_context.raw_setpoint.data[3] = val[3];
  auv::motion::motion_context.raw_setpoint.type_mask = mask;
  auv::motion::motion_context.raw_setpoint.is_body = is_body;
  auv::motion::motion_context.raw_setpoint.is_incremental = is_inc;

  // 2. 模式切换对齐 (Bumpless Transition / Anti-Leakage)
  if (new_level != auv::control::chassis.getControlLevel()) {
    if (new_level == auv::motion::ControlLevel::POSITION) {
      for (int i = 0; i < 4; i++) {
        auv::motion::motion_context.current_setpoint.pos_world[i] = nav.pos_world[i];
      }
    } else if (new_level == auv::motion::ControlLevel::VELOCITY) {
      for (int i = 0; i < 4; i++) {
        auv::motion::motion_context.current_setpoint.vel_body[i] = nav.vel_body[i];
      }
    }
  }

  // 3. 执行坐标变换与目标值计算
  if (new_level == auv::motion::ControlLevel::POSITION) {
    float converted_val[4];
    if (is_body) {
      // 传入机体系位置设定（转换为世界系绝对或增量目标）
      auv::motion::motion_context.transformBodyToWorld(new_level, val, converted_val, is_inc);
    }

    for (int i = 0; i < 4; i++) {
      if (!(mask & (1 << i))) {
        if (is_inc) {
          if (is_body) {
            auv::motion::motion_context.current_setpoint.pos_world[i] += converted_val[i];
          } else {
            auv::motion::motion_context.current_setpoint.pos_world[i] += val[i];
          }
        } else {
          if (is_body) {
            auv::motion::motion_context.current_setpoint.pos_world[i] = converted_val[i];
          } else {
            auv::motion::motion_context.current_setpoint.pos_world[i] = val[i];
          }
        }
      }
    }
  } else if (new_level == auv::motion::ControlLevel::VELOCITY) {
    float converted_val[4];
    if (!is_body) {
      // 传入世界系速度目标（转换为机体系绝对或增量目标）
      auv::motion::motion_context.transformWorldToBody(new_level, val, converted_val, is_inc);
    }

    for (int i = 0; i < 4; i++) {
      if (!(mask & (1 << i))) {
        if (is_inc) {
          if (is_body) {
            auv::motion::motion_context.current_setpoint.vel_body[i] += val[i];
          } else {
            auv::motion::motion_context.current_setpoint.vel_body[i] += converted_val[i];
          }
        } else {
          if (is_body) {
            auv::motion::motion_context.current_setpoint.vel_body[i] = val[i];
          } else {
            auv::motion::motion_context.current_setpoint.vel_body[i] = converted_val[i];
          }
        }
      }
    }
  } else if (new_level == auv::motion::ControlLevel::ACTUATOR) {
    float converted_val[4];
    if (!is_body) {
      // 传入世界系推力目标（转换为机体系绝对或增量目标）
      auv::motion::motion_context.transformWorldToBody(new_level, val, converted_val, is_inc);
    }

    for (int i = 0; i < 4; i++) {
      if (!(mask & (1 << i))) {
        if (is_inc) {
          if (is_body) {
            auv::motion::motion_context.current_setpoint.thrust_body[i] += val[i];
          } else {
            auv::motion::motion_context.current_setpoint.thrust_body[i] += converted_val[i];
          }
        } else {
          if (is_body) {
            auv::motion::motion_context.current_setpoint.thrust_body[i] = val[i];
          } else {
            auv::motion::motion_context.current_setpoint.thrust_body[i] = converted_val[i];
          }
        }
      }
    }
  }

  auv::control::chassis.setControlLevel(new_level);
  taskEXIT_CRITICAL();
}

void MicroRosTask::onArmHeartbeat(const void *msgin) {
  const auto *msg = (const std_msgs__msg__UInt32 *)msgin;
  taskENTER_CRITICAL();
  auv::system::system_context.last_arm_heartbeat_ms = HAL_GetTick();
  auv::system::system_context.last_arm_heartbeat_data = msg->data;
  if (!auv::system::system_context.is_system_armed) {
    if (auv::system::system_context.arm_heartbeat_count == 0)
      auv::system::system_context.arm_start_ms = auv::system::system_context.last_arm_heartbeat_ms;
    auv::system::system_context.arm_heartbeat_count++;
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
  if (update_req_.paths.data) {
    for (size_t _i = 0; _i < update_req_.paths.capacity; ++_i) {
      if (update_req_.paths.data[_i].data) {
        microros_deallocate(update_req_.paths.data[_i].data, NULL);
        update_req_.paths.data[_i].data = NULL;
        update_req_.paths.data[_i].capacity = 0;
        update_req_.paths.data[_i].size = 0;
      }
    }
    rosidl_runtime_c__String__Sequence__fini(&update_req_.paths);
  }
  if (update_req_.values.data) {
    for (size_t _i = 0; _i < update_req_.values.capacity; ++_i) {
      if (update_req_.values.data[_i].data) {
        microros_deallocate(update_req_.values.data[_i].data, NULL);
        update_req_.values.data[_i].data = NULL;
        update_req_.values.data[_i].capacity = 0;
        update_req_.values.data[_i].size = 0;
      }
    }
    rosidl_runtime_c__String__Sequence__fini(&update_req_.values);
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
    auv::device::SoftWatchdog::getInstance().feed(
        auv::device::SoftWatchdog::Component::MICROROS);
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
      // Robust ping: 500ms timeout, 3 failures
      if (RCL_RET_OK != rmw_uros_ping_agent(500, 3)) {
        cleanupMicroRos();
        state = WAITING_AGENT;
      } else {
        rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(1));
        if (now_ms - last_hbt_pub_tick >= 1000) {
          last_hbt_pub_tick = now_ms;
          node_heartbeat_msg_.data = now_ms;
          rcl_publish(&zithbt_pub_, &node_heartbeat_msg_, NULL);
        }
        if (now_ms - last_vel_pub_tick >= 20) {
          last_vel_pub_tick = now_ms;
          auto nav = auv::motion::motion_context.getNavState();
          for (int i = 0; i < 4; i++) {
            vel_buf_[i] = nav.vel_body[i];
          }
          rcl_publish(&vel_pub_, &vel_fb_msg_, NULL);
        }
        if (now_ms - last_thr_pub_tick >= 33) {
          last_thr_pub_tick = now_ms;
          taskENTER_CRITICAL();
          for (int i = 0; i < 4; i++)
            thr_buf_[i] = auv::motion::motion_context.last_output_forces[i];
          taskEXIT_CRITICAL();
          rcl_publish(&thr_pub_, &thr_fb_msg_, NULL);
        }
        if (now_ms - last_pos_pub_tick >= 33) {
          last_pos_pub_tick = now_ms;
          auto nav = auv::motion::motion_context.getNavState();
          for (int i = 0; i < 4; i++) {
            pos_buf_[i] = nav.pos_world[i];
          }
          rcl_publish(&pos_pub_, &pos_fb_msg_, NULL);
        }
        if (now_ms - last_status_pub_tick >= 100) {
          last_status_pub_tick = now_ms;
          auv::motion::NavState nav = auv::motion::motion_context.getNavState();
          taskENTER_CRITICAL();
          status_msg_.is_armed = auv::system::system_context.is_system_armed;
          status_msg_.arm_mode = (uint8_t)auv::system::system_context.last_arm_heartbeat_data;
          status_msg_.control_level =
              (uint8_t)auv::control::chassis.getControlLevel();
          status_msg_.ins_state = auv::system::system_context.nav_status.imu_state;
          status_msg_.navigation_ready = auv::system::system_context.getNavigationValid();
          for (int i = 0; i < 4; i++)
            status_msg_.forces[i] = auv::motion::motion_context.last_output_forces[i];
          status_msg_.cycle_time_ms = (float)auv::motion::motion_context.last_dt_ms;
          status_msg_.battery_voltage = 0.0f;
          status_msg_.error_flags = 0;
          taskEXIT_CRITICAL();
          rcl_publish(&status_pub_, &status_msg_, NULL);
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
