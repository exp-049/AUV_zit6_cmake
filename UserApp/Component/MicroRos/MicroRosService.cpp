#include "MicroRosService.hpp"
#include "ChassisManager.hpp"
#include "../ConfigService/ConfigService.hpp"
#include "INS_Driver.hpp"
#include "MotionController_Driver.hpp"
#include "RosLogger.hpp"
#include "SystemConfig.hpp"
#include <cstring>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rosidl_runtime_c/string_functions.h>

// 外部分配器（由 MicroRosTask 或 MicroRosTransport 配置）
extern "C" void *microros_allocate(size_t size, void *state);
extern "C" void microros_deallocate(void *ptr, void *state);

// 实例指针定义
MicroRosService *MicroRosService::instance_ = nullptr;

// ============================================================================
// 初始化
// ============================================================================

bool MicroRosService::init(rcl_node_t *node, rclc_executor_t *executor) {
  instance_ = this;

  // ---------- update_params 服务 ----------
  {
    rcl_ret_t rc = rclc_service_init_default(
        &update_params_srv_, node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(zit6_interfaces, srv, UpdateParams),
        "/zit6/update_params");
    if (rc != RCL_RET_OK)
      return false;

    zit6_interfaces__srv__UpdateParams_Request__init(&update_req_);
    // 预分配 JSON buffer
    update_req_.json.data = (char *)microros_allocate(4096, NULL);
    update_req_.json.capacity = 4096;
    update_req_.json.size = 0;

    // 预分配 paths & values 序列
    rosidl_runtime_c__String__Sequence__init(&update_req_.paths, 16);
    rosidl_runtime_c__String__Sequence__init(&update_req_.values, 16);
    for (size_t i = 0; i < 16; ++i) {
      update_req_.paths.data[i].data = (char *)microros_allocate(64, NULL);
      update_req_.paths.data[i].capacity = 64;
      update_req_.values.data[i].data = (char *)microros_allocate(64, NULL);
      update_req_.values.data[i].capacity = 64;
    }

    zit6_interfaces__srv__UpdateParams_Response__init(&update_res_);
    rc = rclc_executor_add_service_with_request_id(
        executor, &update_params_srv_, &update_req_, &update_res_,
        &MicroRosService::updateParamsCb);
    if (rc != RCL_RET_OK)
      return false;
  }

  // ---------- get_params 服务 ----------
  {
    rcl_ret_t rc = rclc_service_init_default(
        &get_params_srv_, node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(zit6_interfaces, srv, GetParams),
        "/zit6/get_params");
    if (rc != RCL_RET_OK)
      return false;

    zit6_interfaces__srv__GetParams_Request__init(&get_req_);
    rosidl_runtime_c__String__Sequence__init(&get_req_.paths, 8);
    if (get_req_.paths.data) {
      for (size_t i = 0; i < get_req_.paths.capacity; ++i) {
        get_req_.paths.data[i].data = (char *)microros_allocate(64, NULL);
        get_req_.paths.data[i].capacity = 64;
        get_req_.paths.data[i].size = 0;
      }
    }

    zit6_interfaces__srv__GetParams_Response__init(&get_res_);
    get_res_.config_json.data = (char *)microros_allocate(4096, NULL);
    get_res_.config_json.capacity = 4096;
    get_res_.config_json.size = 0;

    rc = rclc_executor_add_service_with_request_id(
        executor, &get_params_srv_, &get_req_, &get_res_,
        &MicroRosService::getParamsCb);
    if (rc != RCL_RET_OK)
      return false;
  }

  return true;
}

// ============================================================================
// 销毁
// ============================================================================

void MicroRosService::cleanup(rcl_node_t *node) {
  // 释放预分配缓冲区
  if (update_req_.json.data) {
    microros_deallocate(update_req_.json.data, NULL);
    update_req_.json.data = NULL;
    update_req_.json.capacity = 0;
    update_req_.json.size = 0;
  }
  if (update_req_.paths.data) {
    for (size_t i = 0; i < update_req_.paths.capacity; ++i) {
      if (update_req_.paths.data[i].data) {
        microros_deallocate(update_req_.paths.data[i].data, NULL);
        update_req_.paths.data[i].data = NULL;
        update_req_.paths.data[i].capacity = 0;
        update_req_.paths.data[i].size = 0;
      }
    }
    rosidl_runtime_c__String__Sequence__fini(&update_req_.paths);
  }
  if (update_req_.values.data) {
    for (size_t i = 0; i < update_req_.values.capacity; ++i) {
      if (update_req_.values.data[i].data) {
        microros_deallocate(update_req_.values.data[i].data, NULL);
        update_req_.values.data[i].data = NULL;
        update_req_.values.data[i].capacity = 0;
        update_req_.values.data[i].size = 0;
      }
    }
    rosidl_runtime_c__String__Sequence__fini(&update_req_.values);
  }
  if (get_req_.paths.data) {
    for (size_t i = 0; i < get_req_.paths.capacity; ++i) {
      if (get_req_.paths.data[i].data) {
        microros_deallocate(get_req_.paths.data[i].data, NULL);
        get_req_.paths.data[i].data = NULL;
        get_req_.paths.data[i].capacity = 0;
        get_req_.paths.data[i].size = 0;
      }
    }
    rosidl_runtime_c__String__Sequence__fini(&get_req_.paths);
  }
  if (get_res_.config_json.data) {
    microros_deallocate(get_res_.config_json.data, NULL);
    get_res_.config_json.data = NULL;
    get_res_.config_json.capacity = 0;
    get_res_.config_json.size = 0;
  }

  rcl_service_fini(&update_params_srv_, node);
  rcl_service_fini(&get_params_srv_, node);
}

// ============================================================================
// 服务处理实现
// ============================================================================

void MicroRosService::onUpdateParams(const void *reqin,
                                     rmw_request_id_t *req_id, void *resin) {
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
  res->success = auv::component::ConfigService::updateParams(
      json_ptr, paths, values, count, out_buf, 64);

  if (res->success) {
    ctx_->chassis->applyConfig(auv::config::sys_config.chassis);
    ROS_LOG_INFO("Params updated: %s", out_buf);
  } else {
    ROS_LOG_WARN("Params update failed: %s", out_buf);
  }

  rosidl_runtime_c__String__assign(&res->message, out_buf);
}

void MicroRosService::onGetParams(const void *reqin, rmw_request_id_t *req_id,
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
      auv::component::ConfigService::getParamsJson(paths, count);
  res->success = true;
  rosidl_runtime_c__String__assign(&res->config_json, json_res);
  rosidl_runtime_c__String__assign(&res->message, "ok");
  ROS_LOG_INFO("Params queried: count=%d", (int)count);
}
