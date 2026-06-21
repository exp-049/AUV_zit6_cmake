#ifndef __MICROROS_SERVICE_HPP
#define __MICROROS_SERVICE_HPP

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <stdint.h>

#include <zit6_interfaces/srv/get_params.h>
#include <zit6_interfaces/srv/update_params.h>

/**
 * @class MicroRosService
 * @brief 参数配置服务模块
 *
 * 提供服务：
 * - /zit6/update_params — 参数更新（JSON + 路径/值数组）
 * - /zit6/get_params    — 参数查询
 */
class MicroRosService {
public:
  MicroRosService() = default;

  /**
   * @brief 创建服务并注册到 executor
   * @param node     rcl 节点指针
   * @param executor rclc executor 指针
   * @return true 全部成功
   */
  bool init(rcl_node_t *node, rclc_executor_t *executor);

  /**
   * @brief 销毁所有服务并释放预分配缓冲区
   * @param node rcl 节点指针
   */
  void cleanup(rcl_node_t *node);

private:
  // ---------- 服务句柄 ----------
  rcl_service_t update_params_srv_, get_params_srv_;

  // ---------- 请求/响应消息 ----------
  zit6_interfaces__srv__UpdateParams_Request update_req_;
  zit6_interfaces__srv__UpdateParams_Response update_res_;
  zit6_interfaces__srv__GetParams_Request get_req_;
  zit6_interfaces__srv__GetParams_Response get_res_;

  // ---------- 实例指针（用于静态回调转发） ----------
  static MicroRosService *instance_;

  // ---------- 回调处理函数 ----------
  void onUpdateParams(const void *req, rmw_request_id_t *req_id, void *res);
  void onGetParams(const void *req, rmw_request_id_t *req_id, void *res);

  // ---------- 静态回调封装 ----------
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

#endif // __MICROROS_SERVICE_HPP
