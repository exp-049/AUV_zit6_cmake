#ifndef __MICROROS_TRANSPORT_HPP
#define __MICROROS_TRANSPORT_HPP

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rcutils/allocator.h>
#include <stdint.h>

/**
 * @class MicroRosTransport
 * @brief Agent 连接与生命周期管理
 *
 * 负责：
 * - micro-ROS 底层传输层初始化（自定义串口传输）
 * - Agent 发现与 ping 保活
 * - rclc_support_t / rcl_node_t / rclc_executor_t 生命周期
 * - 分配器的全局配置
 */
class MicroRosTransport {
public:
  MicroRosTransport();
  ~MicroRosTransport();

  /**
   * @brief 初始化传输层（仅在 WAITING_AGENT 状态调用一次）
   * @param node_name  ROS 节点名
   * @param namespace_ ROS 命名空间
   * @param executor_capacity executor 最大实体容量
   * @return true 初始化成功
   */
  bool init(const char *node_name, const char *namespace_,
            size_t executor_capacity);

  /**
   * @brief 探测 Agent 是否在线
   * @param timeout_ms 单次超时 (ms)
   * @param attempts   重试次数
   * @return true 探测到 Agent
   */
  bool pingAgent(uint32_t timeout_ms, uint8_t attempts);

  /**
   * @brief 销毁所有 rcl 资源，回到未初始化状态
   */
  void fini();

  /// 获取内部 rcl 资源引用（供 Publisher/Subscriber/Service 使用）
  rcl_node_t &getNode() { return node_; }
  rclc_executor_t &getExecutor() { return executor_; }
  rclc_support_t &getSupport() { return support_; }
  rcl_allocator_t &getAllocator() { return rcl_allocator_; }

  bool isInitialized() const { return initialized_; }

private:
  rclc_support_t support_;
  rcl_node_t node_;
  rclc_executor_t executor_;
  rcl_allocator_t rcl_allocator_;

  bool initialized_ = false;
  bool transport_configured_ = false; ///< 传输层回调只配置一次
};

#endif // __MICROROS_TRANSPORT_HPP
