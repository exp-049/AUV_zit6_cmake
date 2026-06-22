#include "MicroRosTransport.hpp"
#include "main.h"
#include <rcl/error_handling.h>
#include <rmw_microros/rmw_microros.h>

// 自定义传输回调（由 STM32CubeMX 生成的 usart.c 提供）
extern "C" {
extern UART_HandleTypeDef huart2;

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

MicroRosTransport::MicroRosTransport() {
  // 传输层回调只需配置一次（rmw_uros_set_custom_transport 可重复调用但无必要）
  rmw_uros_set_custom_transport(true, (void *)&huart2, cubemx_transport_open,
                                cubemx_transport_close, cubemx_transport_write,
                                cubemx_transport_read);

  // 配置 micro-ROS 分配器
  rcutils_allocator_t rcutils_alloc = rcutils_get_zero_initialized_allocator();
  rcutils_alloc.allocate = microros_allocate;
  rcutils_alloc.deallocate = microros_deallocate;
  rcutils_alloc.reallocate = microros_reallocate;
  rcutils_alloc.zero_allocate = microros_zero_allocate;
  rcutils_set_default_allocator(&rcutils_alloc);

  rcl_allocator_ = rcl_get_default_allocator();
}

MicroRosTransport::~MicroRosTransport() {
  if (initialized_)
    fini();
}

bool MicroRosTransport::init(const char *node_name, const char *namespace_,
                             size_t executor_capacity) {
  if (initialized_)
    return true;

  if (RCL_RET_OK != rclc_support_init(&support_, 0, NULL, &rcl_allocator_)) {
    return false;
  }

  // 同步会话时间
  rmw_uros_sync_session(100);

  if (RCL_RET_OK !=
      rclc_node_init_default(&node_, node_name, namespace_, &support_)) {
    rclc_support_fini(&support_);
    return false;
  }

  if (RCL_RET_OK != rclc_executor_init(&executor_, &support_.context,
                                       executor_capacity, &rcl_allocator_)) {
    rcl_node_fini(&node_);
    rclc_support_fini(&support_);
    return false;
  }

  initialized_ = true;
  return true;
}

bool MicroRosTransport::pingAgent(uint32_t timeout_ms, uint8_t attempts) {
  return (RCL_RET_OK == rmw_uros_ping_agent(timeout_ms, attempts));
}

void MicroRosTransport::fini() {
  if (!initialized_)
    return;

  rclc_executor_fini(&executor_);
  rcl_node_fini(&node_);
  rclc_support_fini(&support_);

  // 清空内存，防止残留句柄被误用
  memset(&executor_, 0, sizeof(executor_));
  memset(&node_, 0, sizeof(node_));
  memset(&support_, 0, sizeof(support_));

  initialized_ = false;
}
