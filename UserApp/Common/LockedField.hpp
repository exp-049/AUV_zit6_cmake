#pragma once

#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief 线程安全字段包装器
 *
 * 对任何可拷贝类型 T 自动加 get/set 临界区保护。
 * 专用于 MotionContext 这类跨任务共享变量的场景。
 */
template <typename T> class LockedField {
public:
  LockedField() = default;
  LockedField(const T &initial) : val_(initial) {}

  /** @brief 临界区读取 */
  T get() const {
    taskENTER_CRITICAL();
    T v = val_;
    taskEXIT_CRITICAL();
    return v;
  }

  /** @brief 临界区写入 */
  void set(const T &v) {
    taskENTER_CRITICAL();
    val_ = v;
    taskEXIT_CRITICAL();
  }

  /** @brief 非安全直接访问（不得已时用） */
  T &unsafe() { return val_; }
  const T &unsafe() const { return val_; }

private:
  T val_{};
};
