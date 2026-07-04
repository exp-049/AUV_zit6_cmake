#ifndef __ROS_LOGGER_HPP
#define __ROS_LOGGER_HPP

#include <stdint.h>

// ========== DEBUG 日志开关 ==========
// 取消下一行的注释以启用 DEBUG 级别日志输出
#define DEBUG_LOG

namespace auv {
namespace component {

class RosLogger {
public:
  struct LogEntry {
    uint8_t level;
    char msg[128];
  };

  RosLogger() = default;
  RosLogger(const RosLogger &) = delete;
  RosLogger &operator=(const RosLogger &) = delete;

  void init();
  void log(uint8_t level, const char *format, ...);
  bool popLog(LogEntry &entry);

private:
  void *log_queue_ = nullptr;
};

extern auv::component::RosLogger g_ros_logger;
} // namespace component
} // namespace auv

// ========== 日志宏 ==========
// DEBUG 级别：仅在定义了 DEBUG_LOG 时编译
#ifdef DEBUG_LOG
#define ROS_LOG_DEBUG(fmt, ...)                                                \
  auv::component::g_ros_logger.log(10, fmt, ##__VA_ARGS__)
#else
#define ROS_LOG_DEBUG(fmt, ...) ((void)0)
#endif
#define ROS_LOG_INFO(fmt, ...)                                                 \
  auv::component::g_ros_logger.log(20, fmt, ##__VA_ARGS__)
#define ROS_LOG_WARN(fmt, ...)                                                 \
  auv::component::g_ros_logger.log(30, fmt, ##__VA_ARGS__)
#define ROS_LOG_ERROR(fmt, ...)                                                \
  auv::component::g_ros_logger.log(40, fmt, ##__VA_ARGS__)
#define ROS_LOG_FATAL(fmt, ...)                                                \
  auv::component::g_ros_logger.log(50, fmt, ##__VA_ARGS__)

#endif // __ROS_LOGGER_HPP
