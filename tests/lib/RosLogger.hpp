#ifndef __ROS_LOGGER_HPP
#define __ROS_LOGGER_HPP

// RosLogger 桩 — 用于主机端测试
// 使用与真实文件相同的 include guard，防止重复定义
// 所有日志宏替换为空操作

#define ROS_LOG_INFO(...)  do {} while(0)
#define ROS_LOG_WARN(...)  do {} while(0)
#define ROS_LOG_ERROR(...) do {} while(0)
#define ROS_LOG_DEBUG(...) do {} while(0)
#define ROS_LOG_FATAL(...) do {} while(0)

namespace auv {
namespace component {

class RosLogger {
public:
  void init() {}
  void log(uint8_t, const char*, ...) {}
  bool popLog(uint8_t*, char*) { return false; }
};

} // namespace component
} // namespace auv

#endif // __ROS_LOGGER_HPP
