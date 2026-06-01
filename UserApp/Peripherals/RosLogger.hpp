#ifndef __ROS_LOGGER_HPP
#define __ROS_LOGGER_HPP

#include <stdint.h>

namespace auv {
namespace device {

class RosLogger {
public:
    struct LogEntry {
        uint8_t level;
        char msg[128];
    };

    static RosLogger& getInstance() {
        static RosLogger instance;
        return instance;
    }

    void init();

    void log(uint8_t level, const char* format, ...);

    bool popLog(LogEntry& entry);

private:
    RosLogger() = default;
    RosLogger(const RosLogger&) = delete;
    RosLogger& operator=(const RosLogger&) = delete;

    void* log_queue_ = nullptr;
};

#define ROS_LOG_DEBUG(fmt, ...) auv::device::RosLogger::getInstance().log(10, fmt, ##__VA_ARGS__)
#define ROS_LOG_INFO(fmt, ...)  auv::device::RosLogger::getInstance().log(20, fmt, ##__VA_ARGS__)
#define ROS_LOG_WARN(fmt, ...)  auv::device::RosLogger::getInstance().log(30, fmt, ##__VA_ARGS__)
#define ROS_LOG_ERROR(fmt, ...) auv::device::RosLogger::getInstance().log(40, fmt, ##__VA_ARGS__)
#define ROS_LOG_FATAL(fmt, ...) auv::device::RosLogger::getInstance().log(50, fmt, ##__VA_ARGS__)

} // namespace device
} // namespace auv

#endif // __ROS_LOGGER_HPP
