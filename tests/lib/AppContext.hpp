#pragma once

// AppContext 桩 — 用于主机端测试
// 只提供指针容器，不引入实际硬件驱动头文件

namespace auv::peripheral {
class INS_Driver;
class MotionController_Driver;
class MS5837_Driver;
}

namespace auv::component {
class RosLogger;
class SoftWatchdog;
class ChassisManager;
}

namespace auv {
namespace system {

struct AppContext {
  auv::peripheral::INS_Driver *ins_driver = nullptr;
  auv::peripheral::MotionController_Driver *motor_driver = nullptr;
  auv::peripheral::MS5837_Driver *depth_sensor = nullptr;
  auv::component::RosLogger *logger = nullptr;
  auv::component::SoftWatchdog *watchdog = nullptr;
  auv::component::ChassisManager *chassis = nullptr;
};

extern AppContext g_app_ctx;

} // namespace system
} // namespace auv
