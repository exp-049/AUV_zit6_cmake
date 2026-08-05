#pragma once

// AppContext 桩 — 用于主机端测试
// 只提供指针容器，不引入实际硬件驱动头文件

namespace auv::peripheral {
class INS_Driver;
class MotionController_Driver;
class Depth_Sensor_Driver;
class Pushrod_Driver;
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
  auv::peripheral::Depth_Sensor_Driver *depth_sensor = nullptr;
  auv::peripheral::Pushrod_Driver *pushrod_driver = nullptr;
  auv::component::RosLogger *logger = nullptr;
  auv::component::SoftWatchdog *watchdog = nullptr;
  auv::component::ChassisManager *chassis = nullptr;
};

extern AppContext g_app_ctx;

} // namespace system
} // namespace auv
