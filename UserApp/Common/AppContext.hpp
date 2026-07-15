#pragma once

/**
 * @file AppContext.hpp
 * @brief 应用上下文 — 依赖注入容器
 *
 * 仅使用前向声明，避免传递依赖。包含此头文件不会引入任何其他模块头文件。
 */

namespace auv::peripheral {
class INS_Driver;
class MotionController_Driver;
class MS5837_Driver;
class USBL_Driver;
} // namespace auv::peripheral

namespace auv::component {
class RosLogger;
class SoftWatchdog;
class ChassisManager;
} // namespace auv::component

/**
 * @struct AppContext
 * @brief 应用上下文 — 所有硬件驱动与组件的指针集合
 *
 * 用于依赖注入：任务构造函数接收 AppContext*，通过它访问所需模块，
 * 而非直接引用全局变量。
 */
namespace auv {
namespace system {

struct AppContext {
  auv::peripheral::INS_Driver *ins_driver;
  auv::peripheral::MotionController_Driver *motor_driver;
  auv::peripheral::MS5837_Driver *depth_sensor;
  auv::peripheral::USBL_Driver *usbl_driver;
  auv::component::RosLogger *logger;
  auv::component::SoftWatchdog *watchdog;
  auv::component::ChassisManager *chassis;
};

extern AppContext g_app_ctx;

} // namespace system
} // namespace auv
