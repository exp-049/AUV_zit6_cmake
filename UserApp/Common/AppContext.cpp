#include "AppContext.hpp"
#include "ChassisManager.hpp"
#include "INS_Driver.hpp"
#include "INS_Porting.hpp" // ins_rx_buffer + INS_Porting
#include "MS5837_Driver.hpp"
#include "MS5837_Porting.hpp"
#include "MotionController_Driver.hpp"
#include "MotionController_Porting.hpp"
#include "RosLogger.hpp"
#include "SoftWatchdog.hpp"
#include "i2c.h"
#include "main.h"
#include "usart.h"

// --- 全局 Porting 实例（硬件适配） ---
static auv::porting::INS_Porting g_ins_port(&huart7, &huart7, ins_rx_buffer,
                                            512);
static auv::porting::MotionController_Porting g_motor_port(&huart6);
static auv::porting::MS5837_Porting g_depth_port(&hi2c1, 0x76 << 1);

// --- 全局驱动实例 ---
namespace auv {
namespace peripheral {
auv::peripheral::INS_Driver ins_driver(auv::peripheral::InsPortOps{
    .ctx = &g_ins_port,
    .init = &auv::porting::INS_Porting::initPort,
    .read = &auv::porting::INS_Porting::readPort,
    .transmit = &auv::porting::INS_Porting::transmitPort,
});
auv::peripheral::MotionController_Driver
    motor_driver(auv::peripheral::MotorPortOps{
        .ctx = &g_motor_port,
        .transmitDMA = &auv::porting::MotionController_Porting::transmitDMA,
        .getTxPacket = &auv::porting::MotionController_Porting::getTxPacket,
    });
auv::peripheral::MS5837_Driver depth_sensor(auv::peripheral::DepthPortOps{
    .ctx = &g_depth_port,
    .writeByte = &auv::porting::MS5837_Porting::writePort,
    .readByte = &auv::porting::MS5837_Porting::readPortByte,
    .read = &auv::porting::MS5837_Porting::readPort,
    .delay = &auv::porting::MS5837_Porting::delayPort,
});
} // namespace peripheral
} // namespace auv

// --- 全局组件实例 ---
namespace auv {
namespace component {
auv::component::SoftWatchdog g_soft_watchdog;
auv::component::RosLogger g_ros_logger;
auv::component::ChassisManager chassis;
} // namespace component
} // namespace auv

// --- 全局应用上下文（依赖注入容器）---
namespace auv {
namespace system {
AppContext g_app_ctx = {
    .ins_driver = &auv::peripheral::ins_driver,
    .motor_driver = &auv::peripheral::motor_driver,
    .depth_sensor = &auv::peripheral::depth_sensor,
    .logger = &auv::component::g_ros_logger,
    .watchdog = &auv::component::g_soft_watchdog,
    .chassis = &auv::component::chassis,
};
} // namespace system
} // namespace auv
