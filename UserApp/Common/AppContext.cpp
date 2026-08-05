#include "AppContext.hpp"
#include "ChassisManager.hpp"
#include "INS_Driver.hpp"
#include "INS_Porting.hpp" // ins_rx_buffer + INS_Porting
#include "Depth_Sensor_Driver.hpp"
#include "Depth_Sensor_Porting.hpp"
#include "Pushrod_Driver.hpp"
#include "MotionController_Driver.hpp"
#include "MotionController_Porting.hpp"
#include "USBL_Driver.hpp"
#include "USBL_Porting.hpp"
#include "RosLogger.hpp"
#include "SerialHandles.hpp"
#include "SoftWatchdog.hpp"
#include "cmsis_os2.h"      // osThreadNew, osThreadAttr_t
#include "main.h"
#include "usart.h"

// --- 全局 Porting 实例（硬件适配） ---
static auv::porting::INS_Porting g_ins_port(&AUV_UART_INS, &AUV_UART_INS, ins_rx_buffer,
                                            512);
static auv::porting::MotionController_Porting g_motor_port(&AUV_UART_MOTOR);
static auv::porting::USBL_Porting g_usbl_port(
    &AUV_UART_USBL, auv::porting::usbl_rx_buffer,
    auv::porting::USBL_Porting::kBufferSize);

// --- 全局驱动实例 ---
namespace auv {
namespace peripheral {
auv::peripheral::INS_Driver ins_driver(auv::peripheral::InsPortOps{
    .ctx = &g_ins_port,
    .init = &auv::porting::INS_Porting::initPort,
    .read = &auv::porting::INS_Porting::readPort,
    .transmit = &auv::porting::INS_Porting::transmitPort,
    .getDiagnostics = &auv::porting::INS_Porting::diagnosticsPort,
});
auv::peripheral::MotionController_Driver
    motor_driver(auv::peripheral::MotorPortOps{
        .ctx = &g_motor_port,
        .transmitDMA = &auv::porting::MotionController_Porting::transmitDMA,
        .getTxPacket = &auv::porting::MotionController_Porting::getTxPacket,
    });
auv::peripheral::Depth_Sensor_Driver *depth_sensor =
    auv::porting::getDepthSensorDriver();
auv::peripheral::Pushrod_Driver *pushrod_driver =
    auv::porting::getPushrodDriver();
auv::peripheral::USBL_Driver usbl_driver(auv::peripheral::UsblPortOps{
    .ctx = &g_usbl_port,
    .init = &auv::porting::USBL_Porting::initPort,
    .read = &auv::porting::USBL_Porting::readPort,
    .getDiagnostics = &auv::porting::USBL_Porting::diagnosticsPort,
    .getTickMs = &auv::porting::USBL_Porting::getTickPort,
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
    .depth_sensor = auv::peripheral::depth_sensor,
    .pushrod_driver = auv::peripheral::pushrod_driver,
    .usbl_driver = &auv::peripheral::usbl_driver,
    .logger = &auv::component::g_ros_logger,
    .watchdog = &auv::component::g_soft_watchdog,
    .chassis = &auv::component::chassis,
};
} // namespace system
} // namespace auv
