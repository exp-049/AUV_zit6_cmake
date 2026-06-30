#include "AppContext.hpp"
#include "ChassisManager.hpp"
#include "INS_Driver.hpp"
#include "INS_Porting.hpp" // ins_rx_buffer + INS_Porting
#include "MS5837_Driver.hpp"
#include "MotionController_Driver.hpp"
#include "MotionController_Porting.hpp"
#include "RosLogger.hpp"
#include "SoftWatchdog.hpp"
#include "SystemConfig.hpp" // USE_DEPTH_CALC_BOARD (由 gen_config.py 生成)
#include "cmsis_os2.h"      // osThreadNew, osThreadAttr_t
#include "i2c.h"
#include "main.h"
#include "usart.h"

// --- 深度传感器方案选择 ---
#ifdef USE_DEPTH_CALC_BOARD
// ======== UART 解算板方案 ========
#include "DepthCalcBoard_Porting.hpp"
#include "UART_DepthBackend.hpp"

// 构造顺序：Porting → Backend → 链接
static auv::porting::DepthCalcBoard_Porting g_depth_port(&huart4);
static auv::peripheral::UART_DepthBackend
    g_depth_backend(auv::peripheral::UartPortOps{
        .ctx = &g_depth_port,
        .transmit = &auv::porting::DepthCalcBoard_Porting::transmitPort,
        .poll = &auv::porting::DepthCalcBoard_Porting::pollPort,
        .startRx = &auv::porting::DepthCalcBoard_Porting::startRxPort,
    });
// 链接 Porting → Backend（两个对象都已构造，地址安全）
__attribute__((unused)) static bool g_depth_link =
    (g_depth_port.setBackend(&g_depth_backend), true);

#else
// ======== I2C MS5837 直连方案 ========
#include "I2C_DepthBackend.hpp"
#include "MS5837_Porting.hpp"

static auv::porting::MS5837_Porting g_depth_port(&hi2c1, 0x76 << 1);
static auv::peripheral::I2C_DepthBackend g_depth_backend(
    auv::peripheral::DepthPortOps{
        .ctx = &g_depth_port,
        .writeByte = &auv::porting::MS5837_Porting::writePort,
        .readByte = &auv::porting::MS5837_Porting::readPortByte,
        .read = &auv::porting::MS5837_Porting::readPort,
        .delay = &auv::porting::MS5837_Porting::delayPort,
        .start = &auv::porting::MS5837_Porting::startPort,
    },
    0x76 << 1);
// 链接 Porting → Backend
__attribute__((unused)) static bool g_depth_i2c_link =
    (g_depth_port.setBackend(&g_depth_backend), true);
#endif

// --- 全局 Porting 实例（硬件适配） ---
static auv::porting::INS_Porting g_ins_port(&huart7, &huart7, ins_rx_buffer,
                                            512);
static auv::porting::MotionController_Porting g_motor_port(&huart6);

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
auv::peripheral::MS5837_Driver depth_sensor(&g_depth_backend);
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
