#ifndef __SYSTEM_CONTEXT_HPP
#define __SYSTEM_CONTEXT_HPP

#include "INS_Driver.hpp"
#include "MotionController_Driver.hpp"
#include "ChassisManager.hpp"
#include "MS5837_Class.hpp"
#include "stm32h7xx_hal.h"

// --- 硬件句柄 extern ---
extern "C" {
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart7;  // INS UART
extern IWDG_HandleTypeDef hiwdg1;
extern I2C_HandleTypeDef hi2c1;
}

// --- DMA 缓冲区 extern ---
extern uint8_t ins_rx_buffer[512];
extern auv::device::MotionController_Driver::ThrustPacket motor_tx_packet;

// --- 全局实例 extern ---
namespace auv {
namespace device {
    extern INS_Driver ins_driver;
    extern MotionController_Driver motor_driver;
    extern MS5837 depth_sensor;
}
namespace control {
    extern ChassisManager chassis;
}
}

namespace auv {
namespace system {

/**
 * @struct NavStatus
 * @brief 提取自原 NavState 的非实时状态标志和低频定位信息
 */
struct NavStatus {
    uint8_t imu_state = 0;   ///< 惯导模式
    uint8_t dvl_state = 0;   ///< DVL有效性标志
    double lat = 0.0;        ///< 纬度 (deg)
    double lon = 0.0;        ///< 经度 (deg)
    uint32_t timestamp = 0;  ///< 系统毫秒时间戳
};

/**
 * @class SystemContext
 * @brief 系统/状态机上下文（安全解锁状态、心跳监测、规划器控制标志等）
 */
class SystemContext {
public:
    // 解锁与安全监测变量
    bool is_system_armed = false;
    uint32_t arm_heartbeat_count = 0;
    uint32_t last_arm_heartbeat_ms = 0;
    uint32_t last_arm_heartbeat_data = 0;
    uint32_t arm_start_ms = 0;

    // 规划器启用与状态变量
    bool is_planner_active = false;
    volatile bool planner_replan_flag = false;

    // 低频导航状态与标志
    NavStatus nav_status{};

    // 校验导航数据是否有效
    bool getNavigationValid() const;
};

extern SystemContext system_context;

} // namespace system
} // namespace auv

#endif // __SYSTEM_CONTEXT_HPP
