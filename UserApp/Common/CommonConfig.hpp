/**
 * @file CommonConfig.hpp
 * @brief 全局通用数据结构与常量定义
 *
 * 职责：
 * 1. 定义跨模块传输的导航状态 (auv::common::NavState)。
 * 2. 定义控制层级枚举。
 * 3. 提供数学常数与物理参数。
 */

#ifndef __COMMON_CONFIG_HPP
#define __COMMON_CONFIG_HPP

#include <stdint.h>

namespace auv {
namespace common {

/**
 * @struct auv::common::NavState
 * @brief 6-DOF 导航状态结构体
 */
struct NavState {
    // 位置项 (NED系/Geographic)
    float x = 0.0f;      ///< 北向位置 (North, m)
    float y = 0.0f;      ///< 东向位置 (East, m)
    float z = 0.0f;      ///< 深度 (Down/Depth, m)
    float roll = 0.0f;   ///< 横滚角 (rad)
    float pitch = 0.0f;  ///< 俯仰角 (rad)
    float yaw = 0.0f;    ///< 航向角 (rad, 0表示北, 顺时针为正)

    // 速度项 (机体系/Body)
    float vx = 0.0f;     ///< 前进速度 (Surge, m/s)
    float vy = 0.0f;     ///< 横移速度 (Sway, m/s)
    float vz = 0.0f;     ///< 垂直速度 (Heave, m/s)
    float vroll = 0.0f;  ///< 横滚角速度 (rad/s)
    float vpitch = 0.0f; ///< 俯仰角速度 (rad/s)
    float vyaw = 0.0f;   ///< 航向角速度 (Yaw rate, rad/s)

    // 状态标志
    uint8_t imu_state = 0; ///< 惯导模式 (0:待机, 1:粗对准, 2:精对准, 3:SINS/GPS/DVL, 4:SINS/DVL, 5:MRU)
    uint8_t dvl_state = 0; ///< DVL有效性标志 (1:有效, 0:无效)
    double lat = 0.0;      ///< 纬度 (Latitude, deg)
    double lon = 0.0;      ///< 经度 (Longitude, deg)
    uint32_t timestamp = 0; ///< 系统毫秒时间戳
};

/**
 * @enum auv::common::ControlLevel
 * @brief 任务层下发的控制强度级
 */
enum class ControlLevel : uint8_t {
    NONE = 0,     ///< 待机/锁定模式
    POSITION = 1, ///< 位置闭环
    VELOCITY = 2, ///< 速度闭环
    ACTUATOR = 3  ///< 直接推力控制 (通过 mixer)
};

/**
 * @struct OffboardSetpoint
 * @brief 上位机 CompactSetpoint 的内部解包结构
 */
struct OffboardSetpoint {
    auv::common::ControlLevel level; ///< 期望层级
    float data[4];      ///< 目标数据 [x, y, z, yaw] 或 [vx, vy, vz, vyaw]
    uint32_t type_mask; ///< 类型掩码
};

/**
 * @struct Constants
 * @brief 系统数学常数
 */
struct Constants {
    static constexpr float CONTROL_FREQ = 100.0f;       ///< 控制频率 (Hz)
    static constexpr uint32_t CONTROL_PERIOD_MS = 10;   ///< 控制周期 (ms)
    static constexpr float DEG2RAD = 0.0174532925f;    ///< 角度转弧度
    static constexpr float RAD2DEG = 57.2957795f;      ///< 弧度转角度
};

} // namespace common
} // namespace auv

#endif
