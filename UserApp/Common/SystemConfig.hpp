#ifndef __SYSTEM_CONFIG_HPP
#define __SYSTEM_CONFIG_HPP

#include <stdint.h>
#include <stddef.h>

namespace auv {
namespace config {

enum class ParamType {
    FLOAT,
    UINT32,
    INT32,
    BOOL,
    STRING,
    ENUM_Z
};

struct ParamMeta {
    const char* path;
    void* ptr;
    ParamType type;
};

// 传感器枚举定义
enum class ZDataSource {
    USE_INS_INTEGRATED_Z,
    USE_MS5837_Z,
    USE_INS_PRESSURE_Z
};

// --- 自动生成的配置结构体 ---

struct AxisConfig {
    float pos_kp;
    float pos_ki;
    float pos_kd;
    float pos_i_limit;
    float pos_output_limit;
    float vel_kp;
    float vel_ki;
    float vel_kd;
    float vel_i_limit;
    float vel_output_limit;
    float max_v;
    float max_a;
    float mass;
    float drag;
};

struct ChassisConfig {
    bool planner_enabled;
    AxisConfig x;
    AxisConfig y;
    AxisConfig z;
    AxisConfig roll;
    AxisConfig pitch;
    AxisConfig yaw;
};

struct SoftWatchdogConfig {
    uint32_t timeout_ms;
    bool check_microros;
    bool check_ins;
    bool check_depth;
};

struct InsConfig {
    float init_lat;
    float init_lon;
};

struct SensorsConfig {
    ZDataSource z_data_source;
};

struct SimulationConfig {
    bool hitl_enabled;
    bool sitl_enabled;
    float mass;
    float drag;
    float thrust_k;
    float metacentric_height;
};

struct SystemConfig {
    ChassisConfig chassis;
    InsConfig ins;
    SoftWatchdogConfig soft_watchdog;
    SensorsConfig sensors;
    SimulationConfig simulation;
    char firmware_version[32];
};

// 全局配置实例声明
extern SystemConfig sys_config;

// 参数注册表声明
extern const ParamMeta SYSTEM_PARAMS[];
extern const size_t SYSTEM_PARAMS_COUNT;

} // namespace config
} // namespace auv

#endif
