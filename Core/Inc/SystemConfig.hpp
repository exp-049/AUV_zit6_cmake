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
    USE_MS5837_Z
};

// --- 自动生成的配置结构体 ---

struct PIDConfig {
    float kp;
    float ki;
    float kd;
    float i_limit;
    float output_limit;
    float dt;
};

struct ChassisProfile {
    float default_max_v;
    float default_max_a;
};

struct ChassisConfig {
    ChassisProfile profile;
    PIDConfig pos_pid;
    PIDConfig vel_pid;
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
    float mass;
    float drag;
    float thrust_k;
};

struct SystemConfig {
    ChassisConfig chassis;
    InsConfig ins;
    SoftWatchdogConfig soft_watchdog;
    SensorsConfig sensors;
    SimulationConfig simulation;
};

// 全局配置实例声明
extern SystemConfig sys_config;

// 参数注册表声明
extern const ParamMeta SYSTEM_PARAMS[];
extern const size_t SYSTEM_PARAMS_COUNT;

} // namespace config
} // namespace auv

#endif
