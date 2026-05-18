import json
import os
import sys

def get_cpp_type(val):
    if isinstance(val, bool):
        return "ParamType::BOOL", "bool"
    if isinstance(val, int):
        return "ParamType::UINT32", "uint32_t"
    if isinstance(val, float):
        return "ParamType::FLOAT", "float"
    if isinstance(val, str):
        # 特殊处理枚举
        if "use_ms5837_z" in val or "use_ins" in val:
             return "ParamType::ENUM_Z", "ZDataSource"
        return "ParamType::STRING", "const char*"
    return None, None

def collect_params(data, prefix=""):
    params = []
    for k, v in data.items():
        path = f"{prefix}.{k}" if prefix else k
        if isinstance(v, dict):
            params.extend(collect_params(v, path))
        else:
            p_type, cpp_type = get_cpp_type(v)
            if p_type:
                params.append({
                    "path": path,
                    "key": k,
                    "type": p_type,
                    "cpp_type": cpp_type,
                    "val": v
                })
    return params

def gen_system_config(json_path, out_dir):
    with open(json_path, 'r') as f:
        config = json.load(f)

    os.makedirs(out_dir, exist_ok=True)
    header_path = os.path.join(out_dir, 'SystemConfig.hpp')

    # 生成结构体定义 (这里简化处理，手动定义核心结构以保证兼容性，元数据自动生成)
    # 实际上可以完全自动生成，但为了保持现有代码不崩溃，我们先定义好顶层结构
    
    content = """#ifndef __SYSTEM_CONFIG_HPP
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
"""

    # 生成注册表源文件 (SystemConfig.cpp)
    params = collect_params(config)
    
    cpp_content = f"""#include "SystemConfig.hpp"

namespace auv {{
namespace config {{

SystemConfig sys_config = {{
    .chassis = {{
        .planner_enabled = {str(config['chassis']['planner_enabled']).lower()},
        .x = {{ {config['chassis']['x']['pos_kp']}, {config['chassis']['x']['pos_ki']}, {config['chassis']['x']['pos_kd']}, {config['chassis']['x']['pos_i_limit']}, {config['chassis']['x']['pos_output_limit']}, {config['chassis']['x']['vel_kp']}, {config['chassis']['x']['vel_ki']}, {config['chassis']['x']['vel_kd']}, {config['chassis']['x']['vel_i_limit']}, {config['chassis']['x']['vel_output_limit']}, {config['chassis']['x']['max_v']}, {config['chassis']['x']['max_a']}, {config['chassis']['x']['mass']}, {config['chassis']['x']['drag']} }},
        .y = {{ {config['chassis']['y']['pos_kp']}, {config['chassis']['y']['pos_ki']}, {config['chassis']['y']['pos_kd']}, {config['chassis']['y']['pos_i_limit']}, {config['chassis']['y']['pos_output_limit']}, {config['chassis']['y']['vel_kp']}, {config['chassis']['y']['vel_ki']}, {config['chassis']['y']['vel_kd']}, {config['chassis']['y']['vel_i_limit']}, {config['chassis']['y']['vel_output_limit']}, {config['chassis']['y']['max_v']}, {config['chassis']['y']['max_a']}, {config['chassis']['y']['mass']}, {config['chassis']['y']['drag']} }},
        .z = {{ {config['chassis']['z']['pos_kp']}, {config['chassis']['z']['pos_ki']}, {config['chassis']['z']['pos_kd']}, {config['chassis']['z']['pos_i_limit']}, {config['chassis']['z']['pos_output_limit']}, {config['chassis']['z']['vel_kp']}, {config['chassis']['z']['vel_ki']}, {config['chassis']['z']['vel_kd']}, {config['chassis']['z']['vel_i_limit']}, {config['chassis']['z']['vel_output_limit']}, {config['chassis']['z']['max_v']}, {config['chassis']['z']['max_a']}, {config['chassis']['z']['mass']}, {config['chassis']['z']['drag']} }},
        .yaw = {{ {config['chassis']['yaw']['pos_kp']}, {config['chassis']['yaw']['pos_ki']}, {config['chassis']['yaw']['pos_kd']}, {config['chassis']['yaw']['pos_i_limit']}, {config['chassis']['yaw']['pos_output_limit']}, {config['chassis']['yaw']['vel_kp']}, {config['chassis']['yaw']['vel_ki']}, {config['chassis']['yaw']['vel_kd']}, {config['chassis']['yaw']['vel_i_limit']}, {config['chassis']['yaw']['vel_output_limit']}, {config['chassis']['yaw']['max_v']}, {config['chassis']['yaw']['max_a']}, {config['chassis']['yaw']['mass']}, {config['chassis']['yaw']['drag']} }}
    }},
    .ins = {{ {config['ins']['init_lat']}, {config['ins']['init_lon']} }},
    .soft_watchdog = {{ {config['soft_watchdog']['timeout_ms']}, {str(config['soft_watchdog']['check_microros']).lower()}, {str(config['soft_watchdog']['check_ins']).lower()}, {str(config['soft_watchdog']['check_depth']).lower()} }},
    .sensors = {{ ZDataSource::{ "USE_MS5837_Z" if config['z_data_sourse'] == 'use_ms5837_z' else ("USE_INS_PRESSURE_Z" if config['z_data_sourse'] == 'use_ins_pressure_z' else "USE_INS_INTEGRATED_Z") } }},
    .simulation = {{ {str(config['simulation']['hitl_enabled']).lower()}, {config['simulation']['mass']}, {config['simulation']['drag']}, {config['simulation']['thrust_k']} }}
}};

const ParamMeta SYSTEM_PARAMS[] = {{
"""
    
    for p in params:
        cpp_path = p['path']
        # 修正 JSON 路径到 C++ 成员路径的映射
        if cpp_path == "z_data_sourse": cpp_path = "sensors.z_data_source"
        # 不再需要旧的 PID 映射，因为现在是扁平化的 AxisConfig

        cpp_content += f'    {{"{p["path"]}", &sys_config.{cpp_path}, {p["type"]}}},\n'
    
    cpp_content += "    {NULL, NULL, ParamType::FLOAT}\n"
    cpp_content += "};\n\n"
    cpp_content += f"const size_t SYSTEM_PARAMS_COUNT = {len(params)};\n\n"
    cpp_content += "} // namespace config\n"
    cpp_content += "} // namespace auv\n"

    # 写入文件
    with open(header_path, 'w') as f:
        f.write(content)
    
    with open(os.path.join(out_dir, 'SystemConfig.cpp'), 'w') as f:
        f.write(cpp_content)

if __name__ == "__main__":
    if len(sys.argv) < 3:
        sys.exit(1)
    gen_system_config(sys.argv[1], sys.argv[2])
