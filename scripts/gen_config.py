import json
import os
import sys
from jinja2 import Environment, FileSystemLoader

TYPE_MAP = {
    "uint32": ("ParamType::UINT32", "uint32_t"),
    "uint32_t": ("ParamType::UINT32", "uint32_t"),
    "int32": ("ParamType::INT32", "int32_t"),
    "int32_t": ("ParamType::INT32", "int32_t"),
    "float": ("ParamType::FLOAT", "float"),
    "float32": ("ParamType::FLOAT", "float"),
    "bool": ("ParamType::BOOL", "bool"),
    "boolean": ("ParamType::BOOL", "bool"),
    "string": ("ParamType::STRING", "const char*"),
    "cstring": ("ParamType::STRING", "const char*"),
    "enum_z": ("ParamType::ENUM_Z", "ZDataSource"),
    "zdatasource": ("ParamType::ENUM_Z", "ZDataSource"),
}

def load_type_overrides(config_path):
    base_dir = os.path.dirname(config_path)
    type_path = os.path.join(base_dir, "config.types.json")
    if not os.path.exists(type_path):
        return {}
    try:
        with open(type_path, 'r') as f:
            data = json.load(f)
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}

def resolve_type_override(override):
    if override is None:
        return None, None
    if isinstance(override, dict):
        t = override.get("type") or override.get("param_type")
        cpp_override = override.get("cpp_type")
    else:
        t = override
        cpp_override = None
    if not t:
        return None, None
    t_norm = str(t).strip().lower()
    if t_norm.startswith("enum"):
        return "ParamType::ENUM_Z", (cpp_override or "ZDataSource")
    mapped = TYPE_MAP.get(t_norm)
    if not mapped:
        return None, None
    return mapped[0], (cpp_override or mapped[1])

def get_cpp_type(val, path="", type_overrides=None):
    if type_overrides and path in type_overrides:
        p_type, cpp_type = resolve_type_override(type_overrides[path])
        if p_type:
            return p_type, cpp_type
    if isinstance(val, bool):
        return "ParamType::BOOL", "bool"
    if isinstance(val, int):
        return "ParamType::FLOAT", "float"
    if isinstance(val, float):
        return "ParamType::FLOAT", "float"
    if isinstance(val, str):
        # 特殊处理枚举
        enum_vals = {
            "use_ms5837_z",
            "use_ins_integrated_z",
            "use_ins_pressure_z",
            "use_manometer_z",
        }
        if val in enum_vals:
            return "ParamType::ENUM_Z", "ZDataSource"
        return "ParamType::STRING", "const char*"
    return None, None

def collect_params(data, prefix="", type_overrides=None):
    params = []
    for k, v in data.items():
        path = f"{prefix}.{k}" if prefix else k
        if isinstance(v, dict):
            params.extend(collect_params(v, path, type_overrides))
        else:
            p_type, cpp_type = get_cpp_type(v, path, type_overrides)
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

    # 使用 Jinja2 模板生成结构体定义头文件
    script_dir = os.path.dirname(os.path.abspath(__file__))
    template_dir = os.path.join(script_dir, 'templates')
    env = Environment(loader=FileSystemLoader(template_dir))
    template = env.get_template('SystemConfig.hpp.j2')
    content = template.render()

    # 生成注册表源文件 (SystemConfig.cpp)
    if isinstance(config.get("types"), dict):
        type_overrides = config.get("types")
    else:
        type_overrides = load_type_overrides(json_path)
    config_for_params = {k: v for k, v in config.items() if k != "types"}
    params = collect_params(config_for_params, type_overrides=type_overrides)

    z_src = str(config.get('z_data_sourse', 'use_ins_integrated_z'))
    if z_src == 'use_ms5837_z':
        z_enum = 'USE_MS5837_Z'
    elif z_src in ('use_ins_pressure_z', 'use_manometer_z'):
        z_enum = 'USE_INS_PRESSURE_Z'
    else:
        z_enum = 'USE_INS_INTEGRATED_Z'

    cpp_content = f"""#include "SystemConfig.hpp"

namespace auv {{
namespace config {{

SystemConfig sys_config = {{
    .chassis = {{
        .planner_enabled = {str(config['chassis']['planner_enabled']).lower()},
        .x = {{ {config['chassis']['x']['pos_kp']}, {config['chassis']['x']['pos_ki']}, {config['chassis']['x']['pos_kd']}, {config['chassis']['x']['pos_i_limit']}, {config['chassis']['x']['pos_output_limit']}, {config['chassis']['x']['vel_kp']}, {config['chassis']['x']['vel_ki']}, {config['chassis']['x']['vel_kd']}, {config['chassis']['x']['vel_i_limit']}, {config['chassis']['x']['vel_output_limit']}, {config['chassis']['x']['max_v']}, {config['chassis']['x']['max_a']}, {config['chassis']['x']['mass']}, {config['chassis']['x']['drag']} }},
        .y = {{ {config['chassis']['y']['pos_kp']}, {config['chassis']['y']['pos_ki']}, {config['chassis']['y']['pos_kd']}, {config['chassis']['y']['pos_i_limit']}, {config['chassis']['y']['pos_output_limit']}, {config['chassis']['y']['vel_kp']}, {config['chassis']['y']['vel_ki']}, {config['chassis']['y']['vel_kd']}, {config['chassis']['y']['vel_i_limit']}, {config['chassis']['y']['vel_output_limit']}, {config['chassis']['y']['max_v']}, {config['chassis']['y']['max_a']}, {config['chassis']['y']['mass']}, {config['chassis']['y']['drag']} }},
        .z = {{ {config['chassis']['z']['pos_kp']}, {config['chassis']['z']['pos_ki']}, {config['chassis']['z']['pos_kd']}, {config['chassis']['z']['pos_i_limit']}, {config['chassis']['z']['pos_output_limit']}, {config['chassis']['z']['vel_kp']}, {config['chassis']['z']['vel_ki']}, {config['chassis']['z']['vel_kd']}, {config['chassis']['z']['vel_i_limit']}, {config['chassis']['z']['vel_output_limit']}, {config['chassis']['z']['max_v']}, {config['chassis']['z']['max_a']}, {config['chassis']['z']['mass']}, {config['chassis']['z']['drag']} }},
        .roll = {{ {config['chassis']['roll']['pos_kp']}, {config['chassis']['roll']['pos_ki']}, {config['chassis']['roll']['pos_kd']}, {config['chassis']['roll']['pos_i_limit']}, {config['chassis']['roll']['pos_output_limit']}, {config['chassis']['roll']['vel_kp']}, {config['chassis']['roll']['vel_ki']}, {config['chassis']['roll']['vel_kd']}, {config['chassis']['roll']['vel_i_limit']}, {config['chassis']['roll']['vel_output_limit']}, {config['chassis']['roll']['max_v']}, {config['chassis']['roll']['max_a']}, {config['chassis']['roll']['mass']}, {config['chassis']['roll']['drag']} }},
        .pitch = {{ {config['chassis']['pitch']['pos_kp']}, {config['chassis']['pitch']['pos_ki']}, {config['chassis']['pitch']['pos_kd']}, {config['chassis']['pitch']['pos_i_limit']}, {config['chassis']['pitch']['pos_output_limit']}, {config['chassis']['pitch']['vel_kp']}, {config['chassis']['pitch']['vel_ki']}, {config['chassis']['pitch']['vel_kd']}, {config['chassis']['pitch']['vel_i_limit']}, {config['chassis']['pitch']['vel_output_limit']}, {config['chassis']['pitch']['max_v']}, {config['chassis']['pitch']['max_a']}, {config['chassis']['pitch']['mass']}, {config['chassis']['pitch']['drag']} }},
        .yaw = {{ {config['chassis']['yaw']['pos_kp']}, {config['chassis']['yaw']['pos_ki']}, {config['chassis']['yaw']['pos_kd']}, {config['chassis']['yaw']['pos_i_limit']}, {config['chassis']['yaw']['pos_output_limit']}, {config['chassis']['yaw']['vel_kp']}, {config['chassis']['yaw']['vel_ki']}, {config['chassis']['yaw']['vel_kd']}, {config['chassis']['yaw']['vel_i_limit']}, {config['chassis']['yaw']['vel_output_limit']}, {config['chassis']['yaw']['max_v']}, {config['chassis']['yaw']['max_a']}, {config['chassis']['yaw']['mass']}, {config['chassis']['yaw']['drag']} }}
    }},
    .ins = {{ {config['ins']['init_lat']}, {config['ins']['init_lon']} }},
    .soft_watchdog = {{ {config['soft_watchdog']['timeout_ms']}, {str(config['soft_watchdog']['check_microros']).lower()}, {str(config['soft_watchdog']['check_ins']).lower()}, {str(config['soft_watchdog']['check_depth']).lower()} }},
    .sensors = {{ ZDataSource::{z_enum} }},
    .simulation = {{ {str(config['simulation']['hitl_enabled']).lower()}, {str(config['simulation']['sitl_enabled']).lower()}, {config['simulation']['mass']}, {config['simulation']['drag']}, {config['simulation']['thrust_k']}, {config['simulation']['metacentric_height']} }},
    .firmware_version = "26_06_21"
}};

const ParamMeta SYSTEM_PARAMS[] = {{
"""
    
    for p in params:
        cpp_path = p['path']
        # 修正 JSON 路径到 C++ 成员路径的映射
        if cpp_path == "z_data_sourse": cpp_path = "sensors.z_data_source"
        # 不再需要旧的 PID 映射，因为现在是扁平化的 AxisConfig

        cpp_content += f'    {{"{p["path"]}", &sys_config.{cpp_path}, {p["type"]}}},\n'
    
    # 手动添加非 JSON 配置项的注册
    cpp_content += '    {"firmware.version", sys_config.firmware_version, ParamType::STRING},\n'
    cpp_content += "    {NULL, NULL, ParamType::FLOAT}\n"
    cpp_content += "};\n\n"
    cpp_content += f"const size_t SYSTEM_PARAMS_COUNT = {len(params) + 1};\n\n"
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
