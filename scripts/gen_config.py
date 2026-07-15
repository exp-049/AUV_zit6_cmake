import json
import os
import sys
from datetime import datetime
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
        return "ParamType::UINT32", "uint32_t"
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

AXIS_FIELDS = [
    "pos_kp", "pos_ki", "pos_kd", "pos_i_limit", "pos_output_limit",
    "vel_kp", "vel_ki", "vel_kd", "vel_i_limit", "vel_output_limit",
    "max_v", "max_a", "mass", "drag",
]


def _format_literal(v):
    """将 Python 值格式化为 C++ 字面量"""
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, float):
        return f"{v}f"
    if isinstance(v, int):
        return str(v)
    if isinstance(v, str):
        return f'"{v}"'
    return str(v)


def _gen_axis_init(axis_dict):
    """生成单个 AxisConfig 的初始化字符串"""
    parts = []
    for field in AXIS_FIELDS:
        if field in axis_dict:
            parts.append(f".{field} = {_format_literal(axis_dict[field])}")
    return "{\n            " + ",\n            ".join(parts) + "\n        }"


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
    sys_cfg = config.get('system', {})
    depth_calc_board = bool(sys_cfg.get('depth_calc_board', {}).get('enabled', False))
    content = template.render(
        depth_calc_board=depth_calc_board)

    # 生成注册表源文件 (SystemConfig.cpp)
    if isinstance(config.get("types"), dict):
        type_overrides = config.get("types")
    else:
        type_overrides = load_type_overrides(json_path)
    config_for_params = {k: v for k, v in config.items() if k != "types"}
    # 剥离不需要注册为运行时参数的顶层键
    config_for_params.pop("firmware", None)  # 由手动注册处理
    # 剥离 system 中不需要注册为运行时参数的部分
    if "system" in config_for_params:
        sys_params = dict(config_for_params["system"])
        sys_params.pop("depth_calc_board", None)
        config_for_params["system"] = sys_params
    params = collect_params(config_for_params, type_overrides=type_overrides)

    z_src = str(sys_cfg.get('z_data_sourse', 'use_ins_integrated_z'))
    if z_src == 'use_ms5837_z':
        z_enum = 'USE_MS5837_Z'
    elif z_src in ('use_ins_pressure_z', 'use_manometer_z'):
        z_enum = 'USE_INS_PRESSURE_Z'
    else:
        z_enum = 'USE_INS_INTEGRATED_Z'

    # ---- 生成 chassis 初始化 ----
    chassis_cfg = config.get("chassis", {})
    chassis_lines = [f"        .planner_enabled = {_format_literal(chassis_cfg.get('planner_enabled', False))}"]
    for axis_name in ("x", "y", "z", "roll", "pitch", "yaw"):
        axis_dict = chassis_cfg.get(axis_name, {})
        chassis_lines.append(f"        .{axis_name} = {_gen_axis_init(axis_dict)}")
    chassis_init = ",\n".join(chassis_lines)

    # ---- 生成 ins 初始化 ----
    ins_cfg = config.get("ins", {})
    ins_init = f".init_lat = {_format_literal(ins_cfg.get('init_lat', 0.0))}, .init_lon = {_format_literal(ins_cfg.get('init_lon', 0.0))}"

    # ---- 自动生成固件版本 (YY_MM_DD-HHMM) ----
    fw_version = datetime.now().strftime("%y_%m_%d-%H%M")

    cpp_content = f"""#include "SystemConfig.hpp"

namespace auv {{
namespace config {{

SystemConfig sys_config = {{
    .system = {{
        .soft_watchdog = {{ {sys_cfg['soft_watchdog']['timeout_ms']}, {str(sys_cfg['soft_watchdog']['check_microros']).lower()}, {str(sys_cfg['soft_watchdog']['check_ins']).lower()}, {str(sys_cfg['soft_watchdog']['check_depth']).lower()} }},
        .sensors = {{ ZDataSource::{z_enum} }},
    }},
    .chassis = {{
{chassis_init}
    }},
    .ins = {{ {ins_init} }},
    .simulation = {{ {str(config['simulation']['hitl_enabled']).lower()}, {str(config['simulation']['sitl_enabled']).lower()}, {config['simulation']['mass']}, {config['simulation']['drag']}, {config['simulation']['thrust_k']}, {config['simulation']['metacentric_height']} }},
    .firmware_version = "{fw_version}"
}};

const ParamMeta SYSTEM_PARAMS[] = {{
"""
    
    for p in params:
        cpp_path = p['path']
        # 修正 JSON 路径到 C++ 成员路径的映射
        if cpp_path == "z_data_sourse" or cpp_path == "system.z_data_sourse":
            cpp_path = "system.sensors.z_data_source"
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
