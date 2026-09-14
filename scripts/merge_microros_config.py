#!/usr/bin/env python3
"""
将 microros_config.meta（简化格式）合并到 colcon.meta（完整格式）中。

用法：
    python3 merge_microros_config.py <colcon.meta路径> <microros_config.meta路径> [输出路径]
    
    如果不指定输出路径，则原地覆盖 colcon.meta。

microros_config.meta 格式（简化 KEY=VALUE 形式）：
{
    "rmw_microxrcedds": {
        "RMW_UXRCE_MAX_PUBLISHERS": 10,
        "RMW_UXRCE_MAX_SUBSCRIPTIONS": 8
    }
}

colcon.meta 中的对应项将被更新或追加。
"""

import json
import sys


def load_json(path):
    with open(path, 'r') as f:
        return json.load(f)


def save_json(data, path):
    with open(path, 'w') as f:
        json.dump(data, f, indent=4)
        f.write('\n')


def merge_overrides(base, overrides):
    """
    将 overrides（简化格式）合并到 base（colcon.meta 格式）中。
    对 overrides 中的每个包，将其 KEY=VALUE 对转换为 -DKEY=VALUE 格式，
    更新 base 中对应包的 cmake-args（替换同 KEY 的旧值，新增不存在的 KEY）。
    """
    names = base.setdefault('names', {})

    for pkg, params in overrides.items():
        if pkg not in names:
            names[pkg] = {}
        entry = names[pkg]
        if 'cmake-args' not in entry:
            entry['cmake-args'] = []

        args = entry['cmake-args']
        # 解析当前 cmake-args，保留 -D 和 -U 前缀的 KEY=VALUE 对，其余保持原位
        prefix_map = {}    # key -> (prefix, value)
        positional = []    # 非 -D/-U 参数，保持原位
        order = []         # 保持 key 的出现顺序
        for arg in args:
            prefix = ''
            rest = arg
            if arg.startswith('-D'):
                prefix = '-D'
                rest = arg[2:]
            elif arg.startswith('-U'):
                prefix = '-U'
                rest = arg[2:]

            if '=' in rest and prefix:
                k, v = rest.split('=', 1)
                prefix_map[k] = (prefix, v)
                order.append(k)
            else:
                positional.append(arg)

        # 应用覆盖层的值（覆盖层使用 -D 前缀）
        for k, v in params.items():
            val = str(v) if not isinstance(v, str) else v
            if k in prefix_map:
                old_prefix, _ = prefix_map[k]
                prefix_map[k] = (old_prefix, val)  # 保留原前缀
            else:
                prefix_map[k] = ('-D', val)        # 新增用 -D
                order.append(k)

        # 重建 cmake-args：按原顺序，新增的排在最后
        entry['cmake-args'] = positional + [f'{prefix_map[k][0]}{k}={prefix_map[k][1]}' for k in order]

    return base


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    colcon_meta_path = sys.argv[1]
    config_path = sys.argv[2]
    output_path = sys.argv[3] if len(sys.argv) > 3 else colcon_meta_path

    base = load_json(colcon_meta_path)
    overrides = load_json(config_path)

    merged = merge_overrides(base, overrides)
    save_json(merged, output_path)

    print(f'Merged {config_path} -> {output_path}')


if __name__ == '__main__':
    main()
