### 常用命令与操作流程

本指南基于当前仓库实现，给出 ZIT6 AUV 的常用启动、解锁、参数配置与控制示例。

---

#### 0. 基础环境准备（建议别名）

建议在 `~/.bashrc` 中添加：

```bash
alias zit_src='source /home/doc049/dev/2026-auv-sub/AUV_zit6_cmake/install/setup.bash'
alias zit_agt='zit_src && MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600'
alias zit_fox='zit_src && ros2 launch foxglove_bridge foxglove_bridge_launch.xml'
alias zit_rqt='zit_src && rqt'
alias zit_cfg='zit_src && ros2 run upper_examples config_setter'
alias zit_gui='zit_src && ros2 run upper_examples gui'
alias zit_motion='zit_src && ros2 run upper_examples motion_control'
alias zit_hbt='zit_src && ros2 run upper_examples heartbeat'
```

可用的 `upper_examples` 入口当前包括：
- `config_setter`
- `xbox_control`
- `image_viewer`
- `image_publisher`
- `heartbeat`
- `motion_control`
- `gui`
- `hitl_test`

---

#### 1. 启动 Agent 并确认节点上线

```bash
zit_agt
```

另开终端：

```bash
zit_src
ros2 topic echo /zit6/state/status
```

建议同时观察：

```bash
ros2 topic echo /zit6/state/zithbt
ros2 topic echo /zit6/log
```

说明：
- `/zit6/state/zithbt` 当前约 1Hz。
- `/zit6/state/status` 当前约 10Hz。
- `ins_state` 为 `3` 或 `4` 且 INS 数据新鲜时，`navigation_ready` 才会为真。

---

#### 2. 发送 INS 控制命令

```bash
# 开 DVL
ros2 topic pub --once /zit6/cmd/ins std_msgs/msg/UInt8 '{data: 1}'

# 关 DVL
ros2 topic pub --once /zit6/cmd/ins std_msgs/msg/UInt8 '{data: 2}'

# 重启惯导
ros2 topic pub --once /zit6/cmd/ins std_msgs/msg/UInt8 '{data: 3}'

# 位置清零 / resetPosition()
ros2 topic pub --once /zit6/cmd/ins std_msgs/msg/UInt8 '{data: 4}'

# 按 config.json 中的 init_lat/init_lon 装订初始经纬度
ros2 topic pub --once /zit6/cmd/ins std_msgs/msg/UInt8 '{data: 5}'
```

---

#### 3. 在线查看 / 更新参数

查询全部参数：

```bash
ros2 service call /zit6/get_params zit6_interfaces/srv/GetParams '{paths: []}'
```

查询指定前缀：

```bash
ros2 service call /zit6/get_params zit6_interfaces/srv/GetParams '{paths: ["chassis.x", "simulation"]}'
```

JSON 方式更新参数：

```bash
ros2 service call /zit6/update_params zit6_interfaces/srv/UpdateParams '{json: "{\"chassis\":{\"x\":{\"vel_kp\":1.2}}}", paths: [], values: []}'
```

路径方式更新参数：

```bash
ros2 service call /zit6/update_params zit6_interfaces/srv/UpdateParams '{json: "", paths: ["chassis.x.vel_kp", "system.soft_watchdog.timeout_ms"], values: ["1.2", "4000"]}'
```

说明：
- 当前服务端同时支持 `json` 和 `paths + values` 两种形式。
- 参数更新成功后，底盘配置和软件看门狗配置会立即生效。

---

#### 4. 启动 ARM 心跳

正常解锁（需要导航有效）：

```bash
ros2 topic pub -r 10 /zit6/cmd/agxhbt std_msgs/msg/UInt32 '{data: 1}'
```

遥控模式解锁（绕过导航有效检查）：

```bash
ros2 topic pub -r 10 /zit6/cmd/agxhbt std_msgs/msg/UInt32 '{data: 3}'
```

当前代码阈值：
- 最少连续心跳：10 次
- 最少持续时间：1000ms
- 已解锁后掉心跳超时：500ms

停止发送该话题后，系统会在超时后自动上锁。

---

#### 5. 下发控制目标（Setpoint）

`ZitSetpoint` 当前字段：

```text
control_key: 模式 + 标志位
type_mask: skip mask，bit=1 表示跳过该轴；bit0..2/5 对应 X/Y/Z/Yaw，Roll/Pitch 始终旁路
x y z roll pitch yaw: 6-DOF payload（Roll/Pitch 为兼容字段，不参与控制）
seq: 序列号
```

##### 5.1 世界系位置控制：更新 X/Y/Z，保持当前 Yaw

```bash
ros2 topic pub --once /zit6/cmd/setpoint zit6_interfaces/msg/ZitSetpoint '{control_key: 0, type_mask: 32, x: 1.0, y: 0.0, z: -1.0, roll: 0.0, pitch: 0.0, yaw: 0.0, seq: 1}'
```

说明：
- `control_key = 0`：POSITION + WORLD + ABS
- `type_mask = 32`：跳过 Yaw，只更新 X/Y/Z；Roll/Pitch 始终旁路

##### 5.2 机体系速度控制：只控制前进速度 X

```bash
ros2 topic pub --once /zit6/cmd/setpoint zit6_interfaces/msg/ZitSetpoint '{control_key: 17, type_mask: 62, x: 0.2, y: 0.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0, seq: 2}'
```

说明：
- `17 = 0x10 | 0x01`：BODY + VELOCITY
- `type_mask = 62`：跳过 Y/Z/Yaw，只更新 X；Roll/Pitch 即使不置 mask 也始终旁路

##### 5.3 机体系位置增量：向前增量移动 0.5 m

```bash
ros2 topic pub --once /zit6/cmd/setpoint zit6_interfaces/msg/ZitSetpoint '{control_key: 48, type_mask: 62, x: 0.5, y: 0.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0, seq: 3}'
```

说明：
- `48 = 0x10 | 0x20`：BODY + INCREMENT + POSITION
- 当前代码中 body 增量位置会先旋转到世界系后再叠加

##### 5.4 直接推力控制：只控制 Fx

```bash
ros2 topic pub --once /zit6/cmd/setpoint zit6_interfaces/msg/ZitSetpoint '{control_key: 18, type_mask: 62, x: 0.1, y: 0.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0, seq: 4}'
```

说明：
- `18 = 0x10 | 0x02`：BODY + ACTUATOR
- 直接写入机体系推力目标

---

#### 6. 状态观测

```bash
# 世界系位姿 [x, y, z, roll, pitch, yaw]
ros2 topic echo /zit6/state/pos

# 机体系速度 [u, v, w, p, q, r]
ros2 topic echo /zit6/state/vel

# 6-DOF 输出 [fx, fy, fz, mroll, mpitch, myaw]
ros2 topic echo /zit6/state/thr

# 汇总状态
ros2 topic echo /zit6/state/status

# 事件日志
ros2 topic echo /zit6/log
```

当前代码实现频率：
- `/zit6/state/pos`：约 30Hz
- `/zit6/state/vel`：约 50Hz
- `/zit6/state/thr`：约 30Hz
- `/zit6/state/status`：10Hz
- `/zit6/state/zithbt`：1Hz

---

#### 7. 仿真模式

当前固件支持：
- SITL：通过 `/zit6/sim/nav` 注入 12 维导航数据
- HITL：由固件内部 `HitlSimulator` 驱动

SITL 输入格式：

```text
[x, y, z, roll, pitch, yaw, vx, vy, vz, p, q, r]
```

可以使用：

```bash
zit_src
ros2 run upper_examples hitl_test
```

---

#### 8. 常见排查

1. `setpoint` 发了没反应
   - 先检查 `is_armed` 是否为 `true`
   - 再检查 `navigation_ready` 是否为 `true`（POSITION / VELOCITY 模式要求）
   - 检查 `type_mask` 是否把目标轴跳过了

2. 解锁失败
   - 检查 `/zit6/cmd/agxhbt` 是否稳定 10Hz
   - 检查 `ins_state` 是否到 3 或 4
   - 必要时用 `data: 3` 进入遥控模式验证链路

3. 参数没生效
   - 确认 `update_params` 返回 `success: true`
   - 再用 `get_params` 回读确认
