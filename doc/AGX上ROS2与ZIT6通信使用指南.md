# AGX 上使用 ROS2 与 ZIT6 MCU 通信指南（详细版）

本文面向在 NVIDIA AGX（Ubuntu + ROS2）上联调 ZIT6 MCU 的场景，重点解释：
- 如何建立 AGX ↔ MCU 通信链路；
- 每个控制话题应如何发送；
- **消息里的数值（尤其是整数命令）发出去后，MCU 端到底会做什么**。

---

## 1. 前置条件

- AGX 已安装 ROS2（建议与项目依赖一致的发行版）。
- 已安装 Micro XRCE-DDS Agent（命令名通常为 `MicroXRCEAgent`）。
- ZIT6 通过 USB-串口接入 AGX（示例设备：`/dev/ttyUSB0`）。
- 已获取本仓库并完成接口包构建（至少包含 `zit6_interfaces`、`upper_examples`）。

> 建议先确认串口权限：
>
> ```bash
> ls -l /dev/ttyUSB0
> ```

---

## 2. 工作空间准备

假设工作空间为 `<ws>`：

```bash
cd <ws>
source /opt/ros/<distro>/setup.bash
colcon build --packages-select zit6_interfaces upper_examples --symlink-install
source install/setup.bash
```

可选：在 `~/.bashrc` 增加别名

```bash
alias zit_src='source <ws>/install/setup.bash'
alias zit_agt='zit_src && MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600'
```

---

## 3. 建立 AGX 与 MCU 通信（Agent）

启动 micro-ROS Agent：

```bash
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

成功后，MCU 节点可被 AGX ROS2 发现，并开始交换 `/zit6/cmd/*` 与 `/zit6/state/*`。

---

## 4. 先看总体：消息从 AGX 到 MCU 后的处理路径

- `/zit6/cmd/*` 由 MCU 侧 `MicroRosTask` 订阅并回调处理。
- 一部分命令（如 `ins/light/servo`）会**立即转发到底层驱动**。
- `setpoint` 会先经过安全门控：
  - 若 `is_system_armed=false`，直接忽略（不生效）；
  - 若已解锁，才会进入位置/速度/推力控制逻辑。
- `agxhbt` 只更新“心跳状态变量”；真正“是否解锁”由 `ControlTask` 周期判断。

---

## 5. 控制话题详细说明（含“数值 -> 结果”）

## 5.1 `/zit6/cmd/ins` (`std_msgs/msg/UInt8`)

这是你提到的重点：**比如 `cmd/ins = 1` 会发生什么**。

MCU 侧映射如下：

| 发送值 `data` | MCU 动作 | 结果说明 |
|---|---|---|
| 1 | `setDvlPower(true)` | 向 INS 发送命令 `cmd_id=0x03, value=0x01`，请求**打开 DVL 供电** |
| 2 | `setDvlPower(false)` | 向 INS 发送命令 `cmd_id=0x03, value=0x00`，请求**关闭 DVL 供电** |
| 3 | `restart()` | 向 INS 发送 `cmd_id=0x04`，请求**重启惯导** |
| 4 | `resetPosition()` | 向 INS 发送 `cmd_id=0x02`，请求**位置归零/重置** |
| 5 | `setInitialPosition(init_lat, init_lon)` | 向 INS 发送 `cmd_id=0x20`，写入配置中的初始经纬度 |
| 其他值 | 无处理 | `switch` 无匹配，命令被忽略 |

> 关键细节：这些 INS 命令最终通过 INS 驱动封包后从 UART 发出，且会重复发送（驱动层尝试 3 次）。

示例（你问的场景）：

```bash
ros2 topic pub --once /zit6/cmd/ins std_msgs/msg/UInt8 "{data: 1}"
```

**预期行为**：MCU 收到后调用 `setDvlPower(true)`，向 INS 下发“开 DVL 电源”指令。

---

## 5.2 `/zit6/cmd/agxhbt` (`std_msgs/msg/UInt32`)

该消息用于“心跳+解锁模式选择”。

### A) `data` 常用取值

- `1`：普通解锁模式（要求导航有效，或 HITL 仿真启用）。
- `3`：遥控/推力解锁模式（可绕过导航有效性检查）。
- 其他值：不会满足当前解锁条件（通常无法解锁）。

### B) 解锁判据（MCU 实际逻辑）

MCU 不会“收到一包就解锁”，需要同时满足：

1. 心跳累计至少 **10 包**；
2. 心跳持续时间至少 **1 秒**；
3. 且 `data` 满足下列之一：
   - `data==3`；
   - `data==1` 且 `navigation_ready=true`（或 HITL 模式开启）。

### C) 掉心跳后的行为

- 已解锁状态下，若超过 **500ms** 没有新心跳：强制失锁；
- 失锁后推进器输出回到 0（安全态）；
- 未解锁状态下，若心跳中断超过 **1000ms**，解锁累计计数会清零。

示例：

```bash
# 普通解锁（要求导航有效）
ros2 topic pub -r 10 /zit6/cmd/agxhbt std_msgs/msg/UInt32 "{data: 1}"

# 遥控/推力解锁（谨慎）
ros2 topic pub -r 10 /zit6/cmd/agxhbt std_msgs/msg/UInt32 "{data: 3}"
```

---

## 5.3 `/zit6/cmd/setpoint` (`zit6_interfaces/msg/ZitSetpoint`)

字段：`control_key`, `type_mask`, `x y z yaw`, `seq`。

### A) `control_key`（模式 + 标志位）

- 低 2 位：
  - `0=POS` 位置环
  - `1=VEL` 速度环
  - `2=FORCE` 推力/执行器层
- `0x10`：Body 坐标系标志
- `0x20`：增量模式标志

常用组合：
- `0`：世界系位置控制
- `1`：世界系速度控制
- `2`：推力控制
- `16`（0x10）：Body 位置（相对当前姿态解释）
- `17`（0x11）：Body 速度
- `48`（0x30）：Body + 增量位置
- `50`（0x32）：Body + 增量推力

### B) `type_mask`（按位“屏蔽”轴）

> 这里容易误解：在当前固件实现中，**位为 1 表示该轴不更新/被屏蔽**，不是“启用”。

- bit0=1：屏蔽 X
- bit1=1：屏蔽 Y
- bit2=1：屏蔽 Z
- bit3=1：屏蔽 Yaw

示例：
- `type_mask=0`：四轴都更新；
- `type_mask=7`（1|2|4）：X/Y/Z 被屏蔽，仅 Yaw 更新；
- `type_mask=8`：仅屏蔽 Yaw，其余轴更新。

### C) 收到 setpoint 后会发生什么

1. 先检查：NaN/Inf、解锁状态、控制模式合法性；
2. 不满足则直接丢弃；
3. 满足后按 POS/VEL/FORCE 分支更新目标；
4. 若涉及 Body 坐标，内部会做坐标转换；
5. 若增量模式，按当前目标累加；
6. 控制输出由 `ControlTask` 周期计算并发布到推进器。

---

## 5.4 `/zit6/cmd/light` (`std_msgs/msg/UInt8`)

- MCU 收到后直接调用 `setLightState(data)`，并封包发给运动控制板。
- 当前链路中该值不做范围校验，建议按项目约定发送（常见约定：1/2/3）。

示例：

```bash
ros2 topic pub --once /zit6/cmd/light std_msgs/msg/UInt8 "{data: 1}"
```

---

## 5.5 `/zit6/cmd/servo` (`std_msgs/msg/Float32`)

- MCU 收到后直接调用 `setServoAngle(data)`，下发给运动控制板。
- 当前实现不做角度钳位，建议上位机自行限制安全范围。

示例：

```bash
ros2 topic pub --once /zit6/cmd/servo std_msgs/msg/Float32 "{data: 0.2}"
```

---

## 6. 状态反馈如何看是否生效

重点关注 `/zit6/state/status`：
- `is_armed`：是否已解锁；
- `arm_mode`：最近收到的心跳值；
- `navigation_ready`：导航是否有效；
- `control_level`：当前控制层级。

同时看：
- `/zit6/state/pos`：位置/姿态反馈；
- `/zit6/state/vel`：速度反馈；
- `/zit6/state/thr`：推力反馈。

---

## 7. 推荐联调顺序（详细）

1. 启动 Agent；
2. 观察 `/zit6/state/status` 是否刷新；
3. 按需发送 `/zit6/cmd/ins`（如 `1` 打开 DVL）；
4. 持续发送 `/zit6/cmd/agxhbt`（10Hz）；
5. 等 `is_armed=true` 后发送 `/zit6/cmd/setpoint`；
6. 用 `pos/vel/thr/status` 回看行为是否符合预期。

---

## 8. 常见问题

1. **`cmd/ins` 发了没反应**
   - 确认 AGX 侧话题名和类型正确；
   - 确认串口链路与 INS 在线；
   - 检查是否发送了定义外值（非 1~5 会被忽略）。

2. **一直无法解锁**
   - 是否持续 10Hz 心跳；
   - 是否满足“1 秒 + 10 包”；
   - `data=1` 时导航是否有效；
   - 或临时使用 `data=3` 验证链路（谨慎）。

3. **setpoint 发了不动**
   - 是否已经 `is_armed=true`；
   - `type_mask` 是否把轴屏蔽掉了；
   - `control_key` 是否选择了期望控制层。

---

## 9. 安全建议

- 首次联调优先小幅值命令；
- 始终保留 `status` 监控窗口；
- 紧急情况下停止发送 `agxhbt`，系统会超时失锁并回到安全输出。
