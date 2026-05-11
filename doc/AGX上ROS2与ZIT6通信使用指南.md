# AGX 上使用 ROS2 与 ZIT6 MCU 通信指南

本文面向在 NVIDIA AGX（Ubuntu + ROS2）上联调 ZIT6 MCU 的场景，说明从环境准备到解锁控制的最小可用流程。

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
>
> 如无权限，可将当前用户加入 `dialout` 组后重新登录。

---

## 2. 工作空间准备

假设工作空间为 `<ws>`：

```bash
cd <ws>
# 按你的 ROS2 环境先 source，再构建
source /opt/ros/<distro>/setup.bash
colcon build --packages-select zit6_interfaces upper_examples --symlink-install
source install/setup.bash
```

可选：在 `~/.bashrc` 增加别名（便于联调）

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

如果启动成功，AGX 侧即可与 MCU 上的 micro-ROS client 建链。

---

## 4. 核心话题与服务

### 4.1 AGX -> MCU（控制指令）

- `/zit6/cmd/setpoint`（`zit6_interfaces/msg/ZitSetpoint`）：位置/速度/推力控制目标。
- `/zit6/cmd/agxhbt`（`std_msgs/msg/UInt32`）：上位机心跳与解锁触发。
- `/zit6/cmd/ins`（`std_msgs/msg/UInt8`）：INS 控制指令。
- `/zit6/cmd/light`（`std_msgs/msg/UInt8`）：灯控。
- `/zit6/cmd/servo`（`std_msgs/msg/Float32`）：舵机角度。

### 4.2 MCU -> AGX（状态反馈）

- `/zit6/state/status`：综合状态（含 `is_armed`、导航状态等）。
- `/zit6/state/pos`：位置反馈。
- `/zit6/state/vel`：速度反馈。
- `/zit6/state/thr`：推力反馈。
- `/zit6/state/zithbt`：节点心跳。

### 4.3 参数服务

- `/zit6/update_params`：更新参数（JSON 字符串）。
- `/zit6/get_params`：按路径查询参数。

---

## 5. 最小联调流程（推荐顺序）

1. **启动 Agent**
   - 在 AGX 终端运行 `MicroXRCEAgent ...`。
2. **观察状态**
   - `ros2 topic echo /zit6/state/status`
3. **发送 AGX 心跳（解锁前提）**
   - 正常解锁：
     ```bash
     ros2 topic pub -r 10 /zit6/cmd/agxhbt std_msgs/msg/UInt32 "{data: 1}"
     ```
   - 强制解锁（按系统策略谨慎使用）：
     ```bash
     ros2 topic pub -r 10 /zit6/cmd/agxhbt std_msgs/msg/UInt32 "{data: 3}"
     ```
4. **下发 setpoint**
   - 位置控制示例：
     ```bash
     ros2 topic pub /zit6/cmd/setpoint zit6_interfaces/msg/ZitSetpoint "{control_key: 0, type_mask: 7, x: 1.0, y: 0.0, z: -1.0, yaw: 0.0}"
     ```
5. **监控反馈**
   - `ros2 topic echo /zit6/state/pos`
   - `ros2 topic echo /zit6/state/thr`

---

## 6. 常用命令速查

```bash
# 查看当前可见节点与话题
ros2 node list
ros2 topic list

# 查看某个话题类型
ros2 topic type /zit6/cmd/setpoint

# 更新 PID 参数示例
ros2 service call /zit6/update_params zit6_interfaces/srv/UpdateParams "{json: '{\"chassis\":{\"pid\":{\"pos\":{\"kp\":0.015,\"ki\":0.001}}}}'}"

# 查询全部参数
ros2 service call /zit6/get_params zit6_interfaces/srv/GetParams "{paths: []}"
```

---

## 7. 常见问题排查

1. **看不到 `/zit6/*` 话题**
   - 先确认 Agent 是否正常运行。
   - 检查串口设备名与波特率（示例为 `921600`）。
   - 检查串口权限。

2. **能通信但无法解锁（`is_armed` 始终 false）**
   - 确认持续发送 `/zit6/cmd/agxhbt`（建议 10Hz）。
   - 确认导航状态满足系统解锁条件（如 `navigation_ready`）。

3. **控制指令无响应**
   - 确认 `control_key` 与 `type_mask` 组合正确。
   - 检查当前控制模式与安全状态（上锁/急停/超时保护）。

---

## 8. 安全建议

- 首次联调优先使用小幅度目标值，避免直接大推力输出。
- 始终保持心跳与状态监控窗口打开。
- 出现异常时立即停止心跳下发，使系统回到安全状态。
