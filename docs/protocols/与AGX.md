# ZIT6 AUV 通讯协议规范（按当前代码实现）

本文档描述的是当前仓库中已经实现并在 `UserApp/Component/MicroRos/*`、`zit6_interfaces/*` 中定义的真实通信接口。

---

## 0. 总览

当前固件通过 micro-ROS 与上位机通信，主要包含：

- 控制指令：`/zit6/cmd/*`
- 状态反馈：`/zit6/state/*`
- 参数服务：`/zit6/update_params`、`/zit6/get_params`
- 仿真输入：`/zit6/sim/nav`
- 日志输出：`/zit6/log`

本版本将核心运动控制接口统一升级为 6-DOF。由于 ROS 2 接口类型随消息字段变化，旧版 4-DOF 上位机必须同步重新构建，不能与新版节点混用同一个控制话题。

当前通信任务实现位于：
- `UserApp/Application/MicroRosTask.cpp`
- `UserApp/Component/MicroRos/MicroRosPublisher.cpp`
- `UserApp/Component/MicroRos/MicroRosSubscriber.cpp`
- `UserApp/Component/MicroRos/MicroRosService.cpp`

---

## 1. 坐标系约定

系统使用两套坐标系：

| 坐标系 | 约定 | X 轴 | Y 轴 | Z 轴 | 用途 |
|---|---|---|---|---|---|
| 机体系 | FRD | 前 | 右 | 下 | 速度、推力、执行器输出 |
| 世界系 | NED | 北 | 东 | 下 | 位置、路径规划、绝对目标 |

补充说明：
- 姿态角索引遵循 6-DOF 约定：`[X, Y, Z, Roll, Pitch, Yaw]`
- 上位机主控载荷统一为 6-DOF：`[X, Y, Z, Roll, Pitch, Yaw]`
- 外部 setpoint 实际控制轴为 `X/Y/Z/Yaw`；`Roll/Pitch` 为兼容/观测字段，即使下发也始终旁路
- micro-ROS 状态话题保留完整 6-DOF 顺序，供上位机姿态/视觉解算使用

---

## 2. 控制指令

### 2.1 `/zit6/cmd/setpoint`

- 类型：`zit6_interfaces/msg/ZitSetpoint`
- 源定义：`zit6_interfaces/msg/ZitSetpoint.msg`

```text
uint8 control_key
uint8 type_mask
float32 x
float32 y
float32 z
float32 roll   # 兼容字段；固件旁路，不进入控制器
float32 pitch  # 兼容字段；固件旁路，不进入控制器
float32 yaw
uint32 seq
```

#### `control_key` 定义

- 低 2 位：模式
  - `0`：POSITION
  - `1`：VELOCITY
  - `2`：ACTUATOR（直接推力控制）
- `Bit4 (0x10)`：目标是机体系（Body）
  - 未置位则为世界系（World / NED）
- `Bit5 (0x20)`：增量模式

#### `type_mask` 定义

`type_mask` 的语义是：**bit 置 1 表示跳过该轴，不更新该轴目标**。

| bit | 轴 | 含义 |
|---|---|---|
| 0 | X | 1 表示跳过 X |
| 1 | Y | 1 表示跳过 Y |
| 2 | Z | 1 表示跳过 Z |
| 3 | Roll | 兼容位；Roll 始终旁路 |
| 4 | Pitch | 兼容位；Pitch 始终旁路 |
| 5 | Yaw | 1 表示跳过 Yaw |

示例：
- `type_mask = 0`：更新实际控制的 X/Y/Z/Yaw；Roll/Pitch 字段仍被旁路
- `type_mask = 32`：跳过 Yaw，只更新 X/Y/Z
- `type_mask = 62`：只更新 X，保持 Y/Z/Yaw 不变；Roll/Pitch 始终旁路

这和旧文档里“mask=1|2 表示不控制 X 和 Y”的解释方向一致，但要特别强调：**这是 skip mask，不是 enable mask**。

#### 当前代码中的接收约束

`MicroRosSubscriber::onSetpoint()` 当前行为：

1. 对 X/Y/Z/Yaw 做 `isfinite()` 检查，NaN / Inf 直接丢弃；Roll/Pitch 为旁路字段，不参与控制
2. 若系统未解锁（`is_armed == false`），直接丢弃
3. 模式只接受 `0/1/2` 三种
4. POSITION / VELOCITY 模式还要求导航有效，除非处于仿真模式（HITL / SITL）

#### 各模式解释

- POSITION：
  - 目标写入 `MotionContext::current_setpoint_.pos_world`
  - body 指令会先旋转到世界系
  - Roll/Pitch 不写入控制目标，始终保持旁路
- VELOCITY：
  - 目标写入 `current_setpoint_.vel_body`
  - world 指令会先变换到机体系
- ACTUATOR：
  - 目标写入 `current_setpoint_.thrust_body`
  - world 指令会先变换到机体系

### 2.2 `/zit6/cmd/agxhbt`

- 类型：`std_msgs/msg/UInt32`
- 用途：ARM / DISARM 心跳

当前代码约定：
- `data = 1`：正常解锁模式，需要导航有效
- `data = 3`：遥控模式，允许绕过导航有效性检查

当前 `SafetyMonitor` 中的阈值：
- 最小心跳数：`10`
- 最小持续时间：`1000ms`
- 建议心跳频率：`2Hz`
- 已解锁后心跳超时：`1000ms`
- 未解锁状态下心跳计数清零阈值：`1000ms`

### 2.3 `/zit6/cmd/ins`

- 类型：`std_msgs/msg/UInt8`

当前代码中的命令映射：
- `1`：`setDvlPower(true)`
- `2`：`setDvlPower(false)`
- `3`：`restart()`
- `4`：`resetPosition()`
- `5`：`setInitialPosition(sys_config.ins.init_lat, sys_config.ins.init_lon)`

### 2.4 `/zit6/cmd/servo`

- 类型：`std_msgs/msg/Float32`
- 含义：舵机角度，单位 rad

### 2.5 `/zit6/cmd/light`

- 类型：`std_msgs/msg/UInt8`
- 含义：灯光状态字节，具体颜色含义由下位机解释

### 2.6 `/zit6/cmd/pushrod`

- 类型：`zit6_interfaces/msg/ZitPushrod`
- 用途：向深度计解算板下发一个推杆任务

消息定义：

```text
float32 speed       # -1.0～1.0
uint32 duration_ms  # 必须大于 0
```

推杆命令只在系统已解锁时接受。固件将 `speed` 转换为协议层的
`power_x1000`（`speed * 1000`），并由固件自动维护 `task_id`、ACK 和超时重发；
上位机不需要自行生成任务 ID。建议使用一次性发布，不要周期性高频发布同一任务。

---

## 3. 状态发布

### 3.1 `/zit6/state/pos`

- 类型：`std_msgs/msg/Float32MultiArray`
- 当前发布频率：约 `30Hz`
- 数据格式：`[x, y, z, roll, pitch, yaw]`
- 含义：世界系 NED 位姿，角度单位 rad

### 3.2 `/zit6/state/vel`

- 类型：`std_msgs/msg/Float32MultiArray`
- 当前发布频率：约 `50Hz`
- 数据格式：`[u, v, w, p, q, r]`
- 含义：机体系 FRD 线速度与角速度，角速度单位 rad/s

注意：旧文档写成 60Hz 或 62.5Hz 都不准确，当前实现是 `>=20ms` 发布一次，即约 50Hz。

### 3.3 `/zit6/state/thr`

- 类型：`std_msgs/msg/Float32MultiArray`
- 当前发布频率：约 `30Hz`
- 数据格式：`[fx, fy, fz, mroll, mpitch, myaw]`
- 含义：机体系 6-DOF 力/力矩输出快照

### 3.4 `/zit6/state/status`

- 类型：`zit6_interfaces/msg/ZitStatus`
- 当前发布频率：`10Hz`
- 源定义：`zit6_interfaces/msg/ZitStatus.msg`

当前真实字段：

```text
bool is_armed
uint8 arm_mode
uint8 control_level
uint8 ins_state
bool navigation_ready
float32[6] forces
float32 cycle_time_ms
float32 battery_voltage
uint32 error_flags
```

说明：
- `forces` 当前是 6 元素：`[Fx, Fy, Fz, Mroll, Mpitch, Myaw]`
- `battery_voltage` 当前代码固定填 `0.0f`
- `error_flags` 当前代码固定填 `0`
- `arm_mode` 实际写入的是 `last_heartbeat_data`

### 3.5 `/zit6/state/zithbt`

- 类型：`std_msgs/msg/UInt32`
- 当前发布频率：`1Hz`
- 内容：当前 `now_ms`

注意：旧文档写成 10Hz 不符合当前实现。

### 3.6 `/zit6/state/USBL`

- 类型：`zit6_interfaces/msg/ZitUsbl`
- 发布方式：USBL 有效帧到达后事件驱动发布
- 缓存：FreeRTOS `xQueue`，长度 8
- 生产者：`ControlTask`
- 消费者：`MicroRosPublisher`

队列满时非阻塞丢弃最旧帧，保留最新数据，避免 USBL 发布阻塞 100Hz 控制任务。

### 3.7 `/zit6/log`

- 类型：`rcl_interfaces/msg/Log`
- 发布方式：事件驱动
- 来源：`RosLogger` 队列

---

## 4. 参数服务

### 4.1 `/zit6/update_params`

- 类型：`zit6_interfaces/srv/UpdateParams`

请求定义：

```text
string json
string[] paths
string[] values
```

响应定义：

```text
bool success
string message
```

当前服务端逻辑：
- 若 `json` 非空，优先解析 JSON
- 然后再处理 `paths[] + values[]` 形式的逐项更新
- 更新成功后会调用：
  - `ctx_->chassis->applyConfig(sys_config.chassis)`
  - `ctx_->watchdog->init(sys_config.system.soft_watchdog)`

### 4.2 `/zit6/get_params`

- 类型：`zit6_interfaces/srv/GetParams`

请求定义：

```text
string[] paths
```

响应定义：

```text
bool success
string message
string config_json
```

当前行为：
- `paths` 为空时返回完整配置
- 非空时返回匹配前缀的最小 JSON

---

## 5. 仿真输入

### `/zit6/sim/nav`

- 类型：`std_msgs/msg/Float32MultiArray`
- 数据长度要求：至少 12
- 含义：前 6 项为 `pos_world`，后 6 项为 `vel_body`

格式：
- `[x, y, z, roll, pitch, yaw, vx, vy, vz, p, q, r]`

该数据由 `MicroRosSubscriber::onSimNav()` 写入 `sitl_nav_queue`，供 `ControlTask` 消费。

---

## 6. ARM / DISARM 流程（按当前实现）

1. 上电后，`ControlTask` 初始化 INS、深度计、日志和底盘参数
2. `MicroRosTask` 等待 Agent 连接成功
3. 上位机持续发送 `/zit6/cmd/agxhbt`
4. `MonitorTask` 中的 `SafetyMonitor` 检查：
   - 心跳数是否达到 10
   - 心跳持续是否达到 1000ms
   - 是否满足导航有效或遥控模式
5. 满足条件后：
   - 记录 `home_offset`
   - 清空当前 setpoint
   - 设置 `is_armed = true`
6. 解锁后，若 1000ms 内未收到心跳，则自动上锁并切回 `NONE`

注意：
- 当前解锁时不会调用旧文档里描述的“渐进开环推力斜坡”逻辑
- 当前上锁动作会清除 `home_offset`，并通过 `setControlLevel(NONE)` 进入安全态

---

## 7. 示例

### 7.1 正常解锁

```bash
ros2 topic pub -r 2 /zit6/cmd/agxhbt std_msgs/msg/UInt32 '{data: 1}'
```

### 7.2 遥控模式解锁

```bash
ros2 topic pub -r 2 /zit6/cmd/agxhbt std_msgs/msg/UInt32 '{data: 3}'
```

### 7.3 世界系位置控制：更新 X/Y/Z，保持当前 Yaw

```bash
ros2 topic pub /zit6/cmd/setpoint zit6_interfaces/msg/ZitSetpoint '{control_key: 0, type_mask: 32, x: 1.0, y: 0.0, z: -1.0, roll: 0.0, pitch: 0.0, yaw: 0.0, seq: 1}'
```

这里 `type_mask = 32` 表示跳过 Yaw，只更新 X/Y/Z；Roll/Pitch 字段即使填写也会被固件旁路。

### 7.4 机体系速度控制：只控制 X 轴前进速度

```bash
ros2 topic pub /zit6/cmd/setpoint zit6_interfaces/msg/ZitSetpoint '{control_key: 17, type_mask: 62, x: 0.2, y: 0.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0, seq: 2}'
```

解释：
- `17 = 0x10 | 0x01`：Body + Velocity
- `62 = 0b111110`：仅更新 X，跳过 Y/Z/Roll/Pitch/Yaw

### 7.5 查询全部参数

```bash
ros2 service call /zit6/get_params zit6_interfaces/srv/GetParams '{paths: []}'
```

### 7.6 JSON 方式更新参数

```bash
ros2 service call /zit6/update_params zit6_interfaces/srv/UpdateParams '{json: "{\"chassis\":{\"x\":{\"vel_kp\":1.2}}}", paths: [], values: []}'
```

### 7.7 路径方式更新参数

```bash
ros2 service call /zit6/update_params zit6_interfaces/srv/UpdateParams '{json: "", paths: ["chassis.x.vel_kp"], values: ["1.2"]}'
```

---

## 8. 与旧文档相比的已修正项

本次已按当前代码修正以下内容：

- `type_mask` 语义明确为 skip mask
- `vel` 频率修正为约 50Hz
- `zithbt` 频率修正为 1Hz
- `status` 的真实字段与 `ZitStatus.msg` 对齐
- `update_params` 增加 `paths[] / values[]` 说明
- ARM 超时改为当前代码中的 `500ms / 1000ms / 10次 / 1000ms`
- 明确 `/zit6/sim/nav` 已经是正式接入口
