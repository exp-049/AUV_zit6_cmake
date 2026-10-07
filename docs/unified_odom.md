# ZIT6 odom 与显式原点设置

MCU（SIL 中的 ZIT6 native backend）是 nav → odom 的唯一维护者。
ARM/DISARM 不修改原点；原点只在本次上电内保留，不写 Flash。
localization 原样适配版本化 odom，BasicMotion 的世界系目标直接使用该 odom。
位置、速度使用 NED/FRD 约定；底层角度为弧度，兼容 PoseInfo 角度为度。

## 接口

- 空请求服务 `/zit6/cmd/setorigin`，由 hw_manager 代理为
  `/auv/hardware/zit6/cmd/setorigin`，两者均为 `zit6_interfaces/srv/SetOrigin`。
- 回执 `origin_nav=[x,y,z,0,0,yaw]` 是实际采用的原始导航坐标，仅供回执和日志。
  每次成功重设递增 `origin_generation`，零表示尚未初始化。
- `zit6_interfaces/msg/ZitOdom` 从 `/zit6/state/odom` 转发至
  `/auv/hardware/zit6/state/odom`。位姿、机体速度、采样时间、导航有效性、
  初始化标志和版本来自同一次快照。旧 pos/vel topic 保留；没有新增 raw nav topic。
- `/auv/state/reset` 使用 `uv_msgs/msg/StateResetRequest` 的 `request_id`；
  `/auv/state/reset_result` 使用 `StateResetResult` 关联完成结果。
  localization 只有在服务成功且收到对应版本的新鲜 odom 后才返回成功。
- PoseInfo 的 `origin_*` 字段弃用并固定为零，元数据直接复制 MCU 快照。

## START

BasicMotion 是 ARM 心跳唯一拥有者，默认 `heartbeat_rate=15.0`、`arm_mode=1`。
它向 `/auv/hardware/zit6/cmd/agxhbt` 发布，hw_manager 逐条转发。
所有 setpoint 同样经 hw_manager 转发，BasicMotion 不重复向原始 MCU topic 发布。

START 禁止运动并结束旧动作/速度租约，发送零速度，暂停心跳，等待新鲜上锁状态；
然后发起关联 reset，确认匹配的新版本 odom，恢复心跳并确认新鲜解锁状态。
重复 START 执行同样流程。服务失败、超时或取消后保持心跳停止，不自动重试。
`agxhbt=0` 仅更新心跳，不主动上锁，必须停止发布来触发现有 1 秒超时。
START 默认总超时 10 秒（可由 action timeout 指定），代理默认超时 2 秒。
setorigin 成功时同时清除此前累计的 ARM 心跳资格；ARM 提交时重新核验最新资格，
新原点必须等待之后的心跳重新满足计数和持续时间要求。
普通动作取消和速度租约超时只发送零速度，保留原有 safe-stop 语义。

所有 ARM 模式都要求本次上电已设置过原点。setorigin 只允许上锁时从实际有效、
新鲜且有限的导航样本提交；固件样本最大年龄为 200 ms，仿真输入停止不会刷新其时间。

## 仿真与启动

真机先等待 backend/nav 就绪，再 START，最后确认解锁。
SIL/HIL 的 DVL/IMU 积分在仿真后端保持连续，START 不重设该 raw navigation。
HIL 输入不订阅 `/auv/state/odom` 或 `/auv/state/twist`，避免把 MCU 输出送回导航输入。
HIL 在上锁时将 MCU runtime 配置切换为外部 SITL 导航输入，关闭内部 HITL 物理积分。

感知与 RViz 订阅成功的 reset_result 清除旧坐标缓存，并隔离不同 odom 版本的数据。
MCU 重启/导航时间回退会撤销在途确认，迟到服务回执不能完成新的请求。

## 容量与验证

| 资源 | 使用（含可选 pushrod） | 配置 |
|---|---:|---:|
| service | 3 | 4 |
| executor handle | 10 | 18 |
| publisher | 9 | 10 |
| subscription | 7 | 8 |

接口更改后必须重新生成 micro-ROS 静态库并构建 `zit6_interfaces`、`uv_msgs`。
现有 `scripts/build_lib.sh` 会根据接口和资源配置哈希自动重生成。

验收时，在非零 nav XYZ/yaw 下进行两次 START，比较 typed MCU odom、
`/auv/state/odom` 和 BasicMotion 兼容 PoseInfo 的坐标及版本；再验证服务失败、
START 取消、旧版本帧/迟到回执、心跳丢失、MCU 重启和 Agent 重连。
发送 SET 的 `[x,y,z,yaw]` 应只转换角度单位，不添加原点平移或旋转。

```bash
ros2 service call /auv/hardware/zit6/cmd/setorigin zit6_interfaces/srv/SetOrigin '{}'
ros2 action send_goal /auv/basic_motion uv_msgs/action/BasicMotion \
  '{cmd_type: 6, timeout: 10.0}'
```

直接调用 setorigin 不执行自动上锁/解锁；常规启动应调用 BasicMotion START。
