# RTT pushrod debug

配置 `PUSHROD_DEBUG` 后，固件只启动推杆 RTT 调试任务，不启动 NORMAL 的
micro-ROS、ControlTask 和 MonitorTask。任务通过公共 `Pushrod_Driver` 发送定时
推杆动作，并在动作完成或 UART 设备返回时输出 ACK。

## 构建

```bash
cmake --preset PUSHROD_DEBUG
cmake --build --preset PUSHROD_DEBUG
```

默认预设使用 `AUV_HARDWARE_PRESET=1`：推杆复用自研板的 UART4 链路（PA11/PA12，
115200、8N1）。如果要测试 GPIO 推杆，应选择兼容 GPIO 的硬件预设（2、6 或 7），
并将构建配置中的 `AUV_HARDWARE_PRESET` 改为对应值；GPIO 推杆使用 PB8=IN1、
PB7=IN2，正向为 `10`，反向为 `01`，停止为 `00`，不支持 PWM。

## RTT 命令

命令通过 RTT Down Buffer 0 输入，以回车或换行结束：

```text
P500 1000       # 正向 500/1000，持续 1000 ms
P-500,1000      # 反向 500/1000，持续 1000 ms
STATUS          # 查看驱动是否支持
STOP            # 安全停止；UART 模式发送零功率短任务
HELP
```

功率范围为 `-1000..1000`，持续时间范围为 `1..60000 ms`。调试任务 ID 从 `0` 开始，
每次成功发送后递增；GPIO 后端在定时停止后返回 ACK，UART 后端接收到自研板 ACK 后返回
ACK。

推杆动作具有实际机械风险，首次测试应使用较小功率和较短时长，并确认机构处于安全状态。
