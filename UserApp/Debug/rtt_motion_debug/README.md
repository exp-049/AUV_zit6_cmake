# RTT motion debug

配置 `MOTION_DEBUG` 后，固件只启动这个 RTT 控制任务，不启动 NORMAL 的
micro-ROS、ControlTask 和 MonitorTask。RTT Down Buffer 0 接收命令，命令以
回车或换行结束；每条非空命令都会从 RTT Up Buffer 0 返回 `RX: ... -> OK/ERR`。

舵机命令需要显式指定通道：`S1 <角度>` 控制通道 1，`S2 <角度>` 控制通道 2。
`0x04` 已分配给第二路舵机，因此 RTT 工具不再发送旧握手帧。

## RTT 主机工具

工程根目录提供 `scripts/motion_rtt.sh`：

```bash
# 自动烧录后启动 RTT TCP 服务，保持该终端运行
./scripts/motion_rtt.sh rttd

# 另一个终端连接文本 RTT 通道，输入命令后按 Enter
./scripts/motion_rtt.sh nc

# 不经过 TCP，直接用 pyOCD 查看 RTT
./scripts/motion_rtt.sh view

# 自动启动 rttd 并连接 nc
./scripts/motion_rtt.sh all
```

默认 TCP 地址为 `127.0.0.1:8023`，可用 `RTT_PORT=19023` 覆盖。`rttd` 依赖
`socat`，`nc` 依赖 `nc`/`netcat`；VS Code 中对应任务为 `Start Motion RTTD`、
`Connect Motion RTT (nc)` 和 `View Motion RTT (pyOCD)`。

```text
X0.2       # Fx：向前 20%
X-0.2      # Fx：向后 20%
R0.2       # Fy：向右 20%
D-0.2      # Fz：向上/反向 20%，具体正方向由动力板坐标定义
Y0.2       # Fyaw：yaw 正向 20%
S1 -90     # 舵机 1 角度 -90 度
S2 45      # 舵机 2 角度 45 度
L2         # 灯状态 2
STOP       # X/R/D/Y 全部清零并发送
```

`X/R/D/Y` 的数值范围是 `-1..1`，每次修改后会携带当前四个轴值发送完整推力包；
未使用的 pitch、roll 在此调试模式下发送为 0。`S` 按 VIT6 动力板当前接口使用度数，
范围是 `-180..180`。
