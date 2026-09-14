# RTT motion debug

配置 `MOTION_DEBUG` 后，固件只启动这个 RTT 控制任务，不启动 NORMAL 的
micro-ROS、ControlTask 和 MonitorTask。RTT Down Buffer 0 接收命令，命令以
回车或换行结束；每条非空命令都会从 RTT Up Buffer 0 返回 `RX: ... -> OK/ERR`。

`H` 是大写 handshake 调试命令。ZIT 发送：

```text
FA AF 04 FB BF
```

VIT6 收到后回复：

```text
FA AF 04 01 FB BF
```

RTT 会分别打印 `TX HANDSHAKE` 和 `RX HANDSHAKE`；其中 `0x01` 表示 VIT6 已就绪。

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
H          # 主动发起一次 ZIT-VIT6 handshake
R0.2       # Fy：向右 20%
D-0.2      # Fz：向上/反向 20%，具体正方向由动力板坐标定义
Y0.2       # Fyaw：yaw 正向 20%
S-90       # 舵机角度 -90 度
L2         # 灯状态 2
STOP       # X/R/D/Y 全部清零并发送
```

`X/R/D/Y` 的数值范围是 `-1..1`，每次修改后会携带当前四个轴值发送完整推力包；
未使用的 pitch、roll 在此调试模式下发送为 0。`S` 按 VIT6 动力板当前接口使用度数，
范围是 `-180..180`。
