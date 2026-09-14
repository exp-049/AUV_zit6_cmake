# MS5837 RTT 调试

该模式使用当前 UART4 协议后端，不创建 NORMAL 模式的 ControlTask、
micro-ROS 和 MonitorTask。调试任务自身轮询深度后端并刷新 IWDG。

## 构建

```bash
cmake --build --preset MS5837_CAL_DEBUG
```

## RTT 输出

每秒输出一行：

```text
tick=... polls=... samples=... age_ms=... z_protocol=... temp=... connected=... handshake_ack=...
```

- `polls`：本秒调用 `Depth_Sensor_Driver::Read()` 的次数。
- `samples`：收到并通过协议校验、状态有效的数据帧数量。
- `age_ms`：最近一次有效数据距现在的时间；没有数据时为 `4294967295`。
- `z_protocol`：协议 `depth_cm / 100.0` 后写入的 z 值。
- `handshake_ack`：最近 5 秒内是否收到设备 ACK；新协议要求主机约每 2 秒发送握手，
  设备靠有效握手刷新其看门狗。
- `rx_recoveries`：UART4 DMA+IDLE 接收层自动恢复次数；正常情况下应保持为 0，
  若增加说明发生过 UART/DMA 停流并已自动重启。
- `rx_errors`：HAL 报告的 UART4 错误次数。
- `last_error`：最近一次 HAL UART 错误码。
- `recovery_reason`：最近一次恢复原因；当前 `1` 表示 DMAR/DMA stream
  未运行。环形 DMA 不再因为“超过 1 秒没有数据”而恢复，因此新固件不应再产生
  `2`；旧固件日志中的 `2` 代表此前的误超时恢复。
- `rx_events`：HAL UART IDLE/TC 事件次数。
- `dma_pos`：DMA 环形缓冲区当前写位置（0～255）。

如果 `polls` 增加但 `samples` 始终为 0，说明 UART4 没有形成有效协议帧，
需要检查接线、波特率、DMA 接收以及发送端是否使用 `A5 5A ...` 协议。
