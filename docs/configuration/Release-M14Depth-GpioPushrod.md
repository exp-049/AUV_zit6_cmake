# M14 + GPIO 推杆 Release 固件

构建预设 `Release-M14Depth-GpioPushrod` 使用 `Release` 优化、`NORMAL`
应用模式和硬件预设 `7`。M14 使用 UART4（PA11 RX、PA12 TX，115200/8N1），
GPIO 推杆使用 PB8=IN1、PB7=IN2。

```bash
cmake --preset Release-M14Depth-GpioPushrod
cmake --build --preset Release-M14Depth-GpioPushrod
```

ELF 输出位于 `build/Release-M14Depth-GpioPushrod/UserApp/AUV_zit6.elf`。
`config.json` 中 `system.z_data_sourse=use_ms5837_z` 使用 M14 覆写导航 Z；
INS 选项保留 INS 的导航 Z。

实机模式持续监测惯导和深度计的**有效帧**。超时阈值沿用
`system.soft_watchdog.timeout_ms`，默认 3000 ms。初始化及退出仿真后均有
一个超时周期的等待时间。收到串口字节但未解析出有效帧也会触发报警。

INS 的 USART1 和 M14 的 UART4 错误回调都会记录错误并请求任务上下文恢复。
ControlTask 会先中止 UART 接收，再 DeInit/Init 对应 DMA 通道以清理 HAL/DMAMUX
状态，然后重启 DMA，并核验 DMA、DMAR 和 HAL 接收状态均已运行。这样即使 HAL 错误
处理提前清掉 DMAR、DMA 句柄仍停留在 BUSY/ERROR，也可以重新初始化。即使 DMA
寄存器仍显示运行，超过有效帧超时也会重拉接收链路；
重拉成功后最多每 5 秒探测一次。恢复失败按 100/250/500/1000/2000 ms 退避，之后
每 5 秒再探测一次。重拉成功会丢弃当前未完成的 M14 文本行，避免把恢复前后的字节
拼成一帧。

失联时通过 `/zit6/log` 和 RTT 通道 0 立即输出 `ERROR`，并输出
惯导接收/解析计数、DMA 位置/状态、UART/DMA HAL 阶段状态及深度计的原始字节等诊断信息。
持续失联时每 5 秒重复一次；恢复有效帧后输出一次 `INFO` 恢复日志。
Release 正常接收期间不输出每秒深度诊断，失联诊断在 Release 中仍保留。
HITL/SITL 仿真分支不监测物理传感器。

`/zit6/state/status.error_flags` 使用现有协议位：任一传感器超时置位
`ERROR_SENSOR_FAIL | ERROR_COMM_TIMEOUT`（值为 10），均恢复后清零。
按选定策略，新增监测仅报警并继续运行；原有软件/硬件看门狗配置独立生效。

典型日志：

```text
[ERROR] INS data timeout: no valid frame for 3000 ms (limit=3000 ms)
[WARN] INS main: bytes=0 valid=0 invalid=0 reads=0
[ERROR] Depth data timeout: no valid frame for 3000 ms (limit=3000 ms)
[WARN] Depth UART4: start=0 dma=1 last_err=0x0
[INFO] INS data reception recovered
[INFO] Depth data reception recovered
```
