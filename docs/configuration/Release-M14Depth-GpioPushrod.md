# M14 + GPIO 推杆 Release 固件

构建预设 `Release-M14Depth-GpioPushrod` 使用 `Release` 优化、`NORMAL`
应用模式和硬件预设 `7`。M14 使用 UART4（PA11 RX、PA12 TX，115200/8N1），
GPIO 推杆使用 PB8=IN1、PB7=IN2。

```bash
cmake --preset Release-M14Depth-GpioPushrod
cmake --build --preset Release-M14Depth-GpioPushrod
```

ELF 输出位于 `build/Release-M14Depth-GpioPushrod/UserApp/AUV_zit6.elf`。
`config.json` 中 `system.depth_source=use_fused_z` 使用 INS 压力深度、M14 深度和 INS 垂向速度融合导航 Z；
也可设置 `use_m14_z`、`use_ins_pressure_z` 或 `use_ins_integrated_z` 使用单一来源。旧键 `system.z_data_sourse` 与旧值 `use_ms5837_z` 仍兼容。

融合深度以 INS 压力计为参考，只估计一个标量偏置 `b = z_INS_pressure - z_M14`，并将 M14 校正为 `z_M14 + b`。
`b` 的低通时间常数为 30 秒，变化率限制为 0.01 m/s；如果上电时只有一路深度，先用该路连续输出，另一来源恢复后再慢速校准偏置。
垂向速度由 INS 机体系速度和姿态转换到 NED 向下轴，用于两路绝对深度暂不可用时的预测。

INS 协议没有压力样本更新标志或计数器，压力新鲜度只能估计：压力值变化至少 0.01 m 才刷新变化时间；超过 1 秒未变化时权重降至 0.25；若这段时间内垂向速度累计预测位移达到 0.1 m，则将压力标记为疑似陈旧，直到压力值再次变化。静止时的重复压力值仍可作为低权重绝对观测。
`setorigin` 仍要求 INS 全局导航样本新鲜，并要求 M14 或 INS 压力至少一路有效；`nav_valid` 继续表示整体导航有效，消息结构不增加单独的 z 有效位。

实机模式持续监测惯导和深度计的**有效帧**。INS 有效帧指 133 字节长度、`FA AF` 帧头和
`FB BF` 帧尾均匹配；当前 XOR 结果不参与帧有效性判断。超时阈值沿用
`system.soft_watchdog.timeout_ms`，默认 3000 ms。初始化及退出仿真后均有
一个超时周期的等待时间。收到串口字节但未解析出有效帧也会触发报警。

INS USART1 错误回调会记录错误状态和诊断标记，但运行期不会因 UART 错误或无新字节自动中止、重启
INS RX DMA；DMA 只在驱动初始化时启动。帧边界错位时，解析器通过搜索完整 `FA AF` 帧头
重新同步。若底层 DMA 已停止，当前配置不会自动拉起接收。

M14 UART4 保留任务上下文恢复：UART 错误或有效帧超时会触发接收恢复。ControlTask
中止 UART 接收，再 DeInit/Init 对应 DMA 通道以清理 HAL/DMAMUX 状态，然后重启 DMA，
并核验 DMA、DMAR 和 HAL 接收状态。恢复失败按 100/250/500/1000/2000 ms 退避，之后
每 5 秒再探测一次。恢复成功会丢弃当前未完成的 M14 文本行，避免把恢复前后的字节拼成一帧。

失联时通过 `/zit6/log` 和 RTT 通道 0 立即输出 `ERROR`，并输出
惯导接收/解析计数、DMA 位置/状态、UART/DMA HAL 阶段状态及深度计的原始字节等诊断信息。
持续失联时每 10 秒重复一次；恢复有效帧后输出一次 `INFO` 恢复日志。
其中 `INS data reception recovered` 表示再次解析到帧结构有效的数据，不表示 DMA 被重启。
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
