# USBL_DEBUG 调试说明与 2026-07-15 调试日志

## 1. 调试范围

本目录对应 `AUV_APP_MODE=USBL_DEBUG`，只创建 USBL RTT 调试线程，不进入 NORMAL 正式任务链。调试目标是确认 USART3 的 DMA 接收、帧同步、异或校验和字段解析。

调试线程不再直接操作 HAL UART/DMA，也不重复实现帧解析。公共链路由以下两层提供：

- `UserApp/Peripherals/inc/USBL_Driver.hpp` / `src/USBL_Driver.cpp`：USBL 协议组件，负责帧同步、校验和字段解析。
- `UserApp/Porting/inc/USBL_Porting.hpp` / `src/USBL_Porting.cpp`：USART3 Circular DMA + IDLE 硬件移植层，负责环形缓冲读指针和 DMA 诊断。

`UsblDebugTask.cpp` 只通过 `AppContext::usbl_driver` 调用 `init()`、`update()`、`copyLastFrame()` 和 `getDiagnostics()`，因此 Debug 输出与 NORMAL 使用的是同一套驱动链路。

当前硬件配置：

- USBL 使用 `USART3`。
- `PB10 = USART3_TX`，`PB11 = USART3_RX`。
- 串口参数：`115200, 8N1`。
- RX 使用 `DMA1_Stream3`，512 字节 Circular DMA。
- 使用 USART IDLE/全局中断和 DMA 传输完成事件；关闭 DMA 半传输中断。
- RTT 控制块地址：`0x240179f4`（以当前 ELF 为准）。

## 2. 构建与烧录

在工程根目录执行：

```bash
cmake --build --preset USBL_DEBUG
```

烧录当前调试固件：

```bash
~/.local/bin/pyocd load \
  --target stm32h743xx \
  --uid 6894719E6D7C \
  --frequency 2m \
  --connect attach \
  --erase sector \
  -O load.pre_reset=off \
  -O load.post_reset=sysresetreq \
  build/USBL_DEBUG/UserApp/AUV_zit6.elf
```

如需确认 RTT 地址：

```bash
arm-none-eabi-nm build/USBL_DEBUG/UserApp/AUV_zit6.elf | rg '_SEGGER_RTT$'
```

连接 RTT：

```bash
~/.local/bin/pyocd rtt \
  --target stm32h743xx \
  --uid 6894719E6D7C \
  --frequency 2m \
  --connect attach \
  --address 0x240179f4
```

启动后应先看到：

```text
=== USBL_DEBUG USART3 DMA-CIRCULAR+IDLE ===
```

## 3. 实际 USBL 帧协议

实际设备协议与旧文档中的帧头/帧尾不同，当前确认值为：

```text
总长度：133 字节
帧头：EA AE（偏移 0..1）
异或：XOR 偏移 0..129，结果位于偏移 130
帧尾：EB BE（偏移 131..132）
```

多字节整数和 `float` 按小端解析。主要字段：

| 偏移 | 字段 |
|---:|---|
| 2/6/10 | 基阵横滚、俯仰、航向 `float` |
| 14 | 基阵压力 `float` |
| 18/22/26/30 | 四路斜距 `float` |
| 38/42 | 信标纬度/经度 `int32 / 10000000` |
| 46/50/54 | 通道 12/13/14 时间差 `float` |
| 59/61/63 | 被动横滚/俯仰/航向 `int16 / 100` |
| 65/67/69/71 | 四路信号强度 `uint16` |
| 83..90 | 30～37 kHz 能量，共 8 个 `uint8` |
| 91/95 | 综合信号强度、模拟增益 `float` |
| 99/103/107 | 信标北向、东向、压力/深度 `float` |
| 115 | 传感器状态位 |
| 116..120 | 年、月、日、时、分 |
| 121 | 秒 `float` |
| 129 | 当前导航模式 |

状态位从低位开始：bit0 声通讯更新、bit1 声通讯有效、bit2 GPS 更新、bit3 GPS 有效、bit4 USBL 更新、bit5 USBL 有效。

## 4. RTT 输出说明

诊断行格式：

```text
USBL diag: events=... valid=... bad=... invalid=... write_pos=... ndtr=... dma=...
```

- `events`：收到的 IDLE/传输完成事件数。
- `valid`：校验通过的完整帧数。
- `bad`：找到帧头但校验或帧尾错误的帧数。
- `invalid`：HAL 回调长度异常的事件数。
- `write_pos`：DMA 环形缓冲当前写位置。
- `ndtr`：DMA 剩余传输计数。
- `dma=1`：DMA 正在运行。

当前调试代码将 RTT 通道 0 配置为 `SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL`，以避免完整解析表超过默认 1 KiB RTT 缓冲后被丢弃。必须保持 RTT 主机连接；如果主机不读取，调试线程可能阻塞，USBL_DEBUG 仅用于调试，不用于 NORMAL 固件。

## 5. 2026-07-15 调试日志

### 5.1 初始现象

最初解包器使用旧协议值 `EB BE` 帧头和 `ED DE` 帧尾。RTT 中可以看到 DMA 正常运行：

```text
USBL diag: events=1 valid=0 bad=0 write_pos=133 ndtr=379 dma=1 ... rx=EA AE 00 00
USBL invalid: bad=1 xor_calc=EF xor_rx=00 tail=00 BA head=EB BE
```

`events`、`write_pos` 和 `NDTR` 持续变化，说明 USART3 已收到数据；但 `head=EB BE` 是解析器在数据流中偶然匹配到的伪帧头，不能作为真实帧头判断。

### 5.2 环形 DMA 与解析状态修正

修正了两个调试逻辑问题：

1. DMA 回绕事件只消费一次，避免重复消费旧数据。
2. 校验失败后立即清空帧解析状态，重新搜索下一帧帧头。

修正后仍然出现 `valid=0`，进一步对照原始 DMA 数据确认真实帧头为 `EA AE`，真实帧尾为 `EB BE`。

### 5.3 协议值修正后的结果

将 USBL_DEBUG 解包器改为 `EA AE ... EB BE` 后，出现：

```text
USBL frame #8 (133 bytes)
偏置,源码,含义,解析
offset=000, source=EA AE, meaning=frame_header, parsed=raw
...
USBL diag: events=10 valid=8 bad=0 invalid=0 ... dma=1
```

这证明 USART3 通信、DMA 接收、帧同步、长度判断、异或校验和帧尾判断均已通过。该次日志中解析表从偏移 59 处被截断，原因是 RTT 默认非阻塞输出缓冲区满后丢弃字符，并非 USBL 帧缺字段。

### 5.4 当前处理状态

已将 USBL_DEBUG RTT 通道改为阻塞输出模式，并重新编译通过。下一次烧录后，应能从 `offset=000` 连续看到完整字段直到 `offset=132` 的 `frame_tail`。

## 6. 常见异常判断

- `events=0`、`ndtr=512`：优先检查 USART3 映射、PB11 RX 接线、设备是否发送以及串口参数。
- `events` 增加但 `valid=0`、`bad` 增加：检查实际帧头/帧尾、波特率和协议版本。
- `valid` 增加且 `bad=0`：通信和协议解包正常；字段数值再按设备工况核对。
- `Control block not found`：RTT 地址不匹配、固件未烧录或目标未运行。
- `Board ID ... not recognized`、CoreSight 探测警告：在已能连接目标并识别 Cortex-M7 的情况下通常不是 USBL 通信故障。
