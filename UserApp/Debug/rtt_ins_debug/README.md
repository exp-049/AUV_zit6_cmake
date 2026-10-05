# INS_DEBUG

`INS_DEBUG` 复用 NORMAL 使用的 `INS_Driver` 和 `INS_Porting`，只替换应用启动任务，
通过 RTT 输出 USART1 接收的 NAV-300 帧。

## 硬件与协议

- UART：`AUV_UART_INS`（当前为 `huart1` / USART1）
- 引脚：PB14/PB15（USART1，按 CubeMX 信号定义）
- 接收：512 字节 Circular DMA，任务轮询 DMA 写指针
- 配置：115200、8 数据位、1 停止位、无校验
- DMA 在驱动初始化时启动一次；运行期不会因 UART 错误或无新字节自动重启。
- 当前调试链路使用 PDF 中“室外 GPS/SINS/DVL”格式：`FA AF` + 133 字节定长帧，
  帧头为 `FA AF`、帧尾为 `FB BF`。协议中的 XOR 字节位于偏移 130，但当前固件不将
  XOR 结果用于帧通过/拒收判断；满足长度、帧头和帧尾条件的帧会交给解码器。
- PDF 同时定义了 93 字节“室内 SINS/DVL”格式；当前公共 `INS_Driver` 只接收上面的
  133 字节格式，因此设备若输出 93 字节帧，RTT 的 `valid` 不会增加。
- PDF 中经纬度的 int32 缩放为 `1e6`；RTT 字段表按该缩放显示。

## 构建与烧录

```bash
cmake --build --preset INS_DEBUG
~/.local/bin/pyocd load --target stm32h743xx --uid 6894719E6D7C \
  --frequency 2m --connect attach --erase sector \
  -O load.pre_reset=off -O load.post_reset=sysresetreq \
  build/INS_DEBUG/UserApp/AUV_zit6.elf
```

也可以使用 VS Code 任务 `Build INS Debug` 和 `Flash INS Debug (DAPLink)`。

## RTT 查看

```bash
~/.local/bin/pyocd rtt --target stm32h743xx --uid 6894719E6D7C \
  --frequency 2m --connect attach --address <SEGGER_RTT控制块地址>
```

正常接收时每个帧结构有效的帧会输出完整的“偏置,源码,含义,解析”表；每秒输出一次 DMA、
接收字节数、有效帧数、帧头/帧尾失败数和 USART1 状态寄存器快照。

`INS_DEBUG` 不创建 micro-ROS、ControlTask 或 MonitorTask；任务自身刷新硬件看门狗，
因此不会改变 NORMAL 的启动链。
