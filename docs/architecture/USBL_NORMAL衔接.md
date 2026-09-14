# USBL 在 NORMAL 构建中的衔接

## 目标

NORMAL 只保证 USBL 稳定接收和解包。当前 USBL 数据保存在驱动快照中，不参与 INS、MS5837 或控制输出的数据源选择；后续融合/选择策略通过 `AppContext::usbl_driver` 读取即可接入。

## 层次与调用链

```text
USART3 PB11 RX
    ↓
USBL_Porting（512B Circular DMA + IDLE）
    ↓ 轮询 DMA 写位置，增量取出字节
USBL_Driver（EA AE ... EB BE，133B，XOR）
    ↓
ControlTask（100Hz update）
    ↓
AppContext::usbl_driver / UsblState
```

### 协议组件

`UserApp/Peripherals/USBL/src/USBL_Driver.cpp` 不直接引用 UART/DMA。它通过 `UsblPortOps` 获得字节流，完成：

- `EA AE` 帧头同步；
- 固定 133 字节长度收帧；
- 校验偏移 0..129 的异或值是否等于偏移 130；
- 检查 `EB BE` 帧尾；
- 按小端格式填充 `UsblState`；
- 维护有效帧/无效帧计数和最近一帧原始帧。
- 时间戳通过 `UsblPortOps::getTickMs` 注入，不直接依赖 STM32 HAL。

### 硬件移植层

`UserApp/Porting/USBL/src/USBL_Porting.cpp` 绑定 `AUV_UART_USBL`（当前为 `huart3`）：

- RX DMA：`DMA1_Stream3`，Circular，512 字节；
- 串口：115200、8N1；
- 接收启动：`HAL_UARTEx_ReceiveToIdle_DMA()`；
- 关闭 DMA 半传输中断，仅保留 IDLE/传输完成路径；
- DMA 缓冲放在 RAM_D2 的 `.dma_buffer` 段。

移植层回调只做事件计数，协议解析在任务上下文中完成，避免在中断中执行浮点解码和大段内存操作。

## NORMAL 启动顺序

1. `AppContext.cpp` 构造 `USBL_Porting`、`USBL_Driver`，并把两者通过 `UsblPortOps` 连接。
2. `ControlTask::init()` 调用 `ctx_->usbl_driver->init()`，启动 USART3 RX DMA。
3. 实物导航分支每个 10 ms 调用 `ctx_->usbl_driver->update(usbl_state_)`。
4. 成功帧更新 `usbl_state_`，同时可通过 `getUsblState()` 或 `AppContext::usbl_driver` 获取。
5. 当前 INS、MS5837 和电机输出路径保持原样；USBL 不会改变 `z_data_source` 或导航状态。

## Debug 构建边界

`AUV_APP_MODE=USBL_DEBUG` 只额外编译 `rtt_usbl_debug/UsblDebugTask.cpp`。该任务只引用 `AppContext::usbl_driver` 输出 RTT 诊断和完整字段表，不再包含 USBL 的 HAL/DMA 初始化或协议解析副本。这样 Debug 与 NORMAL 共用同一套协议和移植代码，避免“Debug 能通、NORMAL 不能通”的分叉实现。
