# 添加 RTT Debug 测试模式的流程

本目录中的 Debug 模式采用“配置时选择一个应用模式、运行时只启动一个调试任务”的方式。
这样 Debug 测试可以复用 NORMAL 的驱动和移植层，同时不会与 NORMAL 的 micro-ROS、
ControlTask、MonitorTask 并行争抢同一硬件外设。

## 1. 先确定调试边界

新增测试前先明确：

- 测试操作哪个公共驱动接口，例如 `AppContext::motor_driver`、`ins_driver`。
- RTT 输入/输出格式，以及每条指令是否必须返回 `OK/ERR`。
- 是否需要独占 UART、DMA、定时器或看门狗。
- Debug 模式是否应停止 NORMAL 任务，避免同一个外设被多个任务同时控制。

优先调用 `UserApp/Peripherals` 和 `UserApp/Porting` 中已有的公共实现，不要在 Debug
目录复制一套协议解析、DMA 初始化或 UART 发送逻辑。

## 2. 创建 RTT 调试任务

为每个模式创建独立目录，例如：

```text
UserApp/Debug/rtt_xxx_debug/
├── XxxDebugTask.cpp
└── README.md
```

任务通常应包含以下结构：

1. `SEGGER_RTT_Init()`。
2. 配置 RTT Up Buffer 输出诊断信息，必要时使用
   `SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL`，避免长日志被丢弃。
3. 配置 Down Buffer 接收命令；文本协议建议按 `\r`/`\n` 组包。
4. 通过 `AppContext.hpp` 获取公共驱动指针。
5. 对输入做范围、格式和状态检查。
6. 每条完整指令返回 `RX: <command> -> OK/ERR`，同时报告失败原因。
7. 循环中刷新 `hiwdg1` 并 `osDelay()`。

参考实现：[rtt_motion_debug/MotionDebugTask.cpp](rtt_motion_debug/MotionDebugTask.cpp)。

## 3. 注册应用模式

需要同时修改四处，数值必须一致：

### `UserApp/CMakeLists.txt`

在 `AUV_APP_MODE` 的可选列表中加入模式，并分配编译宏，例如：

```cmake
set_property(CACHE AUV_APP_MODE PROPERTY STRINGS NORMAL XXX_DEBUG)
set(_auv_app_modes NORMAL XXX_DEBUG)
...
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE AUV_APP_MODE=5)
```

### `UserApp/Debug/CMakeLists.txt`

只把对应任务加入该模式：

```cmake
elseif(AUV_APP_MODE STREQUAL "XXX_DEBUG")
    target_sources(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/rtt_xxx_debug/XxxDebugTask.cpp
    )
endif()
```

### `UserApp/Debug/DebugApp.hpp`

增加任务的 `extern "C"` 声明：

```cpp
void UserApp_XxxDebugTask(void *argument);
```

### `UserApp/Application/AppMain.cpp`

为任务设置独立名称、栈和优先级，并在对应的 `AUV_APP_MODE` 分支中创建它；创建失败必须调用
`Error_Handler()`，随后退出启动任务。

## 4. 加入 CMake Preset

在根目录 `CMakePresets.json` 中同时加入 configure/build preset：

```json
{
    "name": "XXX_DEBUG",
    "inherits": "default",
    "cacheVariables": {
        "CMAKE_BUILD_TYPE": "Debug",
        "AUV_APP_MODE": "XXX_DEBUG"
    }
}
```

build preset 的 `configurePreset` 也使用 `XXX_DEBUG`。之后可执行：

```bash
cmake --preset XXX_DEBUG
cmake --build --preset XXX_DEBUG
```

## 5. 加入 VS Code Task

在 `.vscode/tasks.json` 增加至少两个任务：

- `Build Xxx Debug`：执行 `cmake --build --preset XXX_DEBUG`。
- `Flash Xxx Debug (DAPLink)`：烧录 `build/XXX_DEBUG/UserApp/AUV_zit6.elf`，并依赖前面的构建任务。

烧录任务应沿用当前板卡的 target、UID、频率和 reset 参数，不要只修改 ELF 路径。
如果需要 Cortex-Debug 单步调试，再在 `.vscode/launch.json` 增加对应 ELF 和
`preLaunchTask`。

## 6. 验证清单

每个新 Debug 模式至少验证：

```bash
git diff --check
cmake --preset XXX_DEBUG
cmake --build --preset XXX_DEBUG
```

上板后检查：

- RTT 首先出现任务启动标志和帮助信息。
- 正常指令收到 `OK`，格式/范围错误收到 `ERR`。
- 发送动作不会同时启动 NORMAL 控制任务。
- 连续发送测试指令时看门狗不复位，UART/DMA 没有被重复初始化。
- 退出该模式前使用安全停止指令，将电机推力归零。

## 7. 当前模式索引

| 模式 | 用途 | VS Code 构建任务 |
|---|---|---|
| `MS5837_CAL_DEBUG` | 深度传感器 RTT 诊断 | `Build MS5837 CAL Debug` |
| `USBL_DEBUG` | USBL DMA/协议 RTT 诊断 | `Build USBL Debug` |
| `INS_DEBUG` | INS 帧接收 RTT 诊断 | `Build INS Debug` |
| `MOTION_DEBUG` | RTT 下发电机、舵机、灯光指令 | `Build Motion Debug` |
