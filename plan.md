# RTT 深度计调试 — 实施计划

> 前置条件：`UserApp/Thirdparty/RTT` 已 clone（SEGGER RTT 库）

---

## 一、背景

RTT (Real-Time Transfer) 是 SEGGER 的高效双向通信技术，
比 UART printf 快数十倍，不阻塞 CPU，适合高频调试输出。

---

## 二、文件结构

```
UserApp/
├── Thirdparty/RTT/              ← RTT 库（子模块）
│   ├── RTT/SEGGER_RTT.c
│   └── Config/SEGGER_RTT_Conf.h
│
├── Debug/                        ← 调试程序（新建）
│   └── rtt_ms5837_cal_debug/
│       ├── CMakeLists.txt        ← 独立编译目标
│       ├── main.c                ← 入口：RTT 初始化 + 深度计循环
│       └── README.md             ← 使用说明
│
├── Peripherals/                  ← 深度计驱动（复用现有）
│   └── inc|src/I2C_DepthBackend.*
│   └── inc|src/MS5837_Driver.*
│   └── inc|src/UART_DepthBackend.*
│
└── CMakeLists.txt                ← 添加 Debug 子目录
```

---

## 三、两个宏定义

### 3.1 `#define RTT_DEBUG`

| 行为 | 说明 |
|------|------|
| 条件 | 在编译命令行或 `UserApp/CMakeLists.txt` 中全局定义 |
| 效果 | 不运行正常飞控程序，只初始化 RTT + 运行单一调试任务 |
| 用途 | 隔离硬件调试，避免飞控逻辑干扰 |
| 入口 | `Debug/rtt_ms5837_cal_debug/main.c` 中的 `main()`（替代原有的 `main.c`） |

### 3.2 `#define RTT_MS5837_CAL_DEBUG`

| 行为 | 说明 |
|------|------|
| 额外条件 | 在 `RTT_DEBUG` 基础上额外定义 |
| 效果 | 运行深度计标定程序 |
| 功能 | 1. 初始化 MS5837 (I2C/UART) |
|       | 2. 循环读取深度/温度 |
|       | 3. 通过 RTT 输出格式化数据 |
| 用途 | 标定深度计零偏、验证通信 |

---

## 四、实施步骤

### Step 1 — 集成 RTT 库

- 将 `Thirdparty/RTT/RTT/SEGGER_RTT.c` 加入编译
- 添加头文件搜索路径到 `Thirdparty/RTT/Config`
- 验证 RTT 初始化 + `SEGGER_RTT_WriteString` 能在 J-Link RTT Viewer 看到输出

### Step 2 — 创建 `UserApp/Debug/rtt_ms5837_cal_debug/main.c`

```c
// 伪代码
#include "SEGGER_RTT.h"

void main(void) {
    // 1. HAL 初始化（时钟、GPIO、I2C/UART）
    // 2. RTT 初始化
    // 3. 初始化深度传感器 (MS5837_Driver)
    // 4. 循环 100Hz:
    //      - depth_sensor.Read()
    //      - depth_sensor.Depth(&d)
    //      - SEGGER_RTT_printf("D=%.3f T=%.2f\n", d, temp)
    // 5. 可输入 RTT 命令: reset, offset, density 等
}
```

### Step 3 — CMake 集成

在 `UserApp/CMakeLists.txt` 中添加：

```cmake
if(RTT_DEBUG)
    add_subdirectory(Debug/rtt_ms5837_cal_debug)
endif()
```

并在 `Debug/rtt_ms5837_cal_debug/CMakeLists.txt` 中定义独立可执行目标。

### Step 4 — 编译与测试

```bash
cmake -DRTT_DEBUG=ON -B build_debug
cmake --build build_debug
# 用 J-Link + RTT Viewer 查看输出
```

---

## 五、RTT 输出格式建议

```
D= +0.032 T=24.56  (有效数据)
D= +0.031 T=24.55
D= +0.000 T=00.00  (异常，红色标记)
ERR: I2C timeout   (错误信息)
```

---

## 六、注意事项

- RTT 需要 J-Link 调试器支持（STM32H7 的 SWD 口）
- 若 PA13(SWDIO) 已被 UART4 TX 占用，需要硬件上分配调试口
- `RTT_DEBUG` 模式下不初始化 FreeRTOS，裸机运行
- 建议先用 I2C 模式验证，再用 UART 解算板模式
