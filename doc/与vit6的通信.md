# 与 VIT6 下位机的通信协议（按当前代码实现）

本文档对应当前仓库中的：
- `UserApp/Peripherals/inc/MotionController_Driver.hpp`
- `UserApp/Peripherals/src/MotionController_Driver.cpp`
- `UserApp/Porting/src/MotionController_Porting.cpp`
- `UserApp/Common/AppContext.cpp`

---

## 1. 物理链路

当前动力控制板（VIT6 下位机）绑定：
- 串口：`UART6`
- 发送方式：`HAL_UART_Transmit_DMA`
- Porting 实现：`MotionController_Porting`

发送前会执行：
- DMA Stream ready 检查
- D-Cache Clean
- 使用 `.dma_buffer` 段中的对齐缓冲区

---

## 2. 协议总览

当前协议是固定头尾的定长二进制帧，不带 CRC 字段：

```text
FA AF [ID] [Payload...] FB BF
```

头尾定义：
- Head：`0xFA 0xAF`
- Tail：`0xFB 0xBF`

当前固件已实现的命令 ID：
- `0x01`：推力 / 力矩下发
- `0x02`：舵机角度
- `0x03`：灯光状态
- `0x07`：推力曲线配置

---

## 3. 0x01 推力下发

对应结构体：`ThrustPacket`

```cpp
struct __attribute__((packed)) ThrustPacket {
  uint8_t head[2];
  uint8_t id;
  float Fx, Fy, Fz;
  float Fyaw, Fpitch, Froll;
  uint8_t tail[2];
};
```

### 3.1 发送字段顺序

```text
FA AF 0x01 Fx Fy Fz Fyaw Fpitch Froll FB BF
```

### 3.2 当前代码中的坐标映射

上层 `publishThrust(fx, fy, fz, fyaw, fp, fr)` 进入驱动后，会映射为：

```cpp
thrust_pkt_ptr_->Fx     = -fy;
thrust_pkt_ptr_->Fy     =  fx;
thrust_pkt_ptr_->Fz     =  fz;
thrust_pkt_ptr_->Fyaw   =  fyaw;
thrust_pkt_ptr_->Fpitch =  fr;
thrust_pkt_ptr_->Froll  = -fp;
```

因此协议帧中的字段不是简单地“原样透传上层参数”，而是已经做过一次坐标重排和符号变换。

### 3.3 当前实际使用情况

- 上层 micro-ROS 主控指令仍然主要是 4-DOF：`X, Y, Z, Yaw`
- 但动力板链路已经支持 6 个 float：
  - 三轴力：`Fx, Fy, Fz`
  - 三轴力矩：`Fyaw, Fpitch, Froll`

所以旧说明里“`fpitch` / `froll` 一直都是 0，不需要”不再适合作为当前协议描述。

---

## 4. 0x07 推力曲线配置

对应结构体：`CurvePacket`

```cpp
struct __attribute__((packed)) CurvePacket {
  uint8_t head[2];
  uint8_t id;
  uint8_t mode;
  uint8_t index;
  float pwm[4];
  float thrust[4];
  uint8_t tail[2];
};
```

### 4.1 帧格式

```text
FA AF 0x07 mode index pwm0 pwm1 pwm2 pwm3 thrust0 thrust1 thrust2 thrust3 FB BF
```

### 4.2 字段含义

- `mode`
  - `0`：Read
  - `1`：Write
- `index`
  - 电机编号
- `pwm[4]`
  - 四个 PWM 标定点
- `thrust[4]`
  - 对应的四个推力标定点

---

## 5. 0x02 舵机命令

对应结构体：`ServoPacket`

```cpp
struct __attribute__((packed)) ServoPacket {
  uint8_t head[2];
  uint8_t id;
  float angle;
  uint8_t tail[2];
};
```

帧格式：

```text
FA AF 0x02 angle FB BF
```

说明：
- `angle` 单位为弧度（rad）
- 由 `/zit6/cmd/servo` 订阅回调调用 `setServoAngle()` 发送

---

## 6. 0x03 灯光命令

对应结构体：`LightPacket`

```cpp
struct __attribute__((packed)) LightPacket {
  uint8_t head[2];
  uint8_t id;
  uint8_t state;
  uint8_t tail[2];
};
```

帧格式：

```text
FA AF 0x03 state FB BF
```

说明：
- `state` 的具体位定义由下位机决定
- 上位机当前通过 `/zit6/cmd/light` 下发 `UInt8`

---

## 7. 当前固件接口

对应 `MotionController_Driver` 对外接口：

```cpp
void publishThrust(float fx, float fy, float fz, float fyaw, float fp = 0, float fr = 0);
void setThrustCurve(uint8_t mode, uint8_t index, const float pwm[4], const float thrust[4]);
void setServoAngle(float angle);
void setLightState(uint8_t state);
```

---

## 8. 与旧草稿相比的修正

本次同步后的关键修正：

1. 串口修正为 `UART6`，不是文中早期草稿写的其他口。
2. 协议说明对齐到当前真实结构体，不再按口语化字节描述。
3. 明确当前实现没有 CRC 字段。
4. 明确推力通道已是 6 float，而不是只有 4 float。
5. 明确 `Fx/Fy/Fz/Fyaw/Fpitch/Froll` 在驱动内存在坐标重排与符号映射。
