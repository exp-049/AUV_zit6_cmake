#ifndef __MS5837_DRIVER_HPP
#define __MS5837_DRIVER_HPP

#include "Depth_Sensor_Driver.hpp"

namespace auv {
namespace peripheral {

// Source compatibility for existing application and debug code.
using MS5837_Driver = Depth_Sensor_Driver;

} // namespace peripheral
} // namespace auv

#endif

// // 1. 触发读取并进行内部计算 (calculate)
// depthSensor.Read();

// // 2. 获取压力和温度
// float currentP = depthSensor.pressure;    // 单位：mbar 或 Pa
// (取决于你的宏定义) float currentT = depthSensor.temperture;  // 单位：摄氏度

// // 3. 计算深度
// float currentDepth = 0;
// depthSensor.Depth(&currentDepth); // 结果存入 currentDepth，单位：米

// // 打印数据调试
// printf("Depth: %.2f m, Temp: %.2f C\r\n", currentDepth, currentT);

// Delay(100); // 控制采样频率
