#pragma once

// Depth_Sensor_Driver 桩 — 用于主机端测试
// SystemContext.hpp 需要此类型完整定义

namespace auv {
namespace peripheral {

class Depth_Sensor_Driver {
public:
  void Init() {}
  void start() {}
  int Read() { return 0; }
  void Depth(float *p) { *p = 0; }
  float getMS5837Z() { return 0; }
  void setMS5837Z(float z) {}
  bool is_connected = false;
};

} // namespace peripheral
} // namespace auv

namespace auv {
namespace peripheral {
using MS5837_Driver = Depth_Sensor_Driver;
} // namespace peripheral
} // namespace auv
