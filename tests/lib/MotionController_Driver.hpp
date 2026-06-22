#pragma once

// MotionController_Driver 桩 — 用于主机端测试
// SystemContext.hpp 需要此类型完整定义

namespace auv {
namespace peripheral {

class MotionController_Driver {
public:
  void publishThrust(float a, float b, float c, float d, float e, float f) {}
};

} // namespace peripheral
} // namespace auv
