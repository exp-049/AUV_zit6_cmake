#pragma once

// INS_Driver 桩 — 用于 SafetyMonitor 测试
// 提供 isDataFresh() 方法，使 getNavigationValid() 可编译

#include <cstdint>

namespace auv {
namespace peripheral {

class INS_Driver {
public:
  bool isDataFresh() const { return data_fresh_; }
  void setDataFresh(bool fresh) { data_fresh_ = fresh; }

private:
  bool data_fresh_ = false;
};

} // namespace peripheral
} // namespace auv
