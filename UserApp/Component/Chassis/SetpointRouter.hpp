#pragma once

#include "MotionContext.hpp"
#include <stdint.h>

namespace auv {
namespace component {

/**
 * @class SetpointRouter
 * @brief 协议解析与坐标变换路由
 */
class SetpointRouter {
public:
  /**
   * @brief 解析并路由设定点
   * @param current_level 当前控制层级
   * @param new_level     目标控制层级
   * @param val           设定值数组 [X, Y, Z, Roll, Pitch, Yaw]
   * @param mask          掩码
   * @param is_body       是否为机体系
   * @param is_inc        是否为增量设定
   * @return 解析后的目标控制层级（传给 CascadeController::setControlLevel）
   */
  auv::motion::ControlLevel route(auv::motion::ControlLevel current_level,
                                  auv::motion::ControlLevel new_level,
                                  const float val[6], uint32_t mask,
                                  bool is_body, bool is_inc);
};

} // namespace component
} // namespace auv
