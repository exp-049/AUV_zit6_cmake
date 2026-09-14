#pragma once

#include "Depth_Sensor_Driver.hpp"
#include "Pushrod_Driver.hpp"

namespace auv {
namespace porting {

// The selected hardware preset owns the physical backends while exposing
// independent depth and pushrod facades to AppContext.
auv::peripheral::Depth_Sensor_Driver *getDepthSensorDriver();
auv::peripheral::Pushrod_Driver *getPushrodDriver();

} // namespace porting
} // namespace auv
