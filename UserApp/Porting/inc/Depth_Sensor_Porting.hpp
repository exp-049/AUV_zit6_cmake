#pragma once

#include "Depth_Sensor_Driver.hpp"
#include "Pushrod_Driver.hpp"

#define DEPTH_BACKEND_COMMERCIAL_UART 1
#define DEPTH_BACKEND_SELF_UART 2
#define DEPTH_BACKEND_I2C 3

#ifndef DEPTH_SENSOR_BACKEND
#define DEPTH_SENSOR_BACKEND DEPTH_BACKEND_SELF_UART
#endif

namespace auv {
namespace porting {

auv::peripheral::Depth_Sensor_Driver *getDepthSensorDriver();
auv::peripheral::Pushrod_Driver *getPushrodDriver();

} // namespace porting
} // namespace auv
