#ifndef __COORDINATE_MANAGER_HPP
#define __COORDINATE_MANAGER_HPP

#include "CommonConfig.hpp"

namespace auv {

class CoordinateManager {
public:
  CoordinateManager();
  void updateBasePose(const auv::common::NavState &state);
  void body2World(float body_x, float body_y, float body_z, float &world_x,
                  float &world_y, float &world_z);
  void world2Body(float world_x, float world_y, float world_z, float &body_x,
                  float &body_y, float &body_z);

private:
  auv::common::NavState current_base_;
  void reset();
};

} // namespace auv

#endif
