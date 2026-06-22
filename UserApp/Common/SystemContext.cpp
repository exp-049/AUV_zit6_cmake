#include "SystemContext.hpp"
#include "AppContext.hpp"
#include "SystemConfig.hpp"

namespace auv {
namespace system {

SystemContext system_context{};

bool SystemContext::getNavigationValid() const {
  auto ns = nav_status_.get();
  bool state_ok = (ns.imu_state == 3 || ns.imu_state == 4);
  if (auv::config::sys_config.simulation.hitl_enabled) {
    return state_ok;
  }
  return (state_ok && g_app_ctx.ins_driver->isDataFresh());
}

} // namespace system
} // namespace auv
