#include "SystemContext.hpp"
#include "SystemConfig.hpp"

// --- DMA 缓冲区定义 (位于 RAM_D2) ---
__attribute__((section(".dma_buffer"))) uint8_t ins_rx_buffer[512];
__attribute__((section(".dma_buffer")))
auv::device::MotionController_Driver::ThrustPacket motor_tx_packet;

// --- 底层驱动全局变量实例化 ---
namespace auv {
namespace device {
INS_Driver ins_driver(&huart7, &huart7, ins_rx_buffer, 512);
MotionController_Driver motor_driver(&huart6, &motor_tx_packet);
MS5837 depth_sensor(&hi2c1);
} // namespace device
} // namespace auv

namespace auv {
namespace control {
ChassisManager chassis;
}
}

namespace auv {
namespace system {

SystemContext system_context{};

bool SystemContext::getNavigationValid() const {
    bool state_ok = (nav_status.imu_state == 3 || nav_status.imu_state == 4);
    if (auv::config::sys_config.simulation.hitl_enabled) {
        return state_ok;
    }
    return (state_ok && auv::device::ins_driver.isDataFresh());
}

} // namespace system
} // namespace auv
