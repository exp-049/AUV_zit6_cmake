#include "GlobalContext.hpp"
#include "FreeRTOS.h"
#include "task.h"

// --- DMA 缓冲区定义 (位于 RAM_D2) ---
__attribute__((section(".dma_buffer"))) uint8_t ins_rx_buffer[512];
__attribute__((section(".dma_buffer")))
auv::device::MotionController_Driver::ThrustPacket motor_tx_packet;
__attribute__((section(".dma_buffer"))) uint8_t usbl_rx_buffer[512];

// --- 驱动实例定义 ---
namespace auv {
namespace device {
INS_Driver ins_driver(&huart7, &huart7, ins_rx_buffer, 512);
MotionController_Driver motor_driver(&huart6, &motor_tx_packet);
MS5837 depth_sensor(&hi2c1);
USBL_Driver usbl_driver(&huart3);
} // namespace device

namespace control {
ChassisManager chassis;
}

// sys_config is defined in SystemConfig.cpp
} // namespace auv

// --- 共享变量定义 ---
float target_p[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float last_output_forces[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float last_dt_ms = 0.0f;
uint32_t last_received_seq = 0;
float current_depth_z = 0.0f;
volatile bool planner_replan_flag = false;

// --- USBL 共享状态 ---
auv::UsblState shared_usbl_state{};

bool is_system_armed = false;
uint32_t arm_heartbeat_count = 0;
uint32_t last_arm_heartbeat_ms = 0;
uint32_t last_arm_heartbeat_data = 0;
uint32_t arm_start_ms = 0;
auv::common::NavState shared_nav_state{};

namespace auv {
namespace shared {

bool isNavigationValid(const auv::common::NavState &nav) {
  // 仿真模式下只要状态正确即有效；非仿真模式下需检查硬件数据新鲜度
  bool state_ok = (nav.imu_state == 3 || nav.imu_state == 4);
  if (auv::config::sys_config.simulation.hitl_enabled) {
    return state_ok;
  }
  return (state_ok && auv::device::ins_driver.isDataFresh());
}

auv::common::NavState snapshotNavState() {
  auv::common::NavState nav;
  taskENTER_CRITICAL();
  nav = shared_nav_state;
  taskEXIT_CRITICAL();
  return nav;
}

} // namespace shared
} // namespace auv
