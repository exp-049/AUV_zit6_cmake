#include "ControlTask.hpp"
#include "AppMain.hpp"
#include "AuvSimulator.hpp"
#include "FreeRTOS.h"
#include "SerialPort.hpp"
#include "SoftWatchdog.hpp"
#include "SystemConfig.hpp"
#include "task.h"
#include <cstdio>
#include <string.h>

using namespace auv::device;
using namespace auv::control;

// 仿真引擎实例 (HITL 模式)
static AuvSimulator g_hitl_sim(0.01f);
static bool g_sim_inited = false;

void ControlTask::fillActualState(const auv::common::NavState &nav,
                                  float (&actual_p)[4], float (&actual_v)[4]) {
  actual_p[0] = nav.x;
  actual_p[1] = nav.y;
  actual_p[2] = nav.z;
  actual_p[3] = nav.yaw;

  actual_v[0] = nav.vx;
  actual_v[1] = nav.vy;
  actual_v[2] = nav.vz;
  actual_v[3] = nav.vyaw;
}

void ControlTask::run() {
  init();

  for (;;) {
    refreshHardwareWatchdogIfNeeded();

    const uint32_t now = HAL_GetTick();
    last_dt_ms = static_cast<float>(now - last_tick_);
    last_tick_ = now;

    auv::common::NavState nav = updateNavigation();
    handleArmState(nav, now);
    computeAndPublish(nav);

    // --- USBL 数据更新 (每周期轮询，解析到新帧即更新共享状态) ---
    {
      auv::UsblState usbl_temp;
      if (usbl_driver.update(usbl_temp)) {
        taskENTER_CRITICAL();
        shared_usbl_state = usbl_temp;
        taskEXIT_CRITICAL();
      }
    }

    // 周期性调试信息（非阻塞，通过 UART5 DMA，忙时丢弃）
    static uint32_t last_log_ms = 0;
    if (now - last_log_ms >= 1000) {
      last_log_ms = now;
      char dbgbuf[128];
      int n = std::snprintf(
          dbgbuf, sizeof(dbgbuf), "DBG t=%lu dt=%.1f z=%.2f armed=%d\r\n",
          (unsigned long)now, last_dt_ms, nav.z, is_system_armed ? 1 : 0);
      if (n > 0) {
        auv::porting::SerialPort::transmitDebug(
            reinterpret_cast<const uint8_t *>(dbgbuf),
            (uint16_t)(n > (int)sizeof(dbgbuf) ? (int)sizeof(dbgbuf) : n));
      }
    }

    vTaskDelayUntil(&last_wake_time_, pdMS_TO_TICKS(kLoopPeriodMs));
  }
}

void ControlTask::init() {
  memset(ins_rx_buffer, 0, sizeof(ins_rx_buffer));
  // 初始化 motor_tx_packet：保留/设置帧头帧尾和 id，清零有效载荷字段
  taskENTER_CRITICAL();
  motor_tx_packet.head[0] = 0xFA;
  motor_tx_packet.head[1] = 0xAF;
  motor_tx_packet.id = 0x01;
  motor_tx_packet.Fx = 0.0f;
  motor_tx_packet.Fy = 0.0f;
  motor_tx_packet.Fz = 0.0f;
  motor_tx_packet.Fyaw = 0.0f;
  motor_tx_packet.Fpitch = 0.0f;
  motor_tx_packet.Froll = 0.0f;
  motor_tx_packet.tail[0] = 0xFB;
  motor_tx_packet.tail[1] = 0xBF;
  taskEXIT_CRITICAL();

  ins_driver.init();
  usbl_driver.init();
  SoftWatchdog::getInstance().init(auv::config::sys_config.soft_watchdog);

  // 初始化底盘 PID 参数与运动学约束 (从 SystemConfig 加载)
  auv::control::chassis.applyConfig(auv::config::sys_config.chassis);

  // 简单测试：非阻塞地通过 UART5 发送一条调试信息（忙时丢弃）
  const char test_msg[] = "DEBUG: UART5 OK\r\n";
  auv::porting::SerialPort::transmitDebug(
      reinterpret_cast<const uint8_t *>(test_msg), sizeof(test_msg) - 1);

  last_wake_time_ = xTaskGetTickCount();
  last_tick_ = HAL_GetTick();
}

void ControlTask::refreshHardwareWatchdogIfNeeded() {
  if (SoftWatchdog::getInstance().check()) {
    HAL_IWDG_Refresh(&hiwdg1);
  }
}

auv::common::NavState ControlTask::updateNavigation() {
  auv::common::NavState nav;

  // 锁定逻辑：如果已经解锁，则根据当时是否触发了仿真初始化来决定数据源，不再受运行时
  // config 突变影响
  bool use_sim = is_system_armed
                     ? g_sim_inited
                     : auv::config::sys_config.simulation.hitl_enabled;

  // 检查是否启用 HITL 仿真模式
  if (use_sim) {
    if (!g_sim_inited) {
      // 首次启动仿真，尝试对齐当前传感器位置（如果有的话）
      auto hardware_nav = auv::shared::snapshotNavState();
      float p0[4] = {hardware_nav.x, hardware_nav.y, hardware_nav.z,
                     hardware_nav.yaw};
      g_hitl_sim.reset(p0);
      g_sim_inited = true;
    }

    auto p = g_hitl_sim.getPosition();
    auto v = g_hitl_sim.getVelocity();
    nav.x = p[0];
    nav.y = p[1];
    nav.z = p[2];
    nav.yaw = p[3];
    nav.vx = v[0];
    nav.vy = v[1];
    nav.vz = v[2];
    nav.vyaw = v[3];
    nav.imu_state = 4; // 强制模拟为最优导航状态 (Mode 4)
    nav.timestamp = HAL_GetTick();
  } else {
    // 正常：读取原始硬件数据
    nav = auv::shared::snapshotNavState();
    ins_driver.update(nav);

    // 根据配置选择是否使用独立的 MS5837 深度覆盖融合深度
    if (auv::config::sys_config.sensors.z_data_source ==
        auv::config::ZDataSource::USE_MS5837_Z) {
      float depth_snapshot = 0.0f;
      taskENTER_CRITICAL();
      depth_snapshot = current_depth_z;
      taskEXIT_CRITICAL();

      nav.z = depth_snapshot;
    } else if (auv::config::sys_config.sensors.z_data_source ==
               auv::config::ZDataSource::USE_INS_PRESSURE_Z) {
      nav.z = ins_driver.getManometerZ();
    }
    g_sim_inited = false; // 退出仿真时重置标记
  }

  taskENTER_CRITICAL();
  shared_nav_state = nav;
  taskEXIT_CRITICAL();

  return nav;
}

void ControlTask::setControlLevelNone(const auv::common::NavState &nav) {
  float actual_p[4];
  float actual_v[4];
  fillActualState(nav, actual_p, actual_v);
  chassis.setControlLevel(auv::common::ControlLevel::NONE, actual_p, actual_v);
}

void ControlTask::forceDisarmWithNeutralLevel(
    const auv::common::NavState &nav) {
  taskENTER_CRITICAL();
  is_system_armed = false;
  arm_heartbeat_count = 0;
  auv::device::ins_driver.clearHomeOffset(); // 失锁时恢复原始坐标系
  taskEXIT_CRITICAL();
  setControlLevelNone(nav);
}

void ControlTask::handleArmState(const auv::common::NavState &nav,
                                 uint32_t now) {
  taskENTER_CRITICAL();
  const bool armed_snapshot = is_system_armed;
  const uint32_t heartbeat_snapshot = last_arm_heartbeat_ms;
  const uint32_t heartbeat_count_snapshot = arm_heartbeat_count;
  const uint32_t arm_start_snapshot = arm_start_ms;
  taskEXIT_CRITICAL();

  if (armed_snapshot) {
    if (now - heartbeat_snapshot > kArmedHeartbeatTimeoutMs) {
      forceDisarmWithNeutralLevel(nav);
    }
    return;
  }

  if (chassis.getControlLevel() != auv::common::ControlLevel::NONE) {
    setControlLevelNone(nav);
  }

  if (heartbeat_count_snapshot >= kArmMinHeartbeatCount &&
      (now - arm_start_snapshot >= kArmMinDurationMs)) {
    taskENTER_CRITICAL();
    const uint32_t hbt_data = last_arm_heartbeat_data;
    taskEXIT_CRITICAL();

    // 允许解锁逻辑：
    // 1. 数据为 kRemoteModeHeartbeatData (3)
    // 2. 数据为 1 且 (导航有效 或 处于仿真模式)
    // 只有导航模式 >= 4 (SINS/DVL) 或仿真模式时，yaw 才足够稳定允许 Arm
    bool nav_ready = false;
    if (auv::config::sys_config.simulation.hitl_enabled) {
        nav_ready = true;
    } else {
        // imu_state: 4=SINS/DVL, 5=MRU（纯姿态无位置增量）
        nav_ready = (nav.imu_state >= 4);
    }

    bool can_arm =
        (hbt_data == kRemoteModeHeartbeatData) ||
        (hbt_data == 1 && nav_ready);

    if (can_arm) {
      taskENTER_CRITICAL();
      if (!is_system_armed) {
        // 解锁瞬间的行为锁定：
        // 1. 锁定仿真模式状态：如果在此时开启了仿真，则整个 Arm
        // 周期都应维持仿真 (此处通过 g_sim_inited 标志位配合 sys_config
        // 实现逻辑锁定)

        // 2. 注入驱动层偏移（建立“家”坐标系）
        // 注意：在仿真模式下，nav 已经是相对坐标，但 setHomeOffset
        // 会处理初始对齐
        auv::device::ins_driver.setHomeOffset(nav.x, nav.y, nav.z, nav.yaw);

        // 3. 锁定控制器目标为当前点（即新坐标系的 0 点）
        target_p[0] = 0.0f;
        target_p[1] = 0.0f;
        target_p[2] = 0.0f;
        target_p[3] = 0.0f;
      }
      is_system_armed = true;
      taskEXIT_CRITICAL();

      const char amsg[] = "INFO: System ARMED\r\n";
      auv::porting::SerialPort::transmitDebug((uint8_t *)amsg,
                                              sizeof(amsg) - 1);
    } else {
      // 如果是因为导航无效导致的无法解锁，打印提示
      if (hbt_data == 1 && !(auv::shared::isNavigationValid(nav) ||
                             auv::config::sys_config.simulation.hitl_enabled)) {
        static uint32_t last_warn_ms = 0;
        if (now - last_warn_ms > 2000) {
          last_warn_ms = now;
          const char msg[] = "WARN: Arm denied - Navigation NOT valid\r\n";
          auv::porting::SerialPort::transmitDebug((uint8_t *)msg,
                                                  sizeof(msg) - 1);
        }
      }
      taskENTER_CRITICAL();
      arm_heartbeat_count = 0;
      taskEXIT_CRITICAL();
    }
  }

  if (now - heartbeat_snapshot > kDisarmedHeartbeatTimeoutMs) {
    taskENTER_CRITICAL();
    arm_heartbeat_count = 0;
    taskEXIT_CRITICAL();
  }
}

void ControlTask::computeAndPublish(const auv::common::NavState &nav) {
  float actual_p[4];
  float actual_v[4];
  fillActualState(nav, actual_p, actual_v);

  float target_snapshot[4];
  taskENTER_CRITICAL();
  for (int i = 0; i < 4; ++i) {
    target_snapshot[i] = target_p[i];
  }
  taskEXIT_CRITICAL();

  auto forces = chassis.update(actual_p, actual_v, target_snapshot);

  // 如果满足仿真锁定状态，将计算出的推力喂回仿真引擎
  bool use_sim = is_system_armed
                     ? g_sim_inited
                     : auv::config::sys_config.simulation.hitl_enabled;
  if (use_sim && g_sim_inited) {
    float k = auv::config::sys_config.simulation.thrust_k;
    float sim_m = auv::config::sys_config.simulation.mass;
    float sim_d = auv::config::sys_config.simulation.drag;
    
    // 对各轴分配合理的物理质量与阻力，避免使用非常小的前馈质量导致发散
    std::array<float, 4> masses = {sim_m, sim_m, sim_m, sim_m * 0.1f};
    std::array<float, 4> drags = {sim_d, sim_d, sim_d, sim_d * 0.1f};
    g_hitl_sim.step(forces, masses, drags, k);
  }

  taskENTER_CRITICAL();
  for (int i = 0; i < 4; ++i) {
    last_output_forces[i] = forces[i];
  }
  const bool armed = is_system_armed;
  taskEXIT_CRITICAL();

  if (armed) {
    motor_driver.publishThrust(forces[0], forces[1], forces[2], forces[3]);
  } else {
    motor_driver.publishThrust(0, 0, 0, 0);
  }
}

void UserApp_ControlTask(void *argument) {
  (void)argument;
  ControlTask runner;
  runner.run();
}
