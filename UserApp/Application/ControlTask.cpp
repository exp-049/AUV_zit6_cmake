#include "ControlTask.hpp"
#include "MotionContext.hpp"
#include "SystemContext.hpp"
#include "AuvSimulator.hpp"
#include "FreeRTOS.h"
#include "SerialPort.hpp"
#include "SoftWatchdog.hpp"
#include "SystemConfig.hpp"
#include "AppMain.hpp"
#include "task.h"
#include <cstdio>
#include <string.h>

using namespace auv::device;
using namespace auv::control;

// 仿真引擎实例 (HITL 模式)
static AuvSimulator g_hitl_sim(0.01f);



void ControlTask::run() {
  init();

  for (;;) {
    refreshHardwareWatchdogIfNeeded();

    const uint32_t now = HAL_GetTick();
    auv::motion::motion_context.setLastDtMs(static_cast<float>(now - last_tick_));
    last_tick_ = now;

    auv::motion::NavState nav = updateNavigation();
    handleArmState(now);
    computeAndPublish();

    // 周期性调试信息（非阻塞，通过 UART5 DMA，忙时丢弃）
    static uint32_t last_log_ms = 0;
    if (now - last_log_ms >= 1000) {
      last_log_ms = now;
      char dbgbuf[128];
      int n = std::snprintf(
          dbgbuf, sizeof(dbgbuf), "DBG t=%lu dt=%.1f z=%.2f armed=%d\r\n",
          (unsigned long)now, auv::motion::motion_context.getLastDtMs(), nav.pos_world[2], auv::system::system_context.is_system_armed ? 1 : 0);
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

auv::motion::NavState ControlTask::updateNavigation() {
  auv::motion::NavState state;

  // 1. 获取原始导航输入（仿真 vs 真实硬件）
  if (auv::config::sys_config.simulation.hitl_enabled) {
    auto p = g_hitl_sim.getPosition();
    auto v = g_hitl_sim.getVelocity();
    for (int i = 0; i < 4; i++) {
      state.pos_world[i] = p[i];
      state.vel_body[i] = v[i];
    }
    auv::system::system_context.nav_status.imu_state = 4; // 强制模拟为最优导航状态 (Mode 4)
    auv::system::system_context.nav_status.timestamp = HAL_GetTick();
  } else {
    // 正常：从硬件读取原始导航数据
    state = ins_driver.getNavState();
    ins_driver.update(state);

    // 根据配置覆盖 Z 轴深度数据源
    if (auv::config::sys_config.sensors.z_data_source == auv::config::ZDataSource::USE_MS5837_Z) {
      state.pos_world[2] = auv::motion::motion_context.getMS5837Z();
    } else if (auv::config::sys_config.sensors.z_data_source == auv::config::ZDataSource::USE_INS_PRESSURE_Z) {
      state.pos_world[2] = ins_driver.getManometerZ();
    }
  }

  // 2. 应用解锁原点平移与偏航旋转变换
  taskENTER_CRITICAL();
  bool use_offset = auv::motion::motion_context.use_offset_;
  float offset_x = auv::motion::motion_context.offset_x_;
  float offset_y = auv::motion::motion_context.offset_y_;
  float offset_z = auv::motion::motion_context.offset_z_;
  float offset_yaw = auv::motion::motion_context.offset_yaw_;
  taskEXIT_CRITICAL();

  if (use_offset) {
    float dx = state.pos_world[0] - offset_x;
    float dy = state.pos_world[1] - offset_y;
    float cos_h = std::cos(offset_yaw);
    float sin_h = std::sin(offset_yaw);
    state.pos_world[0] = dx * cos_h + dy * sin_h;
    state.pos_world[1] = -dx * sin_h + dy * cos_h;
    state.pos_world[2] -= offset_z;
    state.pos_world[3] -= offset_yaw;

    // 航向角归一化
    while (state.pos_world[3] > 3.14159265f)
      state.pos_world[3] -= 6.2831853f;
    while (state.pos_world[3] < -3.14159265f)
      state.pos_world[3] += 6.2831853f;
  }

  // 3. 将融合后的位姿写入全局的 MotionContext
  auv::motion::motion_context.setNavState(state);

  return state;
}

void ControlTask::setControlLevelNone() {
  chassis.setControlLevel(auv::motion::ControlLevel::NONE);
}

void ControlTask::forceDisarmWithNeutralLevel() {
  taskENTER_CRITICAL();
  auv::system::system_context.is_system_armed = false;
  auv::system::system_context.arm_heartbeat_count = 0;
  auv::motion::motion_context.clearHomeOffset(); // 失锁时恢复原始坐标系
  taskEXIT_CRITICAL();
  setControlLevelNone();
}

void ControlTask::handleArmState(uint32_t now) {
  taskENTER_CRITICAL();
  const bool armed_snapshot = auv::system::system_context.is_system_armed;
  const uint32_t heartbeat_snapshot = auv::system::system_context.last_arm_heartbeat_ms;
  const uint32_t heartbeat_count_snapshot = auv::system::system_context.arm_heartbeat_count;
  const uint32_t arm_start_snapshot = auv::system::system_context.arm_start_ms;
  taskEXIT_CRITICAL();

  if (armed_snapshot) {
    if (now - heartbeat_snapshot > kArmedHeartbeatTimeoutMs) {
      forceDisarmWithNeutralLevel();
    }
    return;
  }

  if (chassis.getControlLevel() != auv::motion::ControlLevel::NONE) {
    setControlLevelNone();
  }

  if (heartbeat_count_snapshot >= kArmMinHeartbeatCount &&
      (now - arm_start_snapshot >= kArmMinDurationMs)) {
    taskENTER_CRITICAL();
    const uint32_t hbt_data = auv::system::system_context.last_arm_heartbeat_data;
    taskEXIT_CRITICAL();

    // 允许解锁逻辑：
    // 1. 数据为 kRemoteModeHeartbeatData (3)
    // 2. 数据为 1 且 (导航有效 或 处于仿真模式)
    bool can_arm =
        (hbt_data == kRemoteModeHeartbeatData) ||
        (hbt_data == 1 && (auv::system::system_context.getNavigationValid() ||
                           auv::config::sys_config.simulation.hitl_enabled));

    if (can_arm) {
      taskENTER_CRITICAL();
      if (!auv::system::system_context.is_system_armed) {
        // 解锁瞬间的行为锁定：
        // 1. 锁定仿真模式状态：如果在此时开启了仿真，则整个 Arm
        // 周期都应维持仿真 (此处通过 g_sim_inited 标志位配合 sys_config
        // 实现逻辑锁定)

        // 2. 注入偏移以建立“家”坐标系
        // 注意：在仿真模式下，nav 已经是相对坐标，但 setHomeOffset 会处理初始对齐
        auto nav_state = auv::motion::motion_context.getNavState();
        auv::motion::motion_context.setHomeOffset(nav_state.pos_world[0], nav_state.pos_world[1], nav_state.pos_world[2], nav_state.pos_world[3]);

        // 3. 锁定控制器目标为当前点（即新坐标系的 0 点）
        auv::motion::motion_context.resetSetpoint();
      }
      auv::system::system_context.is_system_armed = true;
      taskEXIT_CRITICAL();

      const char amsg[] = "INFO: System ARMED\r\n";
      auv::porting::SerialPort::transmitDebug((uint8_t *)amsg,
                                              sizeof(amsg) - 1);
    } else {
      // 如果是因为导航无效导致的无法解锁，打印提示
      if (hbt_data == 1 && !(auv::system::system_context.getNavigationValid() ||
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
      auv::system::system_context.arm_heartbeat_count = 0;
      taskEXIT_CRITICAL();
    }
  }

  if (now - heartbeat_snapshot > kDisarmedHeartbeatTimeoutMs) {
    taskENTER_CRITICAL();
    auv::system::system_context.arm_heartbeat_count = 0;
    taskEXIT_CRITICAL();
  }
}

void ControlTask::computeAndPublish() {
  auto forces = chassis.update();

  // 如果满足仿真锁定状态，将计算出的推力喂回仿真引擎
  if (auv::config::sys_config.simulation.hitl_enabled) {
    g_hitl_sim.step(forces);
  }

  auv::motion::motion_context.setLastOutputForces(forces);
  taskENTER_CRITICAL();
  const bool armed = auv::system::system_context.is_system_armed;
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
