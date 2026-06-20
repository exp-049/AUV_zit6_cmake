#include "ControlTask.hpp"
#include "AppMain.hpp"
#include "AuvSimulator.hpp"
#include "FreeRTOS.h"
#include "MotionContext.hpp"
#include "SerialPort.hpp"
#include "SoftWatchdog.hpp"
#include "RosLogger.hpp"
#include "SystemConfig.hpp"
#include "SystemContext.hpp"
#include "task.h"
#include <cstdio>
#include <string.h>
// No global using namespace directives

// 仿真引擎实例 (HITL 模式)
static auv::control::AuvSimulator g_hitl_sim(0.01f);

void ControlTask::run() {
  init();

  for (;;) {
    refreshHardwareWatchdogIfNeeded();

    const uint32_t now = HAL_GetTick();
    auv::motion::motion_context.setLastDtMs(
        static_cast<float>(now - last_tick_));
    last_tick_ = now;

    auv::motion::NavState nav = updateNavigation();
    handleArmState(now);
    computeAndPublish();

    vTaskDelayUntil(&last_wake_time_, pdMS_TO_TICKS(kLoopPeriodMs));
  }
}

void ControlTask::init() {
  memset(ins_rx_buffer, 0, sizeof(ins_rx_buffer));

  auv::device::ins_driver.init();
  auv::device::SoftWatchdog::getInstance().init(
      auv::config::sys_config.soft_watchdog);

  // 初始化底盘 PID 参数与运动学约束 (从 SystemConfig 加载)
  auv::control::chassis.applyConfig(auv::config::sys_config.chassis);

  auv::device::RosLogger::getInstance().init();
  ROS_LOG_INFO("System ControlTask initialized");

  last_wake_time_ = xTaskGetTickCount();
  last_tick_ = HAL_GetTick();
}

void ControlTask::refreshHardwareWatchdogIfNeeded() {
  if (auv::device::SoftWatchdog::getInstance().check()) {
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
    auv::system::system_context.nav_status.imu_state =
        4; // 强制模拟为最优导航状态 (Mode 4)
    auv::system::system_context.nav_status.timestamp = HAL_GetTick();
  } else {
    // 正常：从硬件读取原始导航数据
    state = auv::device::ins_driver.getNavState();
    auv::device::ins_driver.update(state);

    // 根据配置覆盖 Z 轴深度数据源
    if (auv::config::sys_config.sensors.z_data_source ==
        auv::config::ZDataSource::USE_MS5837_Z) {
      state.pos_world[2] = auv::device::depth_sensor.getMS5837Z();
    } else if (auv::config::sys_config.sensors.z_data_source ==
               auv::config::ZDataSource::USE_INS_PRESSURE_Z) {
      state.pos_world[2] = auv::device::ins_driver.getManometerZ();
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
    state.pos_world[5] -= offset_yaw;  // Yaw 偏移

    // 角度归一化（Roll/Pitch/Yaw）
    for (int i = 3; i < 6; i++) {
      state.pos_world[i] = auv::motion::MotionContext::wrapAngle(state.pos_world[i]);
    }
  }

  // 3. 将融合后的位姿写入全局的 MotionContext
  auv::motion::motion_context.setNavState(state);

  return state;
}

void ControlTask::setControlLevelNone() {
  auv::control::chassis.setControlLevel(auv::motion::ControlLevel::NONE);
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
  const uint32_t heartbeat_snapshot =
      auv::system::system_context.last_arm_heartbeat_ms;
  const uint32_t heartbeat_count_snapshot =
      auv::system::system_context.arm_heartbeat_count;
  const uint32_t arm_start_snapshot = auv::system::system_context.arm_start_ms;
  taskEXIT_CRITICAL();

  if (armed_snapshot) {
    if (now - heartbeat_snapshot > kArmedHeartbeatTimeoutMs) {
      forceDisarmWithNeutralLevel();
      ROS_LOG_INFO("System DISARMED - Heartbeat timeout");
    }
    return;
  }

  if (auv::control::chassis.getControlLevel() !=
      auv::motion::ControlLevel::NONE) {
    setControlLevelNone();
  }

  if (heartbeat_count_snapshot >= kArmMinHeartbeatCount &&
      (now - arm_start_snapshot >= kArmMinDurationMs)) {
    taskENTER_CRITICAL();
    const uint32_t hbt_data =
        auv::system::system_context.last_arm_heartbeat_data;
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

        auto nav_state = auv::motion::motion_context.getNavState();
        auv::motion::motion_context.setHomeOffset(
            nav_state.pos_world[0], nav_state.pos_world[1],
            nav_state.pos_world[2],
            nav_state.pos_world[3],  // Roll
            nav_state.pos_world[4],  // Pitch
            nav_state.pos_world[5]); // Yaw

        // 3. 锁定控制器目标为当前点（即新坐标系的 0 点）
        auv::motion::motion_context.resetSetpoint();
      }
      auv::system::system_context.is_system_armed = true;
      taskEXIT_CRITICAL();

      ROS_LOG_INFO("System ARMED");
    } else {
      if (hbt_data == 1 && !(auv::system::system_context.getNavigationValid() ||
                             auv::config::sys_config.simulation.hitl_enabled)) {
        static uint32_t last_warn_ms = 0;
        if (now - last_warn_ms > 2000) {
          last_warn_ms = now;
          ROS_LOG_WARN("Arm denied - Navigation NOT valid");
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
  auto forces4 = auv::control::chassis.update();

  // 如果满足仿真锁定状态，将计算出的推力喂回仿真引擎
  if (auv::config::sys_config.simulation.hitl_enabled) {
    g_hitl_sim.step(forces4);
  }

  // 4DOF → 6DOF 转换（Step 5 后 chassis.update() 将直接返回 6DOF）
  std::array<float, 6> forces6;
  for (int i = 0; i < 4; i++) forces6[i] = forces4[i];
  forces6[4] = 0.0f;  // Pitch 推力暂为 0
  forces6[5] = 0.0f;  // Roll 推力暂为 0
  auv::motion::motion_context.setLastOutputForces(forces6);

  taskENTER_CRITICAL();
  const bool armed = auv::system::system_context.is_system_armed;
  taskEXIT_CRITICAL();

  if (armed) {
    auv::device::motor_driver.publishThrust(forces4[0], forces4[1], forces4[2],
                                            forces4[3]);
  } else {
    auv::device::motor_driver.publishThrust(0, 0, 0, 0);
  }
}

void UserApp_ControlTask(void *argument) {
  (void)argument;
  ControlTask runner;
  runner.run();
}
