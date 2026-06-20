#include "ControlTask.hpp"
#include "AppMain.hpp"
#include "AuvSimulator.hpp"
#include "FreeRTOS.h"
#include "MotionContext.hpp"
#include "RosLogger.hpp"
#include "SerialPort.hpp"
#include "SoftWatchdog.hpp"
#include "SystemConfig.hpp"
#include "SystemContext.hpp"
#include "task.h"
#include <cstdio>
#include <string.h>
// No global using namespace directives

// HITL 仿真模式 — 默认关闭；CMake Debug 预设可添加 -DAUV_SIMULATION_ENABLE
// 生产固件中仿真代码完全被编译器剔除，零开销
#if AUV_SIMULATION_ENABLE
static auv::control::AuvSimulator g_hitl_sim(0.01f);
#endif

void ControlTask::run() {
  init();

  for (;;) {
    refreshHardwareWatchdogIfNeeded();

    // dt 由 vTaskDelayUntil 保证为严格 10ms，固定值避免 HAL_GetTick 截断抖动
    auv::motion::motion_context.setLastDtMs(static_cast<float>(kLoopPeriodMs));

    auv::motion::NavState nav = updateNavigation();
    handleArmState(HAL_GetTick());
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

  // 1. 获取原始导航输入
#if AUV_SIMULATION_ENABLE
  if (auv::config::sys_config.simulation.hitl_enabled) {
    auto p = g_hitl_sim.getPosition();
    auto v = g_hitl_sim.getVelocity();
    for (int i = 0; i < 6; i++) {
      state.pos_world[i] = p[i];
      state.vel_body[i] = v[i];
    }
    auv::system::system_context.nav_status.imu_state = 4;
    auv::system::system_context.nav_status.timestamp = HAL_GetTick();
  } else
#endif
  {
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

  // 2. 应用解锁原点平移与旋转变换（6DOF 矩阵版本）
  // 通过 getHomeOffset() 一次临界区获取全部偏置，避免外层关中断
  auto home = auv::motion::motion_context.getHomeOffset();
  bool use_offset = home.active;
  const auto &offset = home.offset;

  if (use_offset) {
    // η_diff = η_raw - η_home（6DOF 向量减法）
    float diff[6];
    for (int i = 0; i < 6; i++)
      diff[i] = state.pos_world[i] - offset[i];

    // η_home = R(η_offset)⁻¹ · η_diff
    // 由于 setHomeOffset 强制 offset[3]=offset[4]=0，
    // T⁻¹ 退化为单位阵，位置部分退化为 R_z(-ψ) 2D 旋转。
    auv::math::applyRotationToBody(diff, state.pos_world.data(), offset[3],
                                   offset[4], offset[5]);

    // 角度归一化
    for (int i = 3; i < 6; i++) {
      state.pos_world[i] =
          auv::motion::MotionContext::wrapAngle(state.pos_world[i]);
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
        // Roll/Pitch 强制为 0：AUV 重心浮心拉开自然稳定，
        // 防止上锁时倾斜扰动导致控制坐标系偏移，增强健壮性
        {
          auv::math::Vector6f home_offset;
          home_offset << nav_state.pos_world[0], // X
              nav_state.pos_world[1],            // Y
              nav_state.pos_world[2],            // Z
              0.0f,                              // Roll 强制为 0
              0.0f,                              // Pitch 强制为 0
              nav_state.pos_world[5];            // Yaw 正常记录
          auv::motion::motion_context.setHomeOffset(home_offset);
        }

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
  auto forces = auv::control::chassis.update();

#if AUV_SIMULATION_ENABLE
  if (auv::config::sys_config.simulation.hitl_enabled) {
    g_hitl_sim.step(forces);
  }
#endif

  auv::motion::motion_context.setLastOutputForces(forces);

  taskENTER_CRITICAL();
  const bool armed = auv::system::system_context.is_system_armed;
  taskEXIT_CRITICAL();

  if (armed) {
    // forces: [X=0, Y=1, Z=2, ROLL=3, PITCH=4, YAW=5]
    // publishThrust(fx, fy, fz, fyaw, fpitch, froll)
    auv::device::motor_driver.publishThrust(forces[0], forces[1], forces[2],
                                            forces[5],  // fyaw
                                            forces[4],  // fpitch
                                            forces[3]); // froll
  } else {
    auv::device::motor_driver.publishThrust(0, 0, 0, 0, 0, 0);
  }
}

void UserApp_ControlTask(void *argument) {
  (void)argument;
  ControlTask runner;
  runner.run();
}
