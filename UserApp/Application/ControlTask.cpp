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
    safety_monitor_.check(HAL_GetTick());
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
  if (auv::config::sys_config.simulation.sitl_enabled) {
    // SITL 模式：从 micro-ROS 订阅的仿真数据注入
    if (auv::motion::motion_context.isSimDataValid()) {
      state = auv::motion::motion_context.getSimNavState();
      auv::system::system_context.nav_status.imu_state = 4;
      auv::system::system_context.nav_status.timestamp = HAL_GetTick();
    } else {
      // 仿真数据尚未到达，使用零值
      for (int i = 0; i < 6; ++i) {
        state.pos_world[i] = 0.0f;
        state.vel_body[i] = 0.0f;
      }
      auv::system::system_context.nav_status.imu_state = 0;
    }
  } else
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

  // 2. 应用解锁原点平移与旋转变换
  auto home = auv::motion::motion_context.getHomeOffset();
  bool use_offset = home.active;
  const auto &offset = home.offset;

  if (use_offset) {
    // η_diff = η_raw - η_home
    float diff[6];
    for (int i = 0; i < 6; i++)
      diff[i] = state.pos_world[i] - offset[i];

    // η_home = R(η_offset)⁻¹ · η_diff
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
