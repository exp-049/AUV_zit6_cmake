#include "ControlTask.hpp"
#include "AppMain.hpp"
#include "../Component/Chassis/ChassisManager.hpp"
#include "../Component/HitlSimulator/HitlSimulator.hpp"
#include "../Component/RosLogger/RosLogger.hpp"
#include "../Config/SystemConfig.hpp"
#include "../Common/SystemContext.hpp"
#include "../Peripherals/HAL/Core/Inc/main.h"
#include "../Peripherals/HAL/Middlewares/Third_Party/FreeRTOS/Source/include/FreeRTOS.h"
#include "../Peripherals/HAL/Middlewares/Third_Party/FreeRTOS/Source/include/task.h"
#include "../Porting/inc/INS_Porting.hpp"
#include "../Peripherals/inc/INS_Driver.hpp"
#include "../Peripherals/inc/MS5837_Driver.hpp"
#include "../Peripherals/inc/MS5837_LogConfig.hpp"
#include <cstring>

#if AUV_SIMULATION_ENABLE
static auv::component::HitlSimulator g_hitl_sim(0.01f);
#endif

void ControlTask::run() {
  init();

  last_tick_ = static_cast<uint32_t>(xTaskGetTickCount());
  first_cycle_ = true;

  for (;;) {
    const uint32_t loop_start_tick =
        static_cast<uint32_t>(xTaskGetTickCount());
    if (first_cycle_) {
      auv::motion::motion_context.last_dt_ms_.set(
          static_cast<float>(kLoopPeriodMs));
      first_cycle_ = false;
    } else {
      auv::motion::motion_context.last_dt_ms_.set(static_cast<float>(
          (loop_start_tick - last_tick_) * portTICK_PERIOD_MS));
    }
    last_tick_ = loop_start_tick;

    const uint32_t exec_start_ms = HAL_GetTick();
    updateNavigation();
    computeAndPublish();
    const uint32_t exec_ms = HAL_GetTick() - exec_start_ms;
    auv::motion::motion_context.last_exec_ms_.set(static_cast<float>(exec_ms));

    if (exec_ms > kLoopPeriodMs) {
      overrun_count_++;
      auv::motion::motion_context.control_overrun_count_.set(overrun_count_);
      ROS_LOG_WARN("ControlTask overrun: exec=%lu ms, count=%lu",
                   (unsigned long)exec_ms, (unsigned long)overrun_count_);
    }

    TickType_t wake_tick = static_cast<TickType_t>(last_wake_time_);
    vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(kLoopPeriodMs));
    last_wake_time_ = static_cast<uint32_t>(wake_tick);
  }
}

void ControlTask::init() {
  memset(ins_rx_buffer, 0, sizeof(ins_rx_buffer));

  ctx_->ins_driver->init();
  // NORMAL 只负责接收并缓存 USBL 数据，不改变当前 INS/深度数据源选择。
  ctx_->usbl_driver->init();
  ctx_->chassis->applyConfig(auv::config::sys_config.chassis);

  ctx_->logger->init();

  /* 创建 SITL 导航数据队列（长度 3，生产者 onSimNav → 消费者 updateNavigation）
   */
  if (auv::motion::motion_context.sitl_nav_queue == nullptr) {
    auv::motion::motion_context.sitl_nav_queue =
        xQueueCreate(3, sizeof(auv::motion::NavState));
  }

  /* 深度传感器初始化（Init 内部注册回调 + 初始化，start 创建任务/启动 DMA） */
  ctx_->depth_sensor->Init();
  ctx_->depth_sensor->start();

  ROS_LOG_INFO("System ControlTask initialized");

  last_wake_time_ = xTaskGetTickCount();
  last_tick_ = HAL_GetTick();
}

void ControlTask::updateNavigation() {
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
    {
      auto ns = auv::system::system_context.nav_status_.get();
      ns.imu_state = 4;
      ns.timestamp = HAL_GetTick();
      auv::system::system_context.nav_status_.set(ns);
    }
  } else
#endif
      if (auv::config::sys_config.simulation.sitl_enabled) {
    /* 从队列 drain 到最新（FIFO → 丢弃旧帧，只保留最新一帧） */
    {
      bool got_new = false;
      while (xQueueReceive(auv::motion::motion_context.sitl_nav_queue, &state,
                           0) == pdTRUE) {
        last_sitl_state_ = state;
        got_new = true;
      }
      if (!got_new) {
        state = last_sitl_state_;
      }
    }
    {
      auto ns = auv::system::system_context.nav_status_.get();
      ns.imu_state = 4;
      ns.timestamp = HAL_GetTick();
      auv::system::system_context.nav_status_.set(ns);
    }
  } else {
    state = ctx_->ins_driver->getNavState();
    ctx_->ins_driver->update(state);
    // 轮询 DMA 环形缓冲并解包最新 USBL 帧。数据暂不参与融合，供后续
    // 数据源选择/融合模块通过 AppContext::usbl_driver 读取。
    ctx_->usbl_driver->update(usbl_state_);

#ifdef USE_DEPTH_CALC_BOARD
    // 深度计算板是正式硬件链路：pos.z 必须始终来自 MS5837 后端。
    // Read() 先轮询 UART4 DMA 环形缓冲，再由 getMS5837Z() 取最近一帧
    // DATA.depth_cm / 100.0f，避免运行时配置把 z 切换到 INS 压力源。
    const int depth_frame_ready = ctx_->depth_sensor->Read();
    state.pos_world[2] = ctx_->depth_sensor->getMS5837Z();

    // 正式模式保留 1Hz、非阻塞诊断，便于区分“协议没有收到有效 DATA”
    // 与“有效 DATA 的深度确实为 0”。高频日志仍由 MS5837_LOG_ENABLE 控制。
    static uint32_t last_depth_diag_ms = 0U;
    const uint32_t depth_diag_now = HAL_GetTick();
    if ((uint32_t)(depth_diag_now - last_depth_diag_ms) >= 1000U) {
      last_depth_diag_ms = depth_diag_now;
      const long z_milli =
          (long)(state.pos_world[2] * 1000.0f +
                 (state.pos_world[2] >= 0.0f ? 0.5f : -0.5f));
      const long z_abs_milli = z_milli < 0L ? -z_milli : z_milli;
      const char *z_sign = z_milli < 0L ? "-" : "";
      MS5837_LOG_DIAG(
          "MS5837 main: frame=%d z=%s%ld.%03ld connected=%d ack=%d rx_events=%lu "
          "dma_pos=%lu recoveries=%lu errors=%lu last_error=%lu "
          "recovery_reason=%lu",
          depth_frame_ready, z_sign, z_abs_milli / 1000L, z_abs_milli % 1000L,
          ctx_->depth_sensor->is_connected ? 1 : 0,
          ctx_->depth_sensor->isHandshakeAcknowledged() ? 1 : 0,
          (unsigned long)ctx_->depth_sensor->getRxEventCount(),
          (unsigned long)ctx_->depth_sensor->getDmaWritePos(),
          (unsigned long)ctx_->depth_sensor->getRxRecoveryCount(),
          (unsigned long)ctx_->depth_sensor->getRxErrorCount(),
          (unsigned long)ctx_->depth_sensor->getLastRxError(),
          (unsigned long)ctx_->depth_sensor->getLastRxRecoveryReason());
    }
#else
    if (auv::config::sys_config.system.sensors.z_data_source ==
        auv::config::ZDataSource::USE_MS5837_Z) {
      ctx_->depth_sensor->Read();
      state.pos_world[2] = ctx_->depth_sensor->getMS5837Z();
    } else if (auv::config::sys_config.system.sensors.z_data_source ==
               auv::config::ZDataSource::USE_INS_PRESSURE_Z) {
      state.pos_world[2] = ctx_->ins_driver->getManometerZ();
    }
#endif
  }

  // 2. 应用解锁原点平移与旋转变换
  auto home = auv::motion::motion_context.home_offset_.get();
  bool use_offset = home.active;
  const auto &offset = home.offset;

  if (use_offset) {
    float diff[6];
    for (int i = 0; i < 6; i++)
      diff[i] = state.pos_world[i] - offset[i];

    auv::algorithm::math::applyRotationToBody(diff, state.pos_world.data(),
                                              offset[3], offset[4], offset[5]);

    for (int i = 3; i < 6; i++) {
      state.pos_world[i] =
          auv::motion::MotionContext::wrapAngle(state.pos_world[i]);
    }
  }

  // 3. 写入 MotionContext
  auv::motion::motion_context.nav_state_.set(state);
}

void ControlTask::computeAndPublish() {
  auto forces = ctx_->chassis->update();

#if AUV_SIMULATION_ENABLE
  if (auv::config::sys_config.simulation.hitl_enabled) {
    g_hitl_sim.step(forces);
  }
#endif

  auv::motion::motion_context.last_output_forces_.set(forces);

  const bool armed = auv::system::system_context.arm_state_.get().is_armed;
  const bool thrust_ok = armed
                             ? ctx_->motor_driver->publishThrust(
                                   forces[0], forces[1], forces[2], forces[5],
                                   forces[4], forces[3])
                             : ctx_->motor_driver->publishThrust(0, 0, 0, 0, 0, 0);

  if (!thrust_ok) {
    const uint32_t fail_count =
        auv::motion::motion_context.thrust_tx_fail_count_.get() + 1;
    auv::motion::motion_context.thrust_tx_fail_count_.set(fail_count);
  }
}

void UserApp_ControlTask(void *argument) {
  (void)argument;
  ControlTask runner(&auv::system::g_app_ctx);
  runner.run();
}
