#include "ControlTask.hpp"
#include "Pushrod_Porting_Config.h"
#include "AppMain.hpp"
#include "../Component/Chassis/ChassisManager.hpp"
#include "../Component/HitlSimulator/HitlSimulator.hpp"
#include "../Component/RosLogger/RosLogger.hpp"
#include "../Config/SystemConfig.hpp"
#include "../Common/SystemContext.hpp"
#include "../Peripherals/HAL/Core/Inc/main.h"
#include "../Peripherals/HAL/Middlewares/Third_Party/FreeRTOS/Source/include/FreeRTOS.h"
#include "../Peripherals/HAL/Middlewares/Third_Party/FreeRTOS/Source/include/task.h"
#include "INS_Porting.hpp"
#include "INS_Driver.hpp"
#include "MotionController_Driver.hpp"
#include "Depth_Sensor_Driver.hpp"
#include "Pushrod_Driver.hpp"
#include "MS5837_LogConfig.hpp"
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
  ctx_->logger->init();
  memset(ins_rx_buffer, 0, sizeof(ins_rx_buffer));

  ctx_->ins_driver->init();
  // NORMAL 只负责接收并缓存 USBL 数据，不改变当前 INS/深度数据源选择。
  ctx_->usbl_driver->init();
  ctx_->chassis->applyConfig(auv::config::sys_config.chassis);

  /* 创建 SITL 导航数据队列（长度 3，生产者 onSimNav → 消费者 updateNavigation）
   */
  if (auv::motion::motion_context.sitl_nav_queue == nullptr) {
    auv::motion::motion_context.sitl_nav_queue =
        xQueueCreate(3, sizeof(auv::motion::NavState));
  }

  /* USBL topic FIFO：保存尚未发送到 micro-ROS 的有效帧。 */
  if (auv::motion::motion_context.usbl_topic_queue == nullptr) {
    auv::motion::motion_context.usbl_topic_queue =
        xQueueCreate(8, sizeof(auv::motion::UsblTopicSample));
  }

  /* 深度传感器初始化（Init 内部注册回调 + 初始化，start 创建任务/启动 DMA） */
  ctx_->depth_sensor->Init();
  ctx_->depth_sensor->start();

#if AUV_PRESET_USES_GPIO_PUSHROD
  // The GPIO backend owns its PB7/PB8 lifecycle. The self-UART backend is
  // intentionally not initialized here a second time because the depth
  // driver owns the shared UART4 DMA lifecycle.
  ctx_->pushrod_driver->Init();
  ctx_->pushrod_driver->start();
#endif

  ROS_LOG_INFO("System ControlTask initialized");

  last_wake_time_ = xTaskGetTickCount();
  last_tick_ = HAL_GetTick();
  ins_data_monitor_.reset(last_tick_);
  depth_data_monitor_.reset(last_tick_);
  auv::system::system_context.sensor_reception_status_.set({});
}

void ControlTask::updateNavigation() {
  auv::motion::NavState state;

  bool simulated = auv::config::sys_config.simulation.sitl_enabled;
#if AUV_SIMULATION_ENABLE
  simulated = simulated || auv::config::sys_config.simulation.hitl_enabled;
#endif
  if (simulated) {
    // Simulated navigation deliberately bypasses physical sensor reception.
    const uint32_t now_ms = HAL_GetTick();
    ins_data_monitor_.reset(now_ms);
    depth_data_monitor_.reset(now_ms);
    auv::system::system_context.sensor_reception_status_.set({});
  }

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
    const bool ins_frame_ready = ctx_->ins_driver->update(state);
    if (ctx_->ins_driver->serviceRxRecovery(
            auv::config::sys_config.system.soft_watchdog.timeout_ms)) {
      ROS_LOG_WARN("INS UART1 RX DMA rearmed after receive fault");
    }
    // 轮询 DMA 环形缓冲并解包最新 USBL 帧。数据暂不参与融合，供后续
    // 数据源选择/融合模块通过 AppContext::usbl_driver 读取。
    if (ctx_->usbl_driver->update(usbl_state_)) {
      auv::motion::UsblTopicSample sample{};
      sample.state = usbl_state_;
      auv::peripheral::UsblPortDiagnostics diagnostics{};
      ctx_->usbl_driver->getDiagnostics(diagnostics);
      sample.frame_number = diagnostics.valid_frames;

      auto queue = auv::motion::motion_context.usbl_topic_queue;
      if (queue != nullptr && xQueueSend(queue, &sample, 0) != pdPASS) {
        // 队列满时丢弃最旧帧，保证 topic 最终拿到最新数据。
        auv::motion::UsblTopicSample discarded{};
        (void)xQueueReceive(queue, &discarded, 0);
        (void)xQueueSend(queue, &sample, 0);
      }
    }

    // Keep polling the selected depth backend so runtime source switches can
    // use its latest sample. Only USE_MS5837_Z overrides the INS navigation Z;
    // the hardware preset determines the actual external sensor (e.g. M14).
    const int depth_frame_ready = ctx_->depth_sensor->Read();
    const float depth_z = ctx_->depth_sensor->getMS5837Z();
    if (auv::config::sys_config.system.sensors.z_data_source ==
        auv::config::ZDataSource::USE_MS5837_Z) {
      state.pos_world[2] = depth_z;
    }

    const uint32_t sensor_now_ms = HAL_GetTick();
    const bool depth_alarm_logged = monitorSensorReception(
        sensor_now_ms, ins_frame_ready, depth_frame_ready, depth_z);
#if MS5837_MAIN_DIAG_ENABLE
    static uint32_t last_depth_diag_ms = 0U;
    if (sensor_now_ms - last_depth_diag_ms >= 1000U) {
      last_depth_diag_ms = sensor_now_ms;
      if (!depth_alarm_logged) {
        logDepthDiagnostics(depth_frame_ready, depth_z);
      }
    }
#else
    (void)depth_alarm_logged;
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

bool ControlTask::monitorSensorReception(uint32_t now_ms,
                                        bool ins_frame_ready,
                                        int depth_frame_ready, float z) {
  const uint32_t timeout_ms =
      auv::config::sys_config.system.soft_watchdog.timeout_ms;
  const auto ins = ins_data_monitor_.update(now_ms, ins_frame_ready, timeout_ms);
  const auto depth = depth_data_monitor_.update(
      now_ms, depth_frame_ready != 0, timeout_ms);
  auv::system::system_context.sensor_reception_status_.set(
      {ins.timed_out, depth.timed_out});

  if (ins.alarm_due) {
    ROS_LOG_ERROR("INS data timeout: no valid frame for %lu ms (limit=%lu ms)",
                  (unsigned long)ins.age_ms, (unsigned long)timeout_ms);
    logInsDiagnostics();
  } else if (ins.recovered) {
    ROS_LOG_INFO("INS data reception recovered");
  }
  if (depth.alarm_due) {
    ROS_LOG_ERROR("Depth data timeout: no valid frame for %lu ms (limit=%lu ms)",
                  (unsigned long)depth.age_ms, (unsigned long)timeout_ms);
    logDepthDiagnostics(depth_frame_ready, z);
  } else if (depth.recovered) {
    ROS_LOG_INFO("Depth data reception recovered");
  }
  if (ctx_->depth_sensor->serviceRxRecovery(depth.timed_out)) {
    ROS_LOG_WARN("Depth UART4 RX DMA rearmed after receive fault");
  }
  return depth.alarm_due;
}

void ControlTask::logInsDiagnostics() {
  auv::peripheral::InsPortDiagnostics diagnostics{};
  ctx_->ins_driver->getDiagnostics(diagnostics);
  ROS_LOG_WARN("INS main: bytes=%lu valid=%lu invalid=%lu reads=%lu",
               (unsigned long)diagnostics.total_bytes,
               (unsigned long)diagnostics.valid_frames,
               (unsigned long)diagnostics.invalid_frames,
               (unsigned long)diagnostics.read_events);
  ROS_LOG_WARN("INS UART1: pos=%u ndtr=%u dma=%d isr=0x%lx raw=%02X%02X%02X%02X",
               (unsigned int)diagnostics.write_pos,
               (unsigned int)diagnostics.dma_remaining,
               diagnostics.dma_enabled ? 1 : 0,
               (unsigned long)diagnostics.uart_isr,
               (unsigned int)diagnostics.rx_preview[0],
               (unsigned int)diagnostics.rx_preview[1],
               (unsigned int)diagnostics.rx_preview[2],
               (unsigned int)diagnostics.rx_preview[3]);
  ROS_LOG_WARN(
      "INS RX recovery: err=%lu last=0x%lx start=%ld attempts=%lu ok=%lu "
      "fail=%lu pending=%d abort=%ld ddeinit=%ld dinit=%ld rx=%ld "
      "dma_state=%lu dma_err=0x%lx uart_rx=%lu",
      (unsigned long)diagnostics.uart_error_count,
      (unsigned long)diagnostics.last_uart_error,
      (long)diagnostics.rx_start_status,
      (unsigned long)diagnostics.rx_recovery_attempts,
      (unsigned long)diagnostics.rx_recovery_successes,
      (unsigned long)diagnostics.rx_recovery_failures,
      diagnostics.rx_recovery_pending ? 1 : 0,
      (long)diagnostics.rx_abort_status,
      (long)diagnostics.rx_dma_deinit_status,
      (long)diagnostics.rx_dma_init_status,
      (long)diagnostics.rx_hal_start_status,
      (unsigned long)diagnostics.dma_hal_state,
      (unsigned long)diagnostics.dma_hal_error,
      (unsigned long)diagnostics.uart_hal_rx_state);
}

void ControlTask::logDepthDiagnostics(int frame_ready, float z) {
  auv::peripheral::DepthDiagnostics diagnostics{};
  ctx_->depth_sensor->getDiagnostics(diagnostics);
  const long z_milli = (long)(z * 1000.0f + (z >= 0.0f ? 0.5f : -0.5f));
  const long z_abs_milli = z_milli < 0L ? -z_milli : z_milli;
  const char *z_sign = z_milli < 0L ? "-" : "";
  char rx_preview_hex[33] = {};
  static constexpr char kHex[] = "0123456789ABCDEF";
  for (uint8_t i = 0U; i < diagnostics.rx_preview_count; ++i) {
    rx_preview_hex[i * 2U] =
        kHex[(diagnostics.rx_preview[i] >> 4U) & 0x0FU];
    rx_preview_hex[i * 2U + 1U] = kHex[diagnostics.rx_preview[i] & 0x0FU];
  }
  ROS_LOG_WARN(
      "Depth main: frame=%d z=%s%ld.%03ld connected=%d ack=%d "
      "bytes=%lu valid=%lu data=%lu rx=%lu pos=%lu err=%lu",
      frame_ready, z_sign, z_abs_milli / 1000L, z_abs_milli % 1000L,
      diagnostics.connected ? 1 : 0,
      diagnostics.handshake_acknowledged ? 1 : 0,
      (unsigned long)diagnostics.rx_byte_count,
      (unsigned long)diagnostics.valid_frame_count,
      (unsigned long)diagnostics.data_frame_count,
      (unsigned long)diagnostics.rx_event_count,
      (unsigned long)diagnostics.dma_write_pos,
      (unsigned long)diagnostics.rx_error_count);
  ROS_LOG_WARN(
      "Depth proto: parse=%lu hs=%lu push=%lu nr=%lu last=%02x/%u raw=%s",
      (unsigned long)diagnostics.parser_error_count,
      (unsigned long)diagnostics.handshake_ack_count,
      (unsigned long)diagnostics.pushrod_ack_count,
      (unsigned long)diagnostics.sensor_not_ready_count,
      (unsigned int)diagnostics.last_frame_type,
      (unsigned int)diagnostics.last_frame_length, rx_preview_hex);
  if (diagnostics.has_uart_diagnostics) {
    ROS_LOG_WARN(
        "Depth UART4: start=%ld dma=%d last_err=0x%lx",
        (long)diagnostics.rx_start_status,
        diagnostics.rx_dma_active ? 1 : 0,
        (unsigned long)diagnostics.last_rx_error);
    ROS_LOG_WARN(
        "Depth RX recovery: err=%lu attempts=%lu ok=%lu fail=%lu "
        "pending=%d reason=%lu abort=%ld ddeinit=%ld dinit=%ld rx=%ld "
        "dma_state=%lu dma_err=0x%lx uart_rx=%lu",
        (unsigned long)diagnostics.rx_error_count,
        (unsigned long)diagnostics.rx_recovery_attempts,
        (unsigned long)diagnostics.rx_recovery_successes,
        (unsigned long)diagnostics.rx_recovery_failures,
        diagnostics.rx_recovery_pending ? 1 : 0,
        (unsigned long)diagnostics.last_rx_recovery_reason,
        (long)diagnostics.rx_abort_status,
        (long)diagnostics.rx_dma_deinit_status,
        (long)diagnostics.rx_dma_init_status,
        (long)diagnostics.rx_hal_start_status,
        (unsigned long)diagnostics.dma_hal_state,
        (unsigned long)diagnostics.dma_hal_error,
        (unsigned long)diagnostics.uart_hal_rx_state);
  }
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
