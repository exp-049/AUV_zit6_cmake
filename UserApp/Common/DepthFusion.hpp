#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace auv::motion {

/** Inputs and diagnostics for the scalar INS/M14 depth complementary filter. */
struct DepthFusionInput {
  uint32_t now_ms = 0U;
  uint32_t source_timeout_ms = 1000U;

  bool have_ins_frame = false;
  bool ins_frame_received = false;
  uint32_t ins_frame_ms = 0U;
  float ins_pressure_z = 0.0f;

  bool have_m14_sample = false;
  uint32_t m14_sample_ms = 0U;
  float m14_z = 0.0f;

  float roll_rad = 0.0f;
  float pitch_rad = 0.0f;
  float u = 0.0f;
  float v = 0.0f;
  float w = 0.0f;
};

struct DepthFusionOutput {
  float z = 0.0f;
  float bias_m = 0.0f;
  uint32_t pressure_age_ms = 0U;
  bool initialized = false;
  bool ins_live = false;
  bool m14_live = false;
  bool pressure_usable = false;
  bool pressure_stale_suspect = false;
  bool absolute_depth_available = false;
  bool any_sensor_live = false;
  bool bias_calibrated = false;
};

/**
 * Fuses the INS pressure depth and M14 depth around one scalar z-axis bias.
 * z is positive down in the NED navigation frame. The INS pressure port is
 * the reference; bias = z_INS_pressure - z_M14.
 */
class DepthFusion {
public:
  static constexpr float kPressureChangeEpsilonM = 0.01f;
  static constexpr uint32_t kPressureAgingMs = 1000U;
  static constexpr float kPressureMotionStaleM = 0.10f;
  static constexpr uint32_t kPairMaxSkewMs = 1000U;
  static constexpr float kBiasTimeConstantS = 30.0f;
  static constexpr float kBiasMaxRateMps = 0.01f;
  static constexpr float kBiasConvergedErrorM = 0.02f;
  static constexpr uint32_t kBiasConvergedHoldMs = 1000U;
  static constexpr float kAbsoluteCorrectionTimeConstantS = 0.5f;

  DepthFusionOutput update(const DepthFusionInput &input) {
    float dt_s = 0.0f;
    if (have_tick_) {
      const uint32_t dt_ms = input.now_ms - last_tick_ms_;
      // A delayed control iteration must not create a large one-step jump.
      dt_s = static_cast<float>(std::min<uint32_t>(dt_ms, 100U)) * 0.001f;
    }
    last_tick_ms_ = input.now_ms;
    have_tick_ = true;

    const bool finite_pressure = std::isfinite(input.ins_pressure_z);
    bool pressure_changed = false;
    if (input.ins_frame_received && finite_pressure) {
      if (!have_pressure_) {
        pressure_changed = true;
      } else if (std::fabs(input.ins_pressure_z - last_pressure_z_) >=
                 kPressureChangeEpsilonM) {
        pressure_changed = true;
      }
      if (pressure_changed) {
        last_pressure_change_ms_ = input.now_ms;
        pressure_motion_m_ = 0.0f;
        pressure_stale_suspect_ = false;
      }
      last_pressure_z_ = input.ins_pressure_z;
      have_pressure_ = true;
    }

    const bool ins_live = input.have_ins_frame &&
        (input.now_ms - input.ins_frame_ms <= input.source_timeout_ms);
    const bool m14_live = input.have_m14_sample &&
        (input.now_ms - input.m14_sample_ms <= input.source_timeout_ms) &&
        std::isfinite(input.m14_z);
    const bool velocity_finite = std::isfinite(input.roll_rad) &&
        std::isfinite(input.pitch_rad) && std::isfinite(input.u) &&
        std::isfinite(input.v) && std::isfinite(input.w);
    const bool velocity_live = ins_live && velocity_finite;
    const float v_down = velocity_live
        ? -input.u * std::sin(input.pitch_rad) +
              input.v * std::sin(input.roll_rad) * std::cos(input.pitch_rad) +
              input.w * std::cos(input.roll_rad) * std::cos(input.pitch_rad)
        : 0.0f;

    if (!pressure_changed && velocity_live && have_pressure_) {
      pressure_motion_m_ += v_down * dt_s;
    }

    const uint32_t pressure_age_ms = have_pressure_
        ? input.now_ms - last_pressure_change_ms_
        : UINT32_MAX;
    if (have_pressure_ && ins_live && pressure_age_ms >= kPressureAgingMs &&
        std::fabs(pressure_motion_m_) >= kPressureMotionStaleM) {
      pressure_stale_suspect_ = true;
    }
    const bool pressure_usable = have_pressure_ && finite_pressure && ins_live &&
                                 !pressure_stale_suspect_;
    const float pressure_weight = pressure_age_ms <= kPressureAgingMs
                                      ? 1.0f
                                      : 0.25f;

    if (initialized_ && velocity_live) {
      z_ += v_down * dt_s;
    }

    const bool pair_for_bias = pressure_usable && m14_live &&
        pressure_age_ms <= kPressureAgingMs &&
        absTimeDifference(last_pressure_change_ms_, input.m14_sample_ms) <=
            kPairMaxSkewMs;
    if (pair_for_bias) {
      if (!bias_initialized_ && !initialized_) {
        const float skew_s =
            signedTimeDifference(last_pressure_change_ms_, input.m14_sample_ms) *
            0.001f;
        const float m14_at_pressure_time = input.m14_z + v_down * skew_s;
        bias_m_ = input.ins_pressure_z - m14_at_pressure_time;
        bias_initialized_ = true;
        bias_calibrated_ = true;
      } else {
        bias_initialized_ = true;
        const float skew_s =
            signedTimeDifference(last_pressure_change_ms_, input.m14_sample_ms) *
            0.001f;
        const float m14_at_pressure_time = input.m14_z + v_down * skew_s;
        const float measured_bias = input.ins_pressure_z - m14_at_pressure_time;
        const float low_pass_step =
            (measured_bias - bias_m_) *
            (dt_s / (kBiasTimeConstantS + dt_s));
        const float max_step = kBiasMaxRateMps * dt_s;
        bias_m_ += std::clamp(low_pass_step, -max_step, max_step);
        if (std::fabs(measured_bias - bias_m_) <= kBiasConvergedErrorM) {
          bias_converged_ms_ += static_cast<uint32_t>(dt_s * 1000.0f);
          if (bias_converged_ms_ >= kBiasConvergedHoldMs) {
            bias_calibrated_ = true;
          }
        } else {
          bias_converged_ms_ = 0U;
        }
      }
    }

    const bool absolute_available = pressure_usable || m14_live;
    if (absolute_available) {
      float weighted_z = 0.0f;
      float total_weight = 0.0f;
      const float pressure_at_now = input.ins_pressure_z +
          v_down * static_cast<float>(input.now_ms - last_pressure_change_ms_) *
              0.001f;
      const float m14_at_now = input.m14_z +
          v_down * static_cast<float>(input.now_ms - input.m14_sample_ms) *
              0.001f;
      bool use_pressure = pressure_usable;
      bool use_m14 = m14_live;
      // If the filter started from one source, keep that source as its anchor
      // while b converges. This prevents an uncalibrated second source from
      // pulling z toward its own datum.
      if (initialized_ && !bias_calibrated_) {
        if (anchor_ == Anchor::InsPressure) {
          use_m14 = false;
        } else if (anchor_ == Anchor::M14) {
          use_pressure = false;
        }
      } else if (!initialized_ && !bias_calibrated_ && use_pressure && use_m14) {
        // If timestamps cannot be paired yet, initialize in the INS datum and
        // let M14 join after its scalar offset has been learned.
        use_m14 = false;
      }
      if (use_pressure) {
        weighted_z += pressure_at_now * pressure_weight;
        total_weight += pressure_weight;
      }
      if (use_m14) {
        weighted_z += (m14_at_now + bias_m_);
        total_weight += 1.0f;
      }
      if (total_weight <= 0.0f) {
        return {z_, bias_m_, pressure_age_ms, initialized_, ins_live, m14_live,
                pressure_usable, pressure_stale_suspect_, absolute_available,
                ins_live || m14_live, bias_calibrated_};
      }
      const float absolute_z = weighted_z / total_weight;
      if (!initialized_) {
        z_ = absolute_z;
        initialized_ = true;
        if (pressure_usable && !m14_live) {
          anchor_ = Anchor::InsPressure;
        } else if (pressure_usable && m14_live && !bias_calibrated_) {
          anchor_ = Anchor::InsPressure;
        } else if (m14_live && !pressure_usable) {
          anchor_ = Anchor::M14;
        }
      } else if (dt_s > 0.0f) {
        const float alpha = dt_s /
            (kAbsoluteCorrectionTimeConstantS + dt_s);
        z_ += alpha * (absolute_z - z_);
      }
    }

    return {z_, bias_m_, pressure_age_ms, initialized_, ins_live, m14_live,
            pressure_usable, pressure_stale_suspect_, absolute_available,
            ins_live || m14_live, bias_calibrated_};
  }

  void reset() { *this = DepthFusion{}; }

private:
  enum class Anchor : uint8_t { None, InsPressure, M14 };

  static uint32_t absTimeDifference(uint32_t a, uint32_t b) {
    const int32_t difference = static_cast<int32_t>(a - b);
    return static_cast<uint32_t>(difference < 0 ? -difference : difference);
  }

  static int32_t signedTimeDifference(uint32_t a, uint32_t b) {
    return static_cast<int32_t>(a - b);
  }

  uint32_t last_tick_ms_ = 0U;
  uint32_t last_pressure_change_ms_ = 0U;
  float last_pressure_z_ = 0.0f;
  float pressure_motion_m_ = 0.0f;
  float bias_m_ = 0.0f;
  float z_ = 0.0f;
  uint32_t bias_converged_ms_ = 0U;
  bool have_tick_ = false;
  bool have_pressure_ = false;
  bool pressure_stale_suspect_ = false;
  bool bias_initialized_ = false;
  bool bias_calibrated_ = false;
  bool initialized_ = false;
  Anchor anchor_ = Anchor::None;
};

} // namespace auv::motion
