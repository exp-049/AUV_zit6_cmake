#include "../UserApp/Common/DepthFusion.hpp"

#include <gtest/gtest.h>

namespace motion = auv::motion;

TEST(DepthFusion, PairedStartupLearnsOneDimensionalOffset) {
  motion::DepthFusion fusion;
  motion::DepthFusionInput input{};
  input.now_ms = 100U;
  input.source_timeout_ms = 1000U;
  input.have_ins_frame = true;
  input.ins_frame_received = true;
  input.ins_frame_ms = 100U;
  input.ins_pressure_z = 4.0f;
  input.have_m14_sample = true;
  input.m14_sample_ms = 100U;
  input.m14_z = 3.0f;

  const auto output = fusion.update(input);
  EXPECT_TRUE(output.initialized);
  EXPECT_TRUE(output.bias_calibrated);
  EXPECT_FLOAT_EQ(output.bias_m, 1.0f);
  EXPECT_FLOAT_EQ(output.z, 4.0f);
}

TEST(DepthFusion, AlignsHeldAbsoluteSamplesToCurrentTimeWithVelocity) {
  motion::DepthFusion fusion;
  motion::DepthFusionInput input{};
  input.source_timeout_ms = 1000U;
  input.have_ins_frame = true;
  input.ins_frame_received = true;
  input.ins_pressure_z = 1.0f;
  input.have_m14_sample = true;
  input.m14_z = 1.0f;
  input.w = 0.2f;
  ASSERT_TRUE(fusion.update(input).initialized);

  input.now_ms = 100U;
  input.ins_frame_received = false;
  input.ins_frame_ms = 0U;
  input.m14_sample_ms = 0U;
  const auto output = fusion.update(input);
  EXPECT_NEAR(output.z, 1.02f, 1e-4f);
}

TEST(DepthFusion, LearnsScalarBiasWithoutPullingUncalibratedSource) {
  motion::DepthFusion fusion;
  motion::DepthFusionInput input{};
  input.source_timeout_ms = 3000U;
  input.have_m14_sample = true;
  input.m14_sample_ms = 0U;
  input.m14_z = 3.0f;
  auto output = fusion.update(input);
  ASSERT_TRUE(output.initialized);
  EXPECT_FLOAT_EQ(output.z, 3.0f);

  for (uint32_t now = 10U; now <= 1010U; now += 10U) {
    input.now_ms = now;
    input.have_ins_frame = true;
    input.ins_frame_received = now == 10U;
    input.ins_frame_ms = 10U;
    input.ins_pressure_z = 4.0f;
    input.have_m14_sample = true;
    input.m14_sample_ms = now;
    output = fusion.update(input);
  }

  EXPECT_GT(output.bias_m, 0.008f);
  EXPECT_LE(output.bias_m, 0.0101f);
  EXPECT_LT(output.z - 3.0f, 0.02f);
  EXPECT_FALSE(output.bias_calibrated);
}

TEST(DepthFusion, RepeatedFramesDoNotRefreshPressureAgeAndMotionDetectsStale) {
  motion::DepthFusion fusion;
  motion::DepthFusionInput input{};
  input.source_timeout_ms = 5000U;
  input.have_ins_frame = true;
  input.ins_frame_received = true;
  input.ins_frame_ms = 0U;
  input.ins_pressure_z = 2.0f;
  input.w = 0.2f;
  auto output = fusion.update(input);
  ASSERT_TRUE(output.initialized);

  for (uint32_t now = 10U; now <= 1500U; now += 10U) {
    input.now_ms = now;
    input.ins_frame_received = true;
    input.ins_frame_ms = now;
    output = fusion.update(input);
  }

  EXPECT_EQ(output.pressure_age_ms, 1500U);
  EXPECT_TRUE(output.pressure_stale_suspect);
  EXPECT_FALSE(output.pressure_usable);
  EXPECT_TRUE(output.any_sensor_live);
  EXPECT_FALSE(output.absolute_depth_available);
  EXPECT_GT(output.z, 2.1f); // INS down velocity continues the prediction.
}

TEST(DepthFusion, UnchangedPressureAtRestRemainsUsableButIsDownweighted) {
  motion::DepthFusion fusion;
  motion::DepthFusionInput input{};
  input.source_timeout_ms = 5000U;
  input.have_ins_frame = true;
  input.ins_frame_received = true;
  input.ins_frame_ms = 0U;
  input.ins_pressure_z = 1.25f;
  input.have_m14_sample = true;
  input.m14_sample_ms = 0U;
  input.m14_z = 1.25f;
  auto output = fusion.update(input);

  for (uint32_t now = 10U; now <= 2000U; now += 10U) {
    input.now_ms = now;
    input.ins_frame_ms = now;
    input.ins_frame_received = true;
    output = fusion.update(input);
  }

  input.m14_z = 2.25f;
  for (uint32_t now = 2010U; now <= 4000U; now += 10U) {
    input.now_ms = now;
    input.ins_frame_ms = now;
    input.ins_frame_received = true;
    input.m14_sample_ms = now;
    output = fusion.update(input);
  }

  EXPECT_EQ(output.pressure_age_ms, 4000U);
  EXPECT_FALSE(output.pressure_stale_suspect);
  EXPECT_TRUE(output.pressure_usable);
  EXPECT_TRUE(output.absolute_depth_available);
  EXPECT_GT(output.z, 2.0f); // The fresh M14 observation outweighs held pressure.
}

TEST(DepthFusion, M14KeepsZAvailableWhenINSStreamIsLost) {
  motion::DepthFusion fusion;
  motion::DepthFusionInput input{};
  input.source_timeout_ms = 1000U;
  input.have_m14_sample = true;
  input.m14_sample_ms = 0U;
  input.m14_z = 2.0f;
  auto output = fusion.update(input);
  ASSERT_TRUE(output.initialized);

  input.now_ms = 100U;
  input.m14_sample_ms = 100U;
  input.m14_z = 2.5f;
  output = fusion.update(input);
  EXPECT_FALSE(output.ins_live);
  EXPECT_TRUE(output.m14_live);
  EXPECT_TRUE(output.any_sensor_live);
  EXPECT_TRUE(output.absolute_depth_available);
  EXPECT_GT(output.z, 2.0f);
  const float held_z = output.z;

  input.now_ms = 1200U;
  input.m14_sample_ms = 100U;
  output = fusion.update(input);
  EXPECT_FALSE(output.any_sensor_live);
  EXPECT_FALSE(output.absolute_depth_available);
  EXPECT_FLOAT_EQ(output.z, held_z);
}
