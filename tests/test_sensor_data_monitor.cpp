#include "SensorDataMonitor.hpp"

#include <gtest/gtest.h>
#include <limits>

using auv::component::SensorDataMonitor;

TEST(SensorDataMonitorTest, AllowsStartupGraceThenReportsNoData) {
  SensorDataMonitor monitor;
  monitor.reset(100U);
  EXPECT_FALSE(monitor.update(3099U, false, 3000U).timed_out);
  const auto alarm = monitor.update(3100U, false, 3000U);
  EXPECT_TRUE(alarm.timed_out);
  EXPECT_TRUE(alarm.alarm_due);
  EXPECT_EQ(alarm.age_ms, 3000U);
  EXPECT_FALSE(alarm.recovered);
}

TEST(SensorDataMonitorTest, TimesOutWhenPreviouslyWorkingStreamStops) {
  SensorDataMonitor monitor;
  monitor.reset(0U);
  EXPECT_FALSE(monitor.update(1000U, true, 3000U).timed_out);
  EXPECT_FALSE(monitor.update(3999U, false, 3000U).timed_out);
  const auto alarm = monitor.update(4000U, false, 3000U);
  EXPECT_TRUE(alarm.alarm_due);
  EXPECT_EQ(alarm.age_ms, 3000U);
}

TEST(SensorDataMonitorTest, LimitsRepeatedAlarmsAndReportsRecoveryOnce) {
  SensorDataMonitor monitor;
  monitor.reset(0U);
  EXPECT_TRUE(monitor.update(3000U, false, 3000U).alarm_due);
  EXPECT_FALSE(monitor.update(3010U, false, 3000U).alarm_due);
  EXPECT_FALSE(monitor.update(12999U, false, 3000U).alarm_due);
  EXPECT_TRUE(monitor.update(13000U, false, 3000U).alarm_due);
  const auto recovered = monitor.update(13100U, true, 3000U);
  EXPECT_TRUE(recovered.recovered);
  EXPECT_FALSE(recovered.timed_out);
  EXPECT_FALSE(recovered.alarm_due);
  EXPECT_FALSE(monitor.update(13110U, true, 3000U).recovered);
  EXPECT_TRUE(monitor.update(16110U, false, 3000U).alarm_due);
}

TEST(SensorDataMonitorTest, ValidFramesPreventAlarmsAtTimeoutBoundary) {
  SensorDataMonitor monitor;
  monitor.reset(0U);
  const auto sample = monitor.update(3000U, true, 3000U);
  EXPECT_FALSE(sample.timed_out);
  EXPECT_FALSE(sample.alarm_due);
  EXPECT_EQ(sample.age_ms, 0U);
}

TEST(SensorDataMonitorTest, TracksSensorsIndependently) {
  SensorDataMonitor ins;
  SensorDataMonitor depth;
  ins.reset(0U);
  depth.reset(0U);
  EXPECT_FALSE(ins.update(3000U, true, 3000U).timed_out);
  EXPECT_TRUE(depth.update(3000U, false, 3000U).timed_out);
  EXPECT_FALSE(ins.update(3100U, true, 3000U).recovered);
  EXPECT_TRUE(depth.update(3100U, true, 3000U).recovered);
}

TEST(SensorDataMonitorTest, ResetClearsAlarmForSimulationAndRestart) {
  SensorDataMonitor monitor;
  monitor.reset(0U);
  EXPECT_TRUE(monitor.update(3000U, false, 3000U).timed_out);
  monitor.reset(4000U);
  const auto resumed = monitor.update(4010U, false, 3000U);
  EXPECT_FALSE(resumed.timed_out);
  EXPECT_FALSE(resumed.alarm_due);
  EXPECT_FALSE(resumed.recovered);
}

TEST(SensorDataMonitorTest, HandlesMillisecondCounterWraparound) {
  SensorDataMonitor monitor;
  const uint32_t start = std::numeric_limits<uint32_t>::max() - 1000U;
  monitor.reset(start);
  EXPECT_FALSE(monitor.update(start + 2999U, false, 3000U).timed_out);
  const auto alarm = monitor.update(start + 3000U, false, 3000U);
  EXPECT_TRUE(alarm.alarm_due);
  EXPECT_EQ(alarm.age_ms, 3000U);
  EXPECT_FALSE(monitor.update(start + 12999U, false, 3000U).alarm_due);
  EXPECT_TRUE(monitor.update(start + 13000U, false, 3000U).alarm_due);
}
