#include "Pushrod_GPIO_Backend.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace {

struct FakeGpio {
  struct Output {
    bool in1;
    bool in2;
  };

  bool init_ok = true;
  uint32_t now_ms = 0U;
  std::vector<Output> outputs;
};

bool initPort(void *ctx) {
  return static_cast<FakeGpio *>(ctx)->init_ok;
}

bool setOutputs(void *ctx, bool in1, bool in2) {
  static_cast<FakeGpio *>(ctx)->outputs.push_back({in1, in2});
  return true;
}

uint32_t getTick(void *ctx) { return static_cast<FakeGpio *>(ctx)->now_ms; }

auv::peripheral::Pushrod_GPIO_Backend makeBackend(FakeGpio &gpio) {
  return auv::peripheral::Pushrod_GPIO_Backend(
      auv::peripheral::PushrodGpioPortOps{
          .ctx = &gpio,
          .init = &initPort,
          .setOutputs = &setOutputs,
          .getTickMs = &getTick,
      });
}

} // namespace

TEST(PushrodGpioBackendTest, MapsPositiveAndNegativePowerToDirectionPins) {
  FakeGpio gpio;
  auto backend = makeBackend(gpio);

  ASSERT_TRUE(backend.init());
  auv::peripheral::PushrodTask positive{1U, 500, 100U};
  ASSERT_TRUE(backend.sendTask(positive));
  EXPECT_TRUE(gpio.outputs.back().in1);
  EXPECT_FALSE(gpio.outputs.back().in2);

  auv::peripheral::PushrodAck ack{};
  EXPECT_FALSE(backend.readAck(&ack));

  backend.poll(99U);
  EXPECT_TRUE(backend.isActive());
  backend.poll(100U);
  EXPECT_FALSE(backend.isActive());
  EXPECT_FALSE(gpio.outputs.back().in1);
  EXPECT_FALSE(gpio.outputs.back().in2);
  ASSERT_TRUE(backend.readAck(&ack));
  EXPECT_EQ(ack.task_id, 1U);
  EXPECT_EQ(ack.result, auv::peripheral::pushrod::kOk);

  auv::peripheral::PushrodTask negative{2U, -1000, 100U};
  ASSERT_TRUE(backend.sendTask(negative));
  EXPECT_FALSE(gpio.outputs.back().in1);
  EXPECT_TRUE(gpio.outputs.back().in2);
}

TEST(PushrodGpioBackendTest, IgnoresMagnitudeAndTreatsZeroAsSafeStop) {
  FakeGpio gpio;
  auto backend = makeBackend(gpio);
  ASSERT_TRUE(backend.init());

  ASSERT_TRUE(backend.sendTask({3U, 0, 1000U}));
  EXPECT_FALSE(backend.isActive());
  EXPECT_FALSE(gpio.outputs.back().in1);
  EXPECT_FALSE(gpio.outputs.back().in2);

  auv::peripheral::PushrodAck ack{};
  EXPECT_TRUE(backend.readAck(&ack));
  EXPECT_EQ(ack.result, auv::peripheral::pushrod::kOk);
}

TEST(PushrodGpioBackendTest, StopDeenergizesAnActiveMotor) {
  FakeGpio gpio;
  auto backend = makeBackend(gpio);
  ASSERT_TRUE(backend.init());
  ASSERT_TRUE(backend.sendTask({4U, 1, 1000U}));

  backend.stop();
  EXPECT_FALSE(backend.isActive());
  EXPECT_FALSE(gpio.outputs.back().in1);
  EXPECT_FALSE(gpio.outputs.back().in2);
}
