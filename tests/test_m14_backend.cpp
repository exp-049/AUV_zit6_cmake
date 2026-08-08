#include "M14_UART_Backend.hpp"

#include <gtest/gtest.h>

#include <cstring>
#include <string>

namespace {

struct FakeTransport {
  std::string transmitted;
  bool started = false;

  static bool transmit(void *ctx, const uint8_t *data, uint16_t length) {
    auto *self = static_cast<FakeTransport *>(ctx);
    self->transmitted.assign(reinterpret_cast<const char *>(data), length);
    return true;
  }

  static void poll(void *) {}

  static bool startRx(void *ctx) {
    static_cast<FakeTransport *>(ctx)->started = true;
    return true;
  }
};

struct Sample {
  float depth = 0.0f;
  float temperature = 0.0f;
  int count = 0;
};

void onDepth(void *ctx, float depth, float temperature) {
  auto *sample = static_cast<Sample *>(ctx);
  sample->depth = depth;
  sample->temperature = temperature;
  ++sample->count;
}

auv::peripheral::UartPortOps makeOps(FakeTransport *transport) {
  return auv::peripheral::UartPortOps{
      .ctx = transport,
      .transmit = &FakeTransport::transmit,
      .poll = &FakeTransport::poll,
      .startRx = &FakeTransport::startRx,
  };
}

} // namespace

TEST(M14BackendTest, ParsesTemperatureAndDepthLine) {
  FakeTransport transport;
  auv::peripheral::M14_UART_Backend backend(makeOps(&transport));
  Sample sample;
  backend.setCallback({&onDepth, &sample});

  ASSERT_TRUE(backend.init());
  backend.start();
  EXPECT_TRUE(transport.started);

  const char frame[] = "T=12.34D=-0.55\r\n";
  for (char byte : frame) {
    if (byte != '\0') {
      backend.onRxByte(static_cast<uint8_t>(byte));
    }
  }

  EXPECT_TRUE(backend.isConnected());
  ASSERT_TRUE(backend.read());
  EXPECT_FLOAT_EQ(sample.temperature, 12.34f);
  EXPECT_FLOAT_EQ(sample.depth, -0.55f);
  EXPECT_EQ(sample.count, 1);
}

TEST(M14BackendTest, ForwardsDepthThroughDepthSensorDriver) {
  FakeTransport transport;
  auv::peripheral::M14_UART_Backend backend(makeOps(&transport));
  auv::peripheral::Depth_Sensor_Driver driver(&backend);

  driver.Init();
  driver.start();
  const char frame[] = "T=18.25D=3.75\r\n";
  for (char byte : frame) {
    if (byte != '\0') {
      backend.onRxByte(static_cast<uint8_t>(byte));
    }
  }

  ASSERT_EQ(driver.Read(), 1);
  EXPECT_FLOAT_EQ(driver.getMS5837Z(), 3.75f);
}

TEST(M14BackendTest, SendsM14CommandsWithCrLf) {
  FakeTransport transport;
  auv::peripheral::M14_UART_Backend backend(makeOps(&transport));
  ASSERT_TRUE(backend.init());

  ASSERT_TRUE(backend.setDepthOffset(1.25f));
  EXPECT_EQ(transport.transmitted, "!D1.25\r\n");
  ASSERT_TRUE(backend.setTemperatureOffset(-0.50f));
  EXPECT_EQ(transport.transmitted, "!T-0.50\r\n");
  ASSERT_TRUE(backend.setFluidDensity(1025U));
  EXPECT_EQ(transport.transmitted, "!F1025\r\n");
  ASSERT_TRUE(backend.toggleParameterOutput());
  EXPECT_EQ(transport.transmitted, "!!\r\n");
}
