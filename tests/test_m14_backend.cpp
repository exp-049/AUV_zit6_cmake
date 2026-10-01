#include "M14_UART_Backend.hpp"

#include <gtest/gtest.h>

#include <cstring>
#include <string>

namespace {

struct FakeTransport {
  std::string transmitted;
  bool started = false;
  bool recovery_result = false;
  bool last_no_frame_timeout = false;
  uint32_t recovery_calls = 0U;
  auv::peripheral::DepthDiagnostics diagnostics{};

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

  static void getDiagnostics(void *ctx,
                             auv::peripheral::DepthDiagnostics &out) {
    out = static_cast<FakeTransport *>(ctx)->diagnostics;
  }

  static bool serviceRxRecovery(void *ctx, bool no_valid_frame_timeout) {
    auto *self = static_cast<FakeTransport *>(ctx);
    ++self->recovery_calls;
    self->last_no_frame_timeout = no_valid_frame_timeout;
    return self->recovery_result;
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
      .getDiagnostics = &FakeTransport::getDiagnostics,
      .serviceRxRecovery = &FakeTransport::serviceRxRecovery,
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

TEST(M14BackendTest, DelegatesRecoveryAndDropsPartialLineAfterRearm) {
  FakeTransport transport;
  transport.recovery_result = true;
  auv::peripheral::M14_UART_Backend backend(makeOps(&transport));
  auv::peripheral::Depth_Sensor_Driver driver(&backend);
  driver.Init();
  driver.start();

  const std::string partial = "T=18.25D=";
  for (unsigned char byte : partial) {
    backend.onRxByte(byte);
  }
  EXPECT_TRUE(driver.serviceRxRecovery(true));
  EXPECT_EQ(transport.recovery_calls, 1U);
  EXPECT_TRUE(transport.last_no_frame_timeout);

  // The suffix must not be joined to a pre-recovery partial line.
  for (unsigned char byte : std::string("3.75\r\n")) {
    backend.onRxByte(byte);
  }
  EXPECT_FALSE(backend.read());
  for (unsigned char byte : std::string("T=18.25D=3.75\r\n")) {
    backend.onRxByte(byte);
  }
  ASSERT_TRUE(backend.read());
  EXPECT_FLOAT_EQ(backend.getDepth(), 3.75f);
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

TEST(M14BackendTest, ReportsBytesBeforeACompleteFrameArrives) {
  FakeTransport transport;
  auv::peripheral::M14_UART_Backend backend(makeOps(&transport));
  ASSERT_TRUE(backend.init());
  const std::string partial = "T=18.25D=";
  for (unsigned char byte : partial) {
    backend.onRxByte(byte);
  }

  auv::peripheral::DepthDiagnostics diagnostics;
  backend.getDiagnostics(diagnostics);
  EXPECT_EQ(diagnostics.rx_byte_count, partial.size());
  EXPECT_EQ(diagnostics.valid_frame_count, 0U);
  EXPECT_EQ(diagnostics.parser_error_count, 0U);
  EXPECT_FALSE(diagnostics.connected);
  EXPECT_FALSE(backend.read());
  ASSERT_EQ(diagnostics.rx_preview_count, partial.size());
  EXPECT_EQ(std::string(reinterpret_cast<char *>(diagnostics.rx_preview),
                        diagnostics.rx_preview_count), partial);
}

TEST(M14BackendTest, ReportsParserErrorsAndHardwareStateThroughFacade) {
  FakeTransport transport;
  transport.diagnostics.has_uart_diagnostics = true;
  transport.diagnostics.rx_dma_active = true;
  transport.diagnostics.rx_start_status = 0;
  transport.diagnostics.dma_write_pos = 42U;
  transport.diagnostics.rx_error_count = 2U;
  transport.diagnostics.last_rx_error = 8U;
  auv::peripheral::M14_UART_Backend backend(makeOps(&transport));
  auv::peripheral::Depth_Sensor_Driver driver(&backend);
  driver.Init();
  driver.start();
  const std::string input = "garbage\r\nT = 18.25 D = 3.75\r\n";
  for (unsigned char byte : input) {
    backend.onRxByte(byte);
  }
  ASSERT_EQ(driver.Read(), 1);
  EXPECT_FLOAT_EQ(driver.getMS5837Z(), 3.75f);

  auv::peripheral::DepthDiagnostics diagnostics;
  driver.getDiagnostics(diagnostics);
  EXPECT_TRUE(diagnostics.connected);
  EXPECT_EQ(diagnostics.rx_byte_count, input.size());
  EXPECT_EQ(diagnostics.valid_frame_count, 1U);
  EXPECT_EQ(diagnostics.data_frame_count, 1U);
  EXPECT_EQ(diagnostics.parser_error_count, 1U);
  EXPECT_TRUE(diagnostics.has_uart_diagnostics);
  EXPECT_TRUE(diagnostics.rx_dma_active);
  EXPECT_EQ(diagnostics.rx_start_status, 0);
  EXPECT_EQ(diagnostics.dma_write_pos, 42U);
  EXPECT_EQ(diagnostics.rx_error_count, 2U);
  EXPECT_EQ(diagnostics.last_rx_error, 8U);
  EXPECT_FALSE(diagnostics.handshake_acknowledged);
  ASSERT_EQ(diagnostics.rx_preview_count, 16U);
  EXPECT_EQ(std::string(reinterpret_cast<char *>(diagnostics.rx_preview), 16),
            input.substr(input.size() - 16));
}

TEST(M14BackendTest, RejectsAnOverflowedLineAndRecoversAtNextLine) {
  FakeTransport transport;
  auv::peripheral::M14_UART_Backend backend(makeOps(&transport));
  ASSERT_TRUE(backend.init());
  const std::string overflow = std::string(128, 'x') + "T=1D=2\r\n";
  for (unsigned char byte : overflow) {
    backend.onRxByte(byte);
  }
  EXPECT_FALSE(backend.read());
  EXPECT_FALSE(backend.isConnected());
  const std::string frame = "T=5D=6\r\n";
  for (unsigned char byte : frame) {
    backend.onRxByte(byte);
  }
  ASSERT_TRUE(backend.read());
  EXPECT_FLOAT_EQ(backend.getDepth(), 6.0f);
  auv::peripheral::DepthDiagnostics diagnostics;
  backend.getDiagnostics(diagnostics);
  EXPECT_EQ(diagnostics.parser_error_count, 1U);
  EXPECT_EQ(diagnostics.valid_frame_count, 1U);
}
