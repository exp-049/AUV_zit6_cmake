#include "Depth_Sensor_Driver.hpp"
#include "Pushrod_Driver.hpp"

#include <gtest/gtest.h>

namespace {

using auv::peripheral::PushrodAck;
using auv::peripheral::PushrodTask;

class FakeDepthBackend final : public auv::peripheral::DepthBackend {
public:
  bool init() override {
    initialized = true;
    return init_ok;
  }

  void poll() override { ++poll_count; }

  bool read() override {
    if (!ready) {
      return false;
    }
    ready = false;
    if (callback.onDepthReady != nullptr) {
      callback.onDepthReady(callback.ctx, depth, temperature);
    }
    return true;
  }

  void setCallback(auv::peripheral::DepthDataReadyCallback cb) override {
    callback = cb;
  }

  void start() override { started = true; }

  bool isConnected() const override { return connected; }
  float getDepth() const override { return depth; }
  float getTemperature() const override { return temperature; }

  bool init_ok = true;
  bool connected = true;
  bool initialized = false;
  bool started = false;
  bool ready = false;
  int poll_count = 0;
  float depth = 0.0f;
  float temperature = 0.0f;
  auv::peripheral::DepthDataReadyCallback callback{nullptr, nullptr};
};

class FakePushrodBackend final : public auv::peripheral::PushrodBackend {
public:
  bool init() override {
    initialized = true;
    return true;
  }

  void start() override { started = true; }

  bool sendTask(const PushrodTask &task) override {
    last_task = task;
    ++send_count;
    return send_ok;
  }

  bool readAck(PushrodAck *ack) override {
    if (!ack_ready || ack == nullptr) {
      return false;
    }
    *ack = last_ack;
    ack_ready = false;
    return true;
  }

  bool isSupported() const override { return supported; }

  bool initialized = false;
  bool started = false;
  bool send_ok = true;
  bool supported = true;
  bool ack_ready = false;
  int send_count = 0;
  PushrodTask last_task{};
  PushrodAck last_ack{};
};

} // namespace

TEST(DepthSensorDriverTest, DelegatesLifecycleAndCachesDepthCallback) {
  FakeDepthBackend backend;
  auv::peripheral::Depth_Sensor_Driver driver(&backend);

  driver.Init();
  driver.start();
  EXPECT_TRUE(backend.initialized);
  EXPECT_TRUE(backend.started);
  EXPECT_TRUE(driver.isConnected());

  backend.depth = 1.25f;
  backend.temperature = 23.5f;
  backend.ready = true;
  EXPECT_EQ(driver.Read(), 1);
  EXPECT_EQ(backend.poll_count, 1);
  EXPECT_FLOAT_EQ(driver.getMS5837Z(), 1.25f);
  EXPECT_FLOAT_EQ(driver.getTemperature(), 23.5f);
}

TEST(DepthSensorDriverTest, FailedBackendIsReportedDisconnected) {
  FakeDepthBackend backend;
  backend.init_ok = false;
  backend.connected = false;
  auv::peripheral::Depth_Sensor_Driver driver(&backend);

  driver.Init();
  EXPECT_FALSE(driver.isConnected());
  EXPECT_EQ(driver.Read(), 0);
}

TEST(PushrodDriverTest, DelegatesTaskAndAck) {
  FakePushrodBackend backend;
  auv::peripheral::Pushrod_Driver driver(&backend);

  EXPECT_TRUE(driver.Init());
  driver.start();
  EXPECT_TRUE(driver.isSupported());
  EXPECT_TRUE(backend.initialized);
  EXPECT_TRUE(backend.started);

  PushrodTask task{};
  task.task_id = 7U;
  task.power_x1000 = 500;
  task.duration_ms = 1000U;
  EXPECT_TRUE(driver.sendTask(task));
  EXPECT_EQ(backend.last_task.task_id, 7U);
  EXPECT_EQ(backend.send_count, 1);

  backend.last_ack.task_id = 7U;
  backend.last_ack.result = auv::peripheral::pushrod::kOk;
  backend.ack_ready = true;
  PushrodAck ack{};
  EXPECT_TRUE(driver.readAck(&ack));
  EXPECT_EQ(ack.task_id, 7U);
  EXPECT_EQ(ack.result, auv::peripheral::pushrod::kOk);
}

TEST(PushrodDriverTest, UnsupportedBackendIsSafe) {
  FakePushrodBackend backend;
  backend.supported = false;
  backend.send_ok = false;
  auv::peripheral::Pushrod_Driver driver(&backend);

  EXPECT_FALSE(driver.isSupported());
  PushrodTask task{};
  EXPECT_FALSE(driver.sendTask(task));
}
