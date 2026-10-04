#include "MotionController_Driver.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <vector>

namespace {

struct TxCapture {
  bool accept = true;
  int calls = 0;
  std::vector<uint8_t> frame;
};

bool captureTx(void *ctx, const uint8_t *data, uint16_t size) {
  auto *capture = static_cast<TxCapture *>(ctx);
  ++capture->calls;
  capture->frame.assign(data, data + size);
  return capture->accept;
}

void *unusedTxPacket(void *) { return nullptr; }

TEST(MotionControllerServo, MapsLogicalIdsToFirmwareCommandIds) {
  TxCapture capture;
  auv::peripheral::ThrustPacket backing{};
  auv::peripheral::MotorPortOps ops{&capture, &captureTx, &unusedTxPacket};
  auv::peripheral::MotionController_Driver driver(ops, &backing);

  ASSERT_TRUE(driver.setServoAngle(1, 0.75F));
  ASSERT_EQ(capture.frame.size(), sizeof(auv::peripheral::ServoPacket));
  EXPECT_EQ(capture.frame[0], 0xFA);
  EXPECT_EQ(capture.frame[1], 0xAF);
  EXPECT_EQ(capture.frame[2], 0x02);
  EXPECT_EQ(capture.frame[7], 0xFB);
  EXPECT_EQ(capture.frame[8], 0xBF);
  float transmitted_angle = 0.0F;
  std::memcpy(&transmitted_angle, capture.frame.data() + 3,
              sizeof(transmitted_angle));
  EXPECT_FLOAT_EQ(transmitted_angle, 0.75F);

  ASSERT_TRUE(driver.setServoAngle(2, -1.25F));
  EXPECT_EQ(capture.frame[2], 0x04);
  std::memcpy(&transmitted_angle, capture.frame.data() + 3,
              sizeof(transmitted_angle));
  EXPECT_FLOAT_EQ(transmitted_angle, -1.25F);

  float servo1 = 0.0F;
  float servo2 = 0.0F;
  driver.getServoAngles(servo1, servo2);
  EXPECT_FLOAT_EQ(servo1, 0.75F);
  EXPECT_FLOAT_EQ(servo2, -1.25F);
}

TEST(MotionControllerServo, TracksOnlyDmaAcceptedCommands) {
  TxCapture capture;
  auv::peripheral::ThrustPacket backing{};
  auv::peripheral::MotorPortOps ops{&capture, &captureTx, &unusedTxPacket};
  auv::peripheral::MotionController_Driver driver(ops, &backing);

  capture.accept = false;
  EXPECT_FALSE(driver.setServoAngle(1, 1.0F));
  EXPECT_FALSE(driver.setServoAngle(3, 2.0F));
  EXPECT_EQ(capture.calls, 1);

  float servo1 = -1.0F;
  float servo2 = -1.0F;
  driver.getServoAngles(servo1, servo2);
  EXPECT_FLOAT_EQ(servo1, 0.0F);
  EXPECT_FLOAT_EQ(servo2, 0.0F);
}

} // namespace
