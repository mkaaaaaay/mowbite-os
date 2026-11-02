#include <gtest/gtest.h>

#include "xcore_driver/drive_regulator.h"

using namespace xcore;

TEST(DriveRegulator, FeedForwardAndLimit) {
  RegulatorConfig config;
  config.wheel_distance = 0.4;
  config.full_duty_speed = 0.8;
  config.max_duty = 0.5;
  DriveRegulator r(config);
  auto d = r.Update(0.2, 0, 0);
  EXPECT_DOUBLE_EQ(d.left_duty, 0.25);
  EXPECT_DOUBLE_EQ(d.right_duty, 0.25);
  EXPECT_DOUBLE_EQ(d.linear, 0.25);
  EXPECT_DOUBLE_EQ(d.angular, 0);
  // Turning on the spot: the board splits the twist with the same distance back into the wheels.
  d = r.Update(0, 1.0, 0.1);
  EXPECT_DOUBLE_EQ(d.left_duty, -0.25);
  EXPECT_DOUBLE_EQ(d.right_duty, 0.25);
  EXPECT_DOUBLE_EQ(d.linear - d.angular * 0.2, -0.25);
  EXPECT_DOUBLE_EQ(d.linear + d.angular * 0.2, 0.25);
  d = r.Update(1.0, 0, 0.2);
  EXPECT_DOUBLE_EQ(d.left_duty, 0.5);
  EXPECT_DOUBLE_EQ(d.right_duty, 0.5);
}

TEST(DriveRegulator, FeedbackCorrectsTowardsTheTarget) {
  RegulatorConfig config;
  config.feedback = true;
  config.max_duty = 1.0;
  DriveRegulator r(config);
  double first = 0;
  for (int i = 0; i < 10; i++) {
    double t = i * 0.1;
    // Measured only half the wanted speed.
    r.OnMeasured(0.1, 0.1, t);
    auto d = r.Update(0.2, 0, t);
    if (i == 0) first = d.left_duty;
    EXPECT_GT(d.left_duty, 0.2);
  }
  auto d = r.Update(0.2, 0, 1.0);
  EXPECT_GT(d.left_duty, first);
  // Without fresh measurements only feed forward.
  d = r.Update(0.2, 0, 2.0);
  EXPECT_NEAR(d.left_duty, 0.2, 1e-9);
  // A zero target clears the integral at once.
  EXPECT_EQ(r.Update(0, 0, 2.1).left_duty, 0);
}

TEST(DriveRegulator, WheelAgainstSetpointFaults) {
  RegulatorConfig config;
  DriveRegulator r(config);
  for (int i = 0; i <= 4; i++) {
    r.OnMeasured(-0.1, 0.1, i * 0.1);
    r.Update(0.2, 0, i * 0.1);
    EXPECT_FALSE(r.fault());
  }
  r.OnMeasured(-0.1, 0.1, 0.5);
  auto d = r.Update(0.2, 0, 0.5);
  EXPECT_TRUE(r.fault());
  EXPECT_EQ(d.linear, 0);
  EXPECT_EQ(r.Update(0.2, 0, 0.6).left_duty, 0);
  r.ClearFault();
  EXPECT_GT(r.Update(0.2, 0, 0.7).left_duty, 0);
  // Slow creep against the setpoint does not count.
  DriveRegulator slow(config);
  for (int i = 0; i <= 10; i++) {
    slow.OnMeasured(-0.02, 0.1, i * 0.1);
    slow.Update(0.2, 0, i * 0.1);
  }
  EXPECT_FALSE(slow.fault());
}
