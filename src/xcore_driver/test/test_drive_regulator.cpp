#include <gtest/gtest.h>

#include <cmath>

#include "xcore_driver/drive_regulator.h"

using namespace xcore;

TEST(DriveRegulator, FeedForwardAndLimit) {
  RegulatorConfig config;
  config.wheel_distance = 0.4;
  config.duty_slope = 0.8;
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

TEST(DriveRegulator, DeadbandAddsToEveryWheelThatTurns) {
  RegulatorConfig config;
  config.wheel_distance = 0.5;
  config.duty_slope = 0.58;
  config.duty_deadband = 0.019;
  config.max_duty = 0.3;
  DriveRegulator r(config);
  auto d = r.Update(0.058, 0, 0);
  EXPECT_NEAR(d.left_duty, 0.119, 1e-12);
  EXPECT_NEAR(d.right_duty, 0.119, 1e-12);
  d = r.Update(-0.058, 0, 0.1);
  EXPECT_NEAR(d.left_duty, -0.119, 1e-12);
  // A wheel with a target of 0 gets 0, not the deadband.
  d = r.Update(0.05, 0.2, 0.2);
  EXPECT_EQ(d.left_duty, 0);
  EXPECT_NEAR(d.right_duty, 0.1 / 0.58 + 0.019, 1e-12);
  EXPECT_EQ(r.Update(0, 0, 0.3).left_duty, 0);
}

TEST(DriveRegulator, SaturationKeepsTheCurvature) {
  RegulatorConfig config;
  config.wheel_distance = 0.3556;
  config.duty_slope = 0.58;
  config.duty_deadband = 0.019;
  config.max_duty = 0.3;
  DriveRegulator r(config);
  // Left 0.15, right 0.25 m/s: duties 0.2776 and 0.4500; the right one saturates, both shrink by 0.3 / 0.45.
  auto d = r.Update(0.2, 0.1 / 0.3556, 0);
  EXPECT_NEAR(d.right_duty, 0.3, 1e-12);
  const double left = (0.15 / 0.58 + 0.019) * 0.3 / (0.25 / 0.58 + 0.019);
  EXPECT_NEAR(d.left_duty, left, 1e-12);
  EXPECT_NEAR(d.left_duty / d.right_duty, (0.15 / 0.58 + 0.019) / (0.25 / 0.58 + 0.019), 1e-12);
  // Backwards and turning the other way: the same factor, the larger magnitude at -max_duty.
  d = r.Update(-0.2, 0.1 / 0.3556, 0.1);
  EXPECT_NEAR(d.left_duty, -0.3, 1e-12);
  EXPECT_NEAR(d.right_duty, -left, 1e-12);
  // max_duty 0 locks both wheels.
  config.max_duty = 0;
  DriveRegulator locked(config);
  d = locked.Update(0.3, 0.3, 0);
  EXPECT_EQ(d.left_duty, 0);
  EXPECT_EQ(d.right_duty, 0);
  EXPECT_EQ(d.linear, 0);
  EXPECT_EQ(d.angular, 0);
}

TEST(DriveRegulator, SpeedModelCheckAndReach) {
  RegulatorConfig config;
  config.duty_slope = 0.58;
  config.duty_deadband = 0.019;
  config.max_duty = 0.3;
  EXPECT_FALSE(CheckSpeedModel(config));
  EXPECT_NEAR(ReachableWheelSpeed(config), 0.58 * 0.281, 1e-12);
  config.max_duty = 0.9;
  EXPECT_NEAR(ReachableWheelSpeed(config), 0.58 * 0.881, 1e-12);
  config.max_duty = 0;
  EXPECT_FALSE(CheckSpeedModel(config));
  EXPECT_EQ(ReachableWheelSpeed(config), 0);
  auto bad = [](double slope, double deadband, double max_duty) {
    RegulatorConfig c;
    c.duty_slope = slope;
    c.duty_deadband = deadband;
    c.max_duty = max_duty;
    return CheckSpeedModel(c).has_value();
  };
  EXPECT_TRUE(bad(0, 0.019, 0.3));
  EXPECT_TRUE(bad(-0.58, 0.019, 0.3));
  EXPECT_TRUE(bad(0.58, -0.01, 0.3));
  EXPECT_TRUE(bad(0.58, 0.3, 0.3));
  EXPECT_TRUE(bad(0.58, 0.4, 0.3));
  EXPECT_TRUE(bad(std::nan(""), 0.019, 0.3));
  EXPECT_TRUE(bad(0.58, std::nan(""), 0.3));
  EXPECT_TRUE(bad(0.58, 0.019, std::nan("")));
  EXPECT_TRUE(bad(0.58, 0.019, 1.5));
  EXPECT_FALSE(bad(0.58, 0.299, 0.3));
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
