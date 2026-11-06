#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

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

TEST(DriveRegulator, NoDutyWithMaxDutyZeroEvenWithFeedback) {
  RegulatorConfig config;
  config.feedback = true;
  config.max_duty = 0;
  DriveRegulator r(config);
  // The wheels stand while the twist asks for full speed: the integral grows, the duty stays 0.
  for (int i = 0; i < 20; i++) {
    const double t = i * 0.1;
    r.OnMeasured(0, 0, t);
    for (const auto& d : {r.Update(0.3, 0, t), r.Update(0, 0.3, t + 0.01), r.Update(-0.3, -0.3, t + 0.02)}) {
      EXPECT_EQ(d.left_duty, 0);
      EXPECT_EQ(d.right_duty, 0);
    }
  }
}

TEST(DriveRegulator, SetpointsThatAreNoNumberStop) {
  RegulatorConfig config;
  config.feedback = true;
  config.max_duty = 1.0;
  DriveRegulator r(config);
  r.OnMeasured(0, 0, 0);
  EXPECT_GT(r.Update(0.2, 0, 0).left_duty, 0);
  for (const auto& [linear, angular] : std::vector<std::pair<double, double>>{
           {NAN, 0}, {0.2, NAN}, {INFINITY, 0}, {0.2, -INFINITY}, {NAN, NAN}}) {
    const auto d = r.Update(linear, angular, 0.1);
    EXPECT_EQ(d.left_duty, 0) << linear << " " << angular;
    EXPECT_EQ(d.right_duty, 0) << linear << " " << angular;
    EXPECT_EQ(d.linear, 0);
    EXPECT_EQ(d.angular, 0);
  }
  // The integral starts again from 0: the next duty is feed forward only.
  r.OnMeasured(0.2, 0.2, 0.2);
  EXPECT_NEAR(r.Update(0.2, 0, 0.2).left_duty, 0.2, 1e-9);
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
  EXPECT_FALSE(bad(0.58, 0, 0.3));
  EXPECT_FALSE(bad(0.58, 0.019, 1.0));
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

TEST(DriveRegulator, MeasurementsCorrectUpToAFifthOfASecond) {
  RegulatorConfig config;
  config.feedback = true;
  config.max_duty = 1.0;
  DriveRegulator r(config);
  r.OnMeasured(0.1, 0.1, 0);
  // Exactly 0.2 s old: feed forward 0.2 and 0.3 x 0.1 for the error, no time yet for the integral.
  EXPECT_NEAR(r.Update(0.2, 0, 0.2).left_duty, 0.23, 1e-12);
  EXPECT_NEAR(r.Update(0.2, 0, 0.25).left_duty, 0.2, 1e-12);
}

// Measured 0.1 m/s below the target of 0.2 m/s on both wheels: duty 0.2 + 0.3 x 0.1 + 0.6 x integral.
TEST(DriveRegulator, TheIntegralFollowsTheTimeBetweenUpdates) {
  RegulatorConfig config;
  config.feedback = true;
  config.max_duty = 1.0;
  DriveRegulator r(config);
  auto left = [&r](double t) {
    r.OnMeasured(0.1, 0.1, t);
    return r.Update(0.2, 0, t).left_duty;
  };
  EXPECT_NEAR(left(1.0), 0.23, 1e-12);
  EXPECT_NEAR(left(1.1), 0.23 + 0.6 * 0.01, 1e-12);
  // A gap of 0.3 s counts as 0.2 s.
  EXPECT_NEAR(left(1.4), 0.23 + 0.6 * 0.03, 1e-12);
  // The integral stops at integral_limit.
  double t = 1.4;
  for (int i = 0; i < 20; i++) left(t += 0.2);
  EXPECT_NEAR(left(t += 0.2), 0.23 + 0.6 * 0.2, 1e-12);
  // A target of 0 clears it; the next update has 0.1 s.
  r.Update(0, 0, t += 0.1);
  EXPECT_NEAR(left(t += 0.1), 0.23 + 0.6 * 0.01, 1e-12);
  // A reset clears both wheels and the time of the last update.
  for (int i = 0; i < 5; i++) left(t += 0.2);
  r.Reset();
  r.OnMeasured(0.1, 0.1, t += 0.1);
  auto d = r.Update(0.2, 0, t);
  EXPECT_NEAR(d.left_duty, 0.23, 1e-12);
  EXPECT_NEAR(d.right_duty, 0.23, 1e-12);
  // An update at 0 counts as one.
  DriveRegulator from_zero(config);
  from_zero.OnMeasured(0.1, 0.1, 0);
  EXPECT_NEAR(from_zero.Update(0.2, 0, 0).left_duty, 0.23, 1e-12);
  from_zero.OnMeasured(0.1, 0.1, 0.1);
  EXPECT_NEAR(from_zero.Update(0.2, 0, 0.1).left_duty, 0.23 + 0.6 * 0.01, 1e-12);
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

namespace {

// Updates every 0.1 s from t0 to t1 with a forward target of 0.2 m/s and the wheels measured as given.
void Against(DriveRegulator& r, double t0, double t1, double left, double right, double target = 0.2) {
  for (double t = t0; t < t1 + 1e-9; t += 0.1) {
    r.OnMeasured(left, right, t);
    r.Update(target, 0, t);
  }
}

}  // namespace

TEST(DriveRegulator, EachWheelAgainstItsSetpointFaults) {
  RegulatorConfig config;
  DriveRegulator r(config);
  // Late in time, so that the time against counts from its start.
  Against(r, 10.0, 10.4, 0.1, -0.1);
  EXPECT_FALSE(r.fault());
  Against(r, 10.5, 10.5, 0.1, -0.1);
  EXPECT_TRUE(r.fault());
}

TEST(DriveRegulator, TheTimeAgainstStartsAgain) {
  RegulatorConfig config;
  DriveRegulator r(config);
  // Interrupted by a sample in the direction of the setpoint.
  Against(r, 10.0, 10.3, -0.1, 0.1);
  Against(r, 10.4, 10.4, 0.1, 0.1);
  Against(r, 10.5, 10.8, -0.1, 0.1);
  EXPECT_FALSE(r.fault());
  // By a target of 0.
  r.Update(0, 0, 10.9);
  Against(r, 11.0, 11.4, -0.1, 0.1);
  EXPECT_FALSE(r.fault());
  // After a fault cleared.
  Against(r, 11.5, 11.5, -0.1, 0.1);
  ASSERT_TRUE(r.fault());
  r.ClearFault();
  Against(r, 11.6, 12.0, -0.1, 0.1);
  EXPECT_FALSE(r.fault());
}

TEST(DriveRegulator, AgainstMeansTheOtherDirectionAboveTheReverseSpeed) {
  RegulatorConfig config;
  DriveRegulator r(config);
  Against(r, 0, 1.0, -0.05, 0.1);
  EXPECT_FALSE(r.fault());
  // The smallest setpoint: the product with the speed is 0, the direction is the same.
  Against(r, 1.1, 2.0, 0.1, 0.1, std::numeric_limits<double>::denorm_min());
  EXPECT_FALSE(r.fault());
}
