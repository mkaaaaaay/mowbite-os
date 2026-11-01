#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "mowbite_localization/gyro_bias.h"

using namespace mowbite_localization;

namespace {

constexpr double kDt = 0.02;

// Feeds seconds of a constant rate at 50 Hz.
void Feed(GyroBias& bias, double seconds, double rate, bool still) {
  for (long i = 0; i < std::lround(seconds / kDt); ++i) bias.Add(rate, kDt, still);
}

// L1: the first offset is the mean over the calibration time at rest.
TEST(L1GyroCalibration, OffsetIsTheMeanAfterTheCalibrationTime) {
  GyroBias bias({});
  Feed(bias, 4.9, 0.01, true);
  EXPECT_FALSE(bias.calibrated());
  Feed(bias, 0.1, 0.03, true);
  ASSERT_TRUE(bias.calibrated());
  EXPECT_NEAR(bias.offset(), (4.9 * 0.01 + 0.1 * 0.03) / 5.0, 1e-9);
}

TEST(L1GyroCalibration, MovingWheelsStartItAgain) {
  GyroBias bias({});
  Feed(bias, 4.0, 0.05, true);
  Feed(bias, 0.5, 0.2, false);
  Feed(bias, 4.9, 0.01, true);
  EXPECT_FALSE(bias.calibrated());
  Feed(bias, 0.1, 0.01, true);
  ASSERT_TRUE(bias.calibrated());
  EXPECT_NEAR(bias.offset(), 0.01, 1e-9);
}

TEST(L1GyroCalibration, SkippedCalibrationTakesTheGivenOffset) {
  GyroBiasConfig config;
  config.skip_calibration = true;
  config.initial_offset = 0.02;
  GyroBias bias(config);
  EXPECT_TRUE(bias.calibrated());
  EXPECT_DOUBLE_EQ(bias.offset(), 0.02);
  EXPECT_DOUBLE_EQ(bias.Corrected(0.12), 0.1);
}

TEST(L1GyroCalibration, NanAndBadStepsAreIgnored) {
  GyroBias bias({});
  Feed(bias, 4.9, 0.01, true);
  bias.Add(std::numeric_limits<double>::quiet_NaN(), kDt, true);
  bias.Add(0.5, 0.0, true);
  bias.Add(0.5, -1.0, true);
  Feed(bias, 0.1, 0.01, true);
  ASSERT_TRUE(bias.calibrated());
  EXPECT_NEAR(bias.offset(), 0.01, 1e-9);
}

GyroBias Calibrated(double offset) {
  GyroBias bias({});
  Feed(bias, 5.0, offset, true);
  EXPECT_TRUE(bias.calibrated());
  return bias;
}

// E2: standing still the heading is held and the offset follows the gyro.
TEST(E2StandingStill, HoldsAfterOneSecondOfStillWheelsAndAQuietGyro) {
  GyroBias bias = Calibrated(0.01);
  Feed(bias, 0.9, 0.01, true);
  EXPECT_FALSE(bias.holding());
  EXPECT_DOUBLE_EQ(bias.Corrected(0.015), 0.005);
  Feed(bias, 0.12, 0.01, true);
  EXPECT_TRUE(bias.holding());
  EXPECT_DOUBLE_EQ(bias.Corrected(0.015), 0.0);
}

TEST(E2StandingStill, TheOffsetFollowsAChangedBias) {
  GyroBias bias = Calibrated(0.0);
  // 0.3 deg/s more while the mower stands, e.g. a warmer sensor.
  const double rate = 0.3 * M_PI / 180.0;
  Feed(bias, 60.0, rate, true);
  EXPECT_TRUE(bias.holding());
  EXPECT_NEAR(bias.offset(), rate, 0.01 * rate);
}

TEST(E2StandingStill, MovingWheelsEndIt) {
  GyroBias bias = Calibrated(0.0);
  Feed(bias, 2.0, 0.0, true);
  ASSERT_TRUE(bias.holding());
  bias.Add(0.001, kDt, false);
  EXPECT_FALSE(bias.holding());
  EXPECT_DOUBLE_EQ(bias.Corrected(0.001), 0.001);
}

TEST(E2StandingStill, TurningByHandIsNotStanding) {
  GyroBias bias = Calibrated(0.0);
  // Wheels still, the body turns at 5 deg/s: nothing is held and the offset stays.
  const double rate = 5.0 * M_PI / 180.0;
  Feed(bias, 3.0, rate, true);
  EXPECT_FALSE(bias.holding());
  EXPECT_DOUBLE_EQ(bias.offset(), 0.0);
  EXPECT_DOUBLE_EQ(bias.Corrected(rate), rate);
}

}  // namespace
