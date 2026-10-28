#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "mowbite_localization/localization.h"

namespace mowbite_localization {
namespace {

constexpr double kStep = 0.02;  // gyro at 50 Hz

GpsFix Fixed(double x, double y) {
  GpsFix fix;
  fix.flags = 1 | kFlagGpsRtkFixed;
  fix.accuracy = 0.02;
  fix.x = x;
  fix.y = y;
  return fix;
}

// Gyro and wheel speed at 50 Hz, stamps equal to the monotonic clock. Returns the number of advanced steps.
int Step(Localization& loc, double& t, double seconds, double gyro, double wheel) {
  int advanced = 0;
  const long steps = std::lround(seconds / kStep);
  for (long i = 0; i < steps; ++i) {
    t += kStep;
    loc.AddWheelSpeed(wheel, t);
    if (loc.AddImu(t, gyro, t)) advanced++;
  }
  return advanced;
}

void Calibrate(Localization& loc, double& t, double gyro = 0.0) {
  Step(loc, t, 5.1, gyro, 0.0);
  ASSERT_TRUE(loc.calibrated());
}

// Feeds 12 fixes at 10 Hz with the robot standing still, enough to move the filter onto the GPS.
void InitGps(Localization& loc, double& t, double x, double y) {
  for (int i = 0; i < 12; ++i) {
    Step(loc, t, 0.1, 0.0, 0.0);
    loc.AddGps(Fixed(x, y), t);
  }
  ASSERT_TRUE(loc.gps_valid());
}

TEST(Angles, NormalizeToHalfOpenRange) {
  EXPECT_NEAR(NormalizeAngle(3 * M_PI), M_PI, 1e-12);
  EXPECT_NEAR(NormalizeAngle(-M_PI), M_PI, 1e-12);
  EXPECT_NEAR(NormalizeAngle(-3 * M_PI / 2), M_PI / 2, 1e-12);
  EXPECT_NEAR(NormalizeAngle(10.0), 10.0 - 4 * M_PI, 1e-12);
}

TEST(Angles, YawFromQuaternion) {
  EXPECT_NEAR(YawFromQuaternion(0, 0, std::sin(0.6), std::cos(0.6)), 1.2, 1e-12);
  EXPECT_NEAR(YawFromQuaternion(0, 0, 0, 1), 0.0, 1e-12);
}

TEST(Localization, CalibratesGyroOffsetAfterFiveSeconds) {
  Localization loc({});
  double t = 100;
  EXPECT_EQ(Step(loc, t, 4.9, 0.01, 0.0), 0);
  EXPECT_FALSE(loc.calibrated());
  Step(loc, t, 0.2, 0.01, 0.0);
  EXPECT_TRUE(loc.calibrated());
  EXPECT_NEAR(loc.gyro_offset(), 0.01, 1e-12);
}

TEST(Localization, CalibratedOffsetKeepsTheHeading) {
  Localization loc({});
  double t = 100;
  Calibrate(loc, t, 0.01);
  EXPECT_GT(Step(loc, t, 10.0, 0.01, 0.0), 490);
  EXPECT_NEAR(loc.GetEstimate().theta, 0.0, 1e-9);
}

TEST(Localization, CalibrationStartsAgainWhileWheelsMove) {
  Localization loc({});
  double t = 100;
  Step(loc, t, 3.0, 0.05, 0.2);
  Step(loc, t, 4.9, 0.01, 0.0);
  EXPECT_FALSE(loc.calibrated());
  Step(loc, t, 0.2, 0.01, 0.0);
  EXPECT_TRUE(loc.calibrated());
  EXPECT_NEAR(loc.gyro_offset(), 0.01, 1e-12);
}

TEST(Localization, SkippedCalibrationUsesTheGivenOffset) {
  LocalizationConfig config;
  config.skip_gyro_calibration = true;
  config.gyro_offset = 0.01;
  Localization loc(config);
  double t = 1000;
  // The first sample only starts the integration, there is no step from time zero.
  EXPECT_FALSE(loc.AddImu(t, 0.11, t));
  EXPECT_TRUE(loc.calibrated());
  EXPECT_NEAR(loc.GetEstimate().theta, 0.0, 1e-12);
  EXPECT_EQ(Step(loc, t, 1.0, 0.11, 0.0), 50);
  EXPECT_NEAR(loc.GetEstimate().theta, 0.1, 1e-9);
}

TEST(Localization, DeadReckoningStraight) {
  Localization loc({});
  double t = 100;
  Calibrate(loc, t);
  Step(loc, t, 5.0, 0.0, 0.2);
  const Estimate e = loc.GetEstimate();
  EXPECT_NEAR(e.x, 1.0, 1e-6);
  EXPECT_NEAR(e.y, 0.0, 1e-9);
  EXPECT_NEAR(e.vx, 0.2, 1e-3);
}

TEST(Localization, DeadReckoningQuarterCircle) {
  Localization loc({});
  double t = 100;
  Calibrate(loc, t);
  const double rate = M_PI / 10;
  Step(loc, t, 5.0, rate, 0.2);
  const Estimate e = loc.GetEstimate();
  const double radius = 0.2 / rate;
  EXPECT_NEAR(e.theta, M_PI / 2, 1e-9);
  EXPECT_NEAR(e.x, radius, 0.01);
  EXPECT_NEAR(e.y, radius, 0.01);
  EXPECT_NEAR(e.vr, rate, 1e-3);
}

TEST(Localization, StaleWheelSpeedCountsAsZero) {
  Localization loc({});
  double t = 100;
  Calibrate(loc, t);
  loc.AddWheelSpeed(0.3, t);
  for (int i = 0; i < 100; ++i) {
    t += kStep;
    loc.AddImu(t, 0.0, t);
  }
  // 0.5 s at 0.3 m/s, then standstill.
  EXPECT_NEAR(loc.GetEstimate().x, 0.15, 0.3 * kStep + 1e-9);
}

TEST(Localization, ImuGapIsNotIntegrated) {
  Localization loc({});
  double t = 100;
  Calibrate(loc, t);
  t += 2.0;
  EXPECT_FALSE(loc.AddImu(t, 1.0, t));
  EXPECT_NEAR(loc.GetEstimate().theta, 0.0, 1e-12);
  t += kStep;
  EXPECT_TRUE(loc.AddImu(t, 1.0, t));
  EXPECT_NEAR(loc.GetEstimate().theta, kStep, 1e-9);
}

TEST(Localization, HeadingStaysWrapped) {
  Localization loc({});
  double t = 100;
  Calibrate(loc, t);
  Step(loc, t, 10.0, 1.0, 0.0);
  const double theta = loc.GetEstimate().theta;
  EXPECT_GT(theta, -M_PI);
  EXPECT_LE(theta, M_PI);
  EXPECT_NEAR(theta, NormalizeAngle(10.0), 1e-6);
}

TEST(Localization, DropsFixesThatAreNotRtkFixedOrNotAccurate) {
  Localization loc({});
  GpsFix fl = Fixed(1, 1);
  fl.flags = 1 | 4;
  EXPECT_EQ(loc.AddGps(fl, 100), GpsResult::kNotFixed);
  GpsFix coarse = Fixed(1, 1);
  coarse.accuracy = 0.2;
  EXPECT_EQ(loc.AddGps(coarse, 100), GpsResult::kInaccurate);
  coarse.accuracy = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(loc.AddGps(coarse, 100), GpsResult::kInaccurate);
  loc.SetGpsEnabled(false);
  EXPECT_EQ(loc.AddGps(Fixed(1, 1), 100), GpsResult::kDisabled);
  EXPECT_FALSE(loc.RecentGps(100));
}

TEST(Localization, NanSamplesAreDropped) {
  Localization loc({});
  double t = 100;
  Calibrate(loc, t);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  t += kStep;
  EXPECT_FALSE(loc.AddImu(t, nan, t));
  loc.AddWheelSpeed(nan, t);
  EXPECT_EQ(Step(loc, t, 1.0, 0.0, 0.0), 50);
  EXPECT_EQ(loc.AddGps(Fixed(nan, 1), t), GpsResult::kInaccurate);
  const Estimate e = loc.GetEstimate();
  EXPECT_TRUE(std::isfinite(e.x) && std::isfinite(e.y) && std::isfinite(e.theta));
  EXPECT_FALSE(loc.RecentGps(t));
}

TEST(Localization, MovesOntoGpsAfterElevenInliers) {
  Localization loc({});
  double t = 100;
  Calibrate(loc, t);
  EXPECT_EQ(loc.AddGps(Fixed(10, 20), t), GpsResult::kRestarted);
  for (int i = 0; i < 10; ++i) {
    Step(loc, t, 0.1, 0.0, 0.0);
    EXPECT_EQ(loc.AddGps(Fixed(10, 20), t), GpsResult::kWarmup);
  }
  EXPECT_NEAR(loc.GetEstimate().x, 0.0, 1e-9);
  EXPECT_EQ(loc.PositionAccuracy(t), kNoAccuracy);
  // During the warmup the fusion already counts as recent.
  EXPECT_TRUE(loc.RecentGps(t));
  Step(loc, t, 0.1, 0.0, 0.0);
  EXPECT_EQ(loc.AddGps(Fixed(10, 20), t), GpsResult::kInitialized);
  const Estimate e = loc.GetEstimate();
  EXPECT_NEAR(e.x, 10.0, 0.001);
  EXPECT_NEAR(e.y, 20.0, 0.001);
  EXPECT_NEAR(loc.PositionAccuracy(t), 0.02, 1e-6);
  Step(loc, t, 0.1, 0.0, 0.0);
  EXPECT_EQ(loc.AddGps(Fixed(10, 20), t), GpsResult::kFused);
}

TEST(Localization, GapOfFiveSecondsStartsTheWarmupAgain) {
  Localization loc({});
  double t = 100;
  Calibrate(loc, t);
  InitGps(loc, t, 10, 20);
  Step(loc, t, 5.1, 0.0, 0.0);
  EXPECT_EQ(loc.AddGps(Fixed(10, 20), t), GpsResult::kRestarted);
  EXPECT_FALSE(loc.gps_valid());
  EXPECT_EQ(loc.PositionAccuracy(t), kNoAccuracy);
  Step(loc, t, 0.1, 0.0, 0.0);
  EXPECT_EQ(loc.AddGps(Fixed(10, 20), t), GpsResult::kWarmup);
}

TEST(Localization, AccuracyExpiresTenSecondsAfterTheLastFix) {
  LocalizationConfig config;
  config.accuracy_growth_per_s = 0.0;
  Localization loc(config);
  double t = 100;
  Calibrate(loc, t);
  InitGps(loc, t, 10, 20);
  EXPECT_NEAR(loc.PositionAccuracy(t + 9.9), 0.02, 1e-6);
  EXPECT_TRUE(loc.RecentGps(t + 9.9));
  EXPECT_EQ(loc.PositionAccuracy(t + 10.0), kNoAccuracy);
  EXPECT_FALSE(loc.RecentGps(t + 10.0));
}

TEST(Localization, AccuracyGrowsWithDistanceAndTimeWithoutAFix) {
  LocalizationConfig config;
  config.accuracy_growth_per_m = 0.05;
  config.accuracy_growth_per_s = 0.01;
  Localization loc(config);
  double t = 100;
  Calibrate(loc, t);
  InitGps(loc, t, 10, 20);
  EXPECT_NEAR(loc.PositionAccuracy(t), 0.02, 1e-6);
  // 2 s standing: time only.
  Step(loc, t, 2.0, 0.0, 0.0);
  EXPECT_NEAR(loc.PositionAccuracy(t), 0.02 + 0.02, 1e-6);
  // 2 s at 0.3 m/s: 0.6 m more.
  Step(loc, t, 2.0, 0.0, 0.3);
  EXPECT_NEAR(loc.PositionAccuracy(t), 0.02 + 0.04 + 0.03, 1e-3);
  // Reversing counts the same.
  Step(loc, t, 1.0, 0.0, -0.3);
  EXPECT_NEAR(loc.PositionAccuracy(t), 0.02 + 0.05 + 0.045, 1e-3);
  // The next fused fix starts again from its own accuracy.
  GpsFix fix = Fixed(loc.GetEstimate().x, loc.GetEstimate().y);
  EXPECT_EQ(loc.AddGps(fix, t), GpsResult::kFused);
  EXPECT_NEAR(loc.PositionAccuracy(t), 0.02, 1e-6);
}

TEST(Localization, OutliersAreIgnoredUntilTheEleventh) {
  Localization loc({});
  double t = 100;
  Calibrate(loc, t);
  InitGps(loc, t, 10, 20);
  const Estimate before = loc.GetEstimate();
  for (int i = 0; i < 10; ++i) {
    Step(loc, t, 0.1, 0.0, 0.0);
    EXPECT_EQ(loc.AddGps(Fixed(16, 20), t), GpsResult::kOutlier);
  }
  EXPECT_NEAR(loc.GetEstimate().x, before.x, 1e-9);
  EXPECT_TRUE(loc.gps_valid());
  Step(loc, t, 0.1, 0.0, 0.0);
  EXPECT_EQ(loc.AddGps(Fixed(16, 20), t), GpsResult::kOutlierReset);
  EXPECT_FALSE(loc.gps_valid());
  Step(loc, t, 0.1, 0.0, 0.0);
  EXPECT_EQ(loc.AddGps(Fixed(16.01, 20), t), GpsResult::kWarmup);
}

TEST(Localization, AntennaOffsetIsRemovedFromTheFix) {
  LocalizationConfig config;
  config.antenna_offset_x = 0.3;
  Localization loc(config);
  double t = 100;
  Calibrate(loc, t);
  loc.SetPose(0, 0, M_PI / 2);
  // Facing north, the antenna is 0.3 m north of the point of rotation.
  InitGps(loc, t, 10, 10.3);
  for (int i = 0; i < 50; ++i) {
    Step(loc, t, 0.1, 0.0, 0.0);
    loc.AddGps(Fixed(10, 10.3), t);
  }
  const Estimate e = loc.GetEstimate();
  EXPECT_NEAR(e.x, 10.0, 0.01);
  EXPECT_NEAR(e.y, 10.0, 0.01);
  EXPECT_NEAR(e.theta, M_PI / 2, 1e-6);
}

TEST(Localization, MotionVectorCorrectsTheHeading) {
  Localization loc({});
  double t = 100;
  Calibrate(loc, t);
  // The filter starts facing east while the mower drives north at 0.3 m/s.
  const double speed = 0.3;
  double north = 0;
  for (int i = 0; i < 300; ++i) {
    for (int k = 0; k < 5; ++k) {
      t += kStep;
      north += speed * kStep;
      loc.AddWheelSpeed(speed, t);
      loc.AddImu(t, 0.0, t);
    }
    GpsFix fix = Fixed(0, north);
    fix.motion_valid = true;
    fix.motion_y = speed;
    loc.AddGps(fix, t);
  }
  const Estimate e = loc.GetEstimate();
  EXPECT_NEAR(e.theta, M_PI / 2, 0.05);
  EXPECT_NEAR(e.x, 0.0, 0.1);
  EXPECT_NEAR(e.y, north, 0.1);
}

namespace {

// Drives straight at speed along course (rad, from east) for the given number of fixes at 10 Hz, with a motion
// vector of the true course in every fix. Position follows the true course.
void DriveWithCourse(Localization& loc, double& t, double& x, double& y, double course, double speed, int fixes) {
  for (int i = 0; i < fixes; ++i) {
    for (int k = 0; k < 5; ++k) {
      t += kStep;
      x += speed * kStep * std::cos(course);
      y += speed * kStep * std::sin(course);
      loc.AddWheelSpeed(speed, t);
      loc.AddImu(t, 0.0, t);
    }
    GpsFix fix = Fixed(x, y);
    fix.motion_valid = true;
    fix.motion_x = speed * std::cos(course);
    fix.motion_y = speed * std::sin(course);
    loc.AddGps(fix, t);
  }
}

}  // namespace

TEST(Localization, HeadingIsNotValidBeforeTheFirstAlignment) {
  Localization loc({});
  double t = 100;
  EXPECT_FALSE(loc.heading_valid());
  Calibrate(loc, t);
  InitGps(loc, t, 0, 0);
  // Standing with fixes does not align, neither does driving below 0.1 m/s.
  for (int i = 0; i < 50; ++i) {
    Step(loc, t, 0.1, 0.0, 0.0);
    loc.AddGps(Fixed(0, 0), t);
  }
  EXPECT_FALSE(loc.heading_valid());
  double x = 0, y = 0;
  DriveWithCourse(loc, t, x, y, 0.0, 0.05, 50);
  EXPECT_FALSE(loc.heading_valid());
}

TEST(Localization, HeadingIsValidAfterFiveAgreeingMotionVectors) {
  Localization loc({});
  double t = 100;
  Calibrate(loc, t);
  InitGps(loc, t, 0, 0);
  double x = 0, y = 0;
  // The filter faces east and the mower drives east: four agreeing vectors are not enough, the fifth is.
  DriveWithCourse(loc, t, x, y, 0.0, 0.3, 4);
  EXPECT_FALSE(loc.heading_valid());
  DriveWithCourse(loc, t, x, y, 0.0, 0.3, 1);
  EXPECT_TRUE(loc.heading_valid());
}

TEST(Localization, WrongHeadingIsValidOnlyOnceTheFilterTurned) {
  Localization loc({});
  double t = 100;
  Calibrate(loc, t);
  InitGps(loc, t, 0, 0);
  double x = 0, y = 0;
  // The filter faces east while the mower drives north.
  DriveWithCourse(loc, t, x, y, M_PI / 2, 0.3, 20);
  EXPECT_FALSE(loc.heading_valid());
  DriveWithCourse(loc, t, x, y, M_PI / 2, 0.3, 300);
  EXPECT_TRUE(loc.heading_valid());
  EXPECT_NEAR(loc.GetEstimate().theta, M_PI / 2, 10.0 * M_PI / 180.0);
}

TEST(Localization, SetPoseAlignsTheHeading) {
  Localization loc({});
  EXPECT_FALSE(loc.heading_valid());
  loc.SetPose(1, 2, 0.5);
  EXPECT_TRUE(loc.heading_valid());
}

TEST(Localization, SlowMotionVectorIsIgnored) {
  Localization loc({});
  double t = 100;
  Calibrate(loc, t);
  InitGps(loc, t, 0, 0);
  for (int i = 0; i < 50; ++i) {
    Step(loc, t, 0.1, 0.0, 0.0);
    GpsFix fix = Fixed(0, 0);
    fix.motion_valid = true;
    fix.motion_y = 0.005;
    loc.AddGps(fix, t);
  }
  EXPECT_NEAR(loc.GetEstimate().theta, 0.0, 1e-9);
}

TEST(Localization, SetPoseRestartsTheFilter) {
  Localization loc({});
  double t = 100;
  Calibrate(loc, t);
  Step(loc, t, 2.0, 0.1, 0.2);
  loc.SetPose(1, 2, 3);
  const Estimate e = loc.GetEstimate();
  EXPECT_DOUBLE_EQ(e.x, 1);
  EXPECT_DOUBLE_EQ(e.y, 2);
  EXPECT_DOUBLE_EQ(e.theta, 3);
  EXPECT_DOUBLE_EQ(e.vx, 0);
  EXPECT_DOUBLE_EQ(e.pose_covariance[0], 1.0);
  EXPECT_DOUBLE_EQ(e.pose_covariance[1], 0.0);
  EXPECT_DOUBLE_EQ(e.pose_covariance[8], 1.0);
}

}  // namespace
}  // namespace mowbite_localization
