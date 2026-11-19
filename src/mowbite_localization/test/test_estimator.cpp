#include <gtest/gtest.h>

#include <Eigen/LU>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <optional>
#include <random>
#include <vector>

#include "mowbite_localization/estimator.h"

using namespace mowbite_localization;

namespace {

constexpr double kStep = 0.02;
constexpr double kDeg = M_PI / 180.0;

// A mower on a unicycle without faults unless set: gyro and wheels at 50 Hz, an RTK fix of the antenna with its
// velocity at 5 Hz. Durations that are multiples of 0.2 s keep the fixes at the end of a run.
class World {
 public:
  explicit World(const EstimatorConfig& config = {}, const Eigen::Vector2d& lever = Eigen::Vector2d(0.1, 0.0))
      : est(config), lever(lever) {}

  void Geometry() { est.SetAntennaOffset(lever(0), lever(1), t); }

  // 5.2 s at rest: the gyro calibrates, the fixes until then do not count.
  void Calibrate() {
    Run(5.2, 0.0, 0.0);
    EXPECT_TRUE(est.calibrated());
  }

  // Calibrated, and GNSS has set the estimate.
  void Ready() {
    Geometry();
    Calibrate();
    Run(2.4, 0.0, 0.0);
    EXPECT_TRUE(est.gps_valid());
  }

  // Calibrated, GNSS set, heading aligned by a short drive (fixes up to 0.2 s late included).
  void Aligned() {
    Ready();
    Run(1.4, 0.3, 0.0);
    EXPECT_TRUE(est.heading_valid());
    Run(0.4, 0.0, 0.0);
  }

  // Drives at speed (m/s) and turn rate (rad/s); returns how many steps moved the estimate.
  int Run(double seconds, double speed, double turn) {
    return Move(seconds, speed, turn, odom_scale * speed, odom_scale * turn);
  }

  // The wheels turn on the spot at wheel_turn (rad/s on a track of 0.4 m) and lose slip_left and slip_right of their
  // speed on the ground: the base turns less and moves along its heading, the wheels report neither.
  int Spin(double seconds, double wheel_turn, double slip_left, double slip_right) {
    const double left = -0.2 * wheel_turn * (1.0 - slip_left);
    const double right = 0.2 * wheel_turn * (1.0 - slip_right);
    return Move(seconds, (left + right) / 2, (right - left) / 0.4, 0.0, wheel_turn);
  }

  // Moves at speed and turn while the wheels report wheel_speed and wheel_turn.
  int Move(double seconds, double speed, double turn, double wheel_speed, double wheel_turn) {
    int moved = 0;
    for (long i = 0; i < std::lround(seconds / kStep); ++i) {
      t += kStep;
      const double mid = heading + 0.5 * turn * kStep;
      x += speed * kStep * std::cos(mid);
      y += speed * kStep * std::sin(mid);
      heading = WrapAngle(heading + turn * kStep);
      est.AddWheels(t, wheel_speed, wheel_turn);
      if (tilt) est.AddTilt(t, tilt->roll, tilt->pitch);
      if (est.AddImu(t, turn + gyro_bias)) moved++;
      const Eigen::Vector2d at = true_lever.value_or(lever);
      const Eigen::Vector2d velocity =
          AntennaPosition(State(0.0, 0.0, heading), Eigen::Vector2d(speed - turn * at(1), turn * at(0)));
      past_.push_back({t, AntennaPosition(Truth(), at), velocity});
      while (past_.size() > 200) past_.pop_front();
      if (++steps_ % 10 == 0 && gnss) Fix();
    }
    return moved;
  }

  // Somebody puts the mower down elsewhere: the sensors notice nothing.
  void Place(double nx, double ny, double nheading) {
    x = nx;
    y = ny;
    heading = nheading;
    past_.clear();
  }

  State Truth() const { return State(x, y, heading); }
  double HeadingError() const { return WrapAngle(est.pose().heading - heading); }
  double PositionError() const { return std::hypot(est.pose().x - x, est.pose().y - y); }
  // Of the estimate along the heading: positive ahead of the truth.
  double AlongError() const {
    return (est.pose().x - x) * std::cos(heading) + (est.pose().y - y) * std::sin(heading);
  }
  FixResult last() const { return results.empty() ? FixResult::kNotReady : results.back(); }

  Estimator est;
  Eigen::Vector2d lever;
  double t = 100.0;
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double gyro_bias = 0.0;
  double odom_scale = 1.0;
  bool gnss = true;
  double gnss_delay = 0.0;
  double accuracy = 0.02;
  Eigen::Vector2d fix_offset = Eigen::Vector2d::Zero();
  // The receiver reports its velocity turned by this (rad), at the right speed.
  double course_error = 0.0;
  // The velocity is the chord of the antenna over this many s up to the fix (0: momentary), its window reported or not.
  double chord = 0.0;
  bool report_window = true;
  // The horizontal lever where the antenna really is (frame of base_link), if not lever; the tilt the IMU reports.
  std::optional<Eigen::Vector2d> true_lever;
  std::optional<Tilt> tilt;
  std::vector<FixResult> results;

 private:
  struct Past {
    double t;
    Eigen::Vector2d antenna;
    Eigen::Vector2d velocity;
  };

  void Fix() {
    // The newest moment at least gnss_delay old.
    const Past* then = &past_.back();
    for (auto it = past_.rbegin(); it != past_.rend(); ++it) {
      then = &*it;
      if (it->t <= t - gnss_delay + 1e-9) break;
    }
    GnssFix fix;
    fix.solution = kSolutionRtkFixed;
    fix.accuracy = accuracy;
    fix.x = then->antenna(0) + fix_offset(0);
    fix.y = then->antenna(1) + fix_offset(1);
    fix.motion_valid = true;
    Eigen::Vector2d velocity = then->velocity;
    if (chord > 0.0) {
      const Past* from = then;
      for (const Past& p : past_) {
        if (p.t >= then->t - chord - 1e-9) {
          from = &p;
          break;
        }
      }
      if (from->t >= then->t) return;
      velocity = (then->antenna - from->antenna) / (then->t - from->t);
      if (report_window) fix.motion_window = then->t - from->t;
    }
    fix.motion_x = std::cos(course_error) * velocity(0) - std::sin(course_error) * velocity(1);
    fix.motion_y = std::sin(course_error) * velocity(0) + std::cos(course_error) * velocity(1);
    results.push_back(est.AddFix(fix, t));
  }

  long steps_ = 0;
  std::deque<Past> past_;
};

// L1: nothing is integrated before the gyro offset is known.
TEST(L1GyroCalibration, NothingMovesBeforeTheOffsetIsKnown) {
  World w;
  w.Geometry();
  w.gyro_bias = 0.01;
  EXPECT_EQ(w.Run(4.9, 0.0, 0.0), 0);
  EXPECT_FALSE(w.est.calibrated());
  EXPECT_EQ(w.last(), FixResult::kNotReady);
  w.Run(0.2, 0.0, 0.0);
  ASSERT_TRUE(w.est.calibrated());
  EXPECT_NEAR(w.est.gyro_offset(), 0.01, 1e-9);
  EXPECT_GT(w.Run(10.0, 0.0, 0.0), 490);
  EXPECT_NEAR(w.est.pose().heading, 0.0, 1e-9);
}

TEST(L1GyroCalibration, TurningWheelsStartItAgain) {
  World w;
  w.Geometry();
  w.Run(3.0, 0.2, 0.0);
  w.Run(4.9, 0.0, 0.0);
  EXPECT_FALSE(w.est.calibrated());
  w.Run(0.2, 0.0, 0.0);
  EXPECT_TRUE(w.est.calibrated());
}

TEST(L1GyroCalibration, ASingleNoiseSampleEndsItWithoutConfirmTime) {
  World w;
  w.Geometry();
  w.Run(3.0, 0.0, 0.0);
  w.Run(0.02, 0.05, 0.0);
  w.Run(2.2, 0.0, 0.0);
  EXPECT_FALSE(w.est.calibrated());
}

TEST(L1GyroCalibration, ASingleNoiseSampleDoesNotEndItWithConfirmTime) {
  EstimatorConfig config;
  config.motion_confirm_time = 0.1;
  World w(config);
  w.Geometry();
  w.Run(3.0, 0.0, 0.0);
  w.Run(0.02, 0.05, 0.0);
  w.Run(2.2, 0.0, 0.0);
  EXPECT_TRUE(w.est.calibrated());
}

TEST(L1GyroCalibration, MotionLongerThanTheConfirmTimeStartsItAgain) {
  EstimatorConfig config;
  config.motion_confirm_time = 0.1;
  World w(config);
  w.Geometry();
  w.Run(3.0, 0.0, 0.0);
  w.Run(0.2, 0.2, 0.0);
  w.Run(4.9, 0.0, 0.0);
  EXPECT_FALSE(w.est.calibrated());
  w.Run(0.2, 0.0, 0.0);
  EXPECT_TRUE(w.est.calibrated());
}

TEST(L1GyroCalibration, SkippedItTakesTheOffsetAndTheFirstSampleOnlyStarts) {
  EstimatorConfig config;
  config.gyro.skip_calibration = true;
  config.gyro.initial_offset = 0.01;
  Estimator est(config);
  double t = 1000.0;
  EXPECT_FALSE(est.AddImu(t, 0.11));
  EXPECT_TRUE(est.calibrated());
  int moved = 0;
  for (int i = 0; i < 50; ++i) moved += est.AddImu(t += kStep, 0.11);
  EXPECT_EQ(moved, 50);
  EXPECT_NEAR(est.pose().heading, 0.1, 1e-9);
}

// L1: without wheel reports nobody knows the mower stands, so the gyro offset is not measured and nothing moves.
TEST(L1GyroCalibration, NoWheelReportsNoCalibration) {
  Estimator est({});
  double t = 100.0;
  for (int i = 0; i < 1000; ++i) {
    t += kStep;
    EXPECT_FALSE(est.AddImu(t, 0.001));
  }
  EXPECT_FALSE(est.calibrated());
  // Wheel reports that stand, from now on: 5 s later the offset is known.
  for (int i = 0; i < 260; ++i) {
    t += kStep;
    est.AddWheels(t, 0.0, 0.0);
    est.AddImu(t, 0.001);
  }
  EXPECT_TRUE(est.calibrated());
  EXPECT_NEAR(est.gyro_offset(), 0.001, 1e-9);
}

// L2: each gyro sample moves the estimate by the wheel speed along the heading.
TEST(L2Prediction, StraightAndAQuarterCircle) {
  World w;
  w.Geometry();
  w.Calibrate();
  w.gnss = false;
  w.Run(5.0, 0.2, 0.0);
  EXPECT_NEAR(w.est.pose().x, w.x, 1e-9);
  EXPECT_NEAR(w.est.pose().y, 0.0, 1e-9);
  EXPECT_DOUBLE_EQ(w.est.speed(), 0.2);
  w.Run(5.0, 0.2, M_PI / 10);
  EXPECT_NEAR(w.est.pose().heading, M_PI / 2, 1e-9);
  EXPECT_NEAR(w.est.pose().x, w.x, 1e-9);
  EXPECT_NEAR(w.est.pose().y, w.y, 1e-9);
  EXPECT_NEAR(w.est.turn_rate(), M_PI / 10, 1e-12);
}

TEST(L2Prediction, TheWheelScaleOfTheGeometryCorrectsTheDriversSpeed) {
  // The driver computes with 1000 ticks per m, the wheels really make 940: its speeds are 1000/940 too slow.
  World w;
  w.Geometry();
  w.Calibrate();
  w.gnss = false;
  w.odom_scale = 0.94;
  w.est.SetWheelScale(1000.0, 940.0);
  EXPECT_NEAR(w.est.wheel_scale(), 1000.0 / 940.0, 1e-12);
  w.Run(5.0, 0.2, 0.0);
  EXPECT_NEAR(w.est.pose().x, w.x, 1e-9);
  EXPECT_NEAR(w.est.speed(), 0.2, 1e-12);
  // Unknown or nonsense values count as 1.
  const std::vector<std::pair<double, double>> wrong = {{0.0, 940.0}, {1000.0, 0.0}, {-1.0, 940.0}, {NAN, 940.0}};
  for (const auto& [driver, geometry] : wrong) {
    w.est.SetWheelScale(driver, geometry);
    EXPECT_DOUBLE_EQ(w.est.wheel_scale(), 1.0);
  }
}

TEST(L2Prediction, AStaleWheelSpeedCountsAsZero) {
  Estimator est({});
  double t = 100.0;
  for (int i = 0; i < 300; ++i) {
    est.AddWheels(t += kStep, 0.0, 0.0);
    est.AddImu(t, 0.0);
  }
  ASSERT_TRUE(est.calibrated());
  est.AddWheels(t, 0.3, 0.0);
  for (int i = 0; i < 100; ++i) est.AddImu(t += kStep, 0.0);
  // 0.5 s at 0.3 m/s, then nothing.
  EXPECT_NEAR(est.pose().x, 0.15, 0.3 * kStep + 1e-9);
}

TEST(L2Prediction, GapsNanAndTimeGoingBackAreNotIntegrated) {
  Estimator est({});
  double t = 100.0;
  // Standing wheels for the calibration only, so that nothing is held afterwards.
  for (int i = 0; i < 300; ++i) {
    if (i < 260) est.AddWheels(t + kStep, 0.0, 0.0);
    est.AddImu(t += kStep, 0.0);
  }
  ASSERT_TRUE(est.calibrated());
  t += 0.6;
  EXPECT_FALSE(est.AddImu(t, 1.0));
  EXPECT_FALSE(est.AddImu(t + kStep, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(est.AddImu(t - 1.0, 1.0));
  EXPECT_DOUBLE_EQ(est.pose().heading, 0.0);
  EXPECT_TRUE(est.AddImu(t - 1.0 + kStep, 1.0));
  EXPECT_NEAR(est.pose().heading, kStep, 1e-12);
  est.AddWheels(t, std::numeric_limits<double>::quiet_NaN(), 0.0);
  EXPECT_TRUE(est.AddImu(t - 1.0 + 2 * kStep, 1.0));
  EXPECT_DOUBLE_EQ(est.pose().x, 0.0);
}

TEST(L2Prediction, TheHeadingStaysWrapped) {
  World w;
  w.Geometry();
  w.Calibrate();
  w.Run(10.0, 0.0, 1.0);
  EXPECT_GT(w.est.pose().heading, -M_PI);
  EXPECT_LE(w.est.pose().heading, M_PI);
  EXPECT_NEAR(w.HeadingError(), 0.0, 1e-9);
}

// L3: only RTK fixed fixes within the limit count, with GNSS on and the antenna offset known.
TEST(L3FixesThatCount, NeedCalibrationGeometryAndGnssOn) {
  World w;
  w.Calibrate();
  w.Run(0.2, 0.0, 0.0);
  EXPECT_EQ(w.last(), FixResult::kNoGeometry);
  w.Geometry();
  w.est.SetGpsEnabled(false);
  w.Run(0.2, 0.0, 0.0);
  EXPECT_EQ(w.last(), FixResult::kDisabled);
  EXPECT_FALSE(w.est.RecentGps(w.t));
  w.est.SetGpsEnabled(true);
  w.Run(0.2, 0.0, 0.0);
  EXPECT_EQ(w.last(), FixResult::kRestarted);
  w.accuracy = 0.15;
  w.Run(0.2, 0.0, 0.0);
  EXPECT_EQ(w.last(), FixResult::kInaccurate);
  GnssFix fix;
  fix.solution = kSolutionRtkFloat;
  fix.accuracy = 0.02;
  EXPECT_EQ(w.est.AddFix(fix, w.t), FixResult::kNotFixed);
}

// L4: a warmup of the first fix and ten more, then GNSS sets the estimate.
TEST(L4Warmup, ThenGnssSetsTheEstimate) {
  World w({}, Eigen::Vector2d(0.3, 0.0));
  w.x = 10.0;
  w.y = 20.0;
  w.Geometry();
  // The fix right after the calibration starts the warmup.
  w.Calibrate();
  EXPECT_EQ(w.last(), FixResult::kRestarted);
  w.Run(2.0, 0.0, 0.0);
  EXPECT_EQ(w.last(), FixResult::kWarmup);
  EXPECT_EQ(w.est.PositionAccuracy(w.t), kNoAccuracy);
  EXPECT_TRUE(w.est.RecentGps(w.t));
  EXPECT_NEAR(w.est.pose().x, 0.0, 1e-9);
  w.Run(0.2, 0.0, 0.0);
  EXPECT_EQ(w.last(), FixResult::kInitialized);
  // base_link behind the antenna, by the heading of the estimate (east, as the truth).
  EXPECT_NEAR(w.est.pose().x, 10.0, 1e-9);
  EXPECT_NEAR(w.est.pose().y, 20.0, 1e-9);
  EXPECT_NEAR(w.est.PositionAccuracy(w.t), 0.02, 1e-9);
  EXPECT_FALSE(w.est.TakeJump());
  w.Run(0.2, 0.0, 0.0);
  EXPECT_EQ(w.last(), FixResult::kFused);
}

TEST(L4Warmup, FiveSecondsWithoutAFixStartIt) {
  World w;
  w.Ready();
  w.gnss = false;
  w.Run(5.0, 0.0, 0.0);
  w.gnss = true;
  w.Run(0.2, 0.0, 0.0);
  EXPECT_EQ(w.last(), FixResult::kRestarted);
  EXPECT_FALSE(w.est.gps_valid());
  EXPECT_EQ(w.est.PositionAccuracy(w.t), kNoAccuracy);
}

// L5 and L6: outliers are dropped; the eleventh in a row starts a warmup that ends in a jump.
TEST(L5Outliers, AreDroppedUntilTheEleventhAndThenAJump) {
  World w;
  w.x = 10.0;
  w.y = 20.0;
  w.Ready();
  const Pose2 before = w.est.pose();
  w.fix_offset = Eigen::Vector2d(6.0, 0.0);
  w.Run(2.0, 0.0, 0.0);
  EXPECT_EQ(w.last(), FixResult::kOutlier);
  EXPECT_DOUBLE_EQ(w.est.pose().x, before.x);
  EXPECT_TRUE(w.est.gps_valid());
  w.Run(0.2, 0.0, 0.0);
  EXPECT_EQ(w.last(), FixResult::kOutlierReset);
  EXPECT_FALSE(w.est.gps_valid());
  EXPECT_FALSE(w.est.TakeJump());
  w.Run(2.2, 0.0, 0.0);
  EXPECT_EQ(w.last(), FixResult::kInitialized);
  const auto jump = w.est.TakeJump();
  ASSERT_TRUE(jump);
  EXPECT_EQ(jump->reason, WarmupReason::kOutliers);
  EXPECT_NEAR(jump->from_x, 10.0, 1e-6);
  EXPECT_NEAR(jump->to_x, 16.0, 1e-6);
  EXPECT_NEAR(jump->distance(), 6.0, 1e-6);
  EXPECT_FALSE(w.est.TakeJump());
}

// L5: carried by hand less than an outlier's distance while the heading is valid: the fixes contradict the
// covariance, the fifth in a row sets the position onto GNSS as a jump.
TEST(L5Outliers, ContradictingFixesSetThePositionAfterFiveInARow) {
  World w;
  w.Aligned();
  w.Run(20.0, 0.0, 0.0);
  const Pose2 before = w.est.pose();
  w.Place(w.x - 2.0, w.y + 0.5, w.heading);
  w.Run(0.8, 0.0, 0.0);
  EXPECT_EQ(w.last(), FixResult::kContradicting);
  EXPECT_DOUBLE_EQ(w.est.pose().x, before.x);
  EXPECT_FALSE(w.est.TakeJump());
  w.Run(0.2, 0.0, 0.0);
  EXPECT_EQ(w.last(), FixResult::kContradictionReset);
  EXPECT_LT(w.PositionError(), 0.03);
  EXPECT_TRUE(w.est.gps_valid());
  EXPECT_TRUE(w.est.heading_valid());
  // The fix, and the loosened heading over the lever of 0.1 m.
  EXPECT_NEAR(w.est.PositionAccuracy(w.t), 0.02 + EstimatorConfig().disturbed_heading_sigma * 0.1, 1e-9);
  // Without that part, for the reference check on a station that sets the heading.
  EXPECT_NEAR(w.est.PositionAccuracyAtHeading(w.t), 0.02, 1e-9);
  const auto jump = w.est.TakeJump();
  ASSERT_TRUE(jump);
  EXPECT_EQ(jump->reason, WarmupReason::kOutliers);
  EXPECT_NEAR(jump->distance(), std::hypot(2.0, 0.5), 0.03);
  w.Run(1.0, 0.0, 0.0);
  EXPECT_EQ(w.last(), FixResult::kFused);
}

// L5: a single fix 0.5 m off is dropped, and fixes within max_position_accuracy are fused whatever the covariance.
TEST(L5Outliers, SingleContradictionsAndSmallOnesAreNoJump) {
  World w;
  w.Aligned();
  w.Run(20.0, 0.0, 0.0);
  const Pose2 before = w.est.pose();
  w.fix_offset = Eigen::Vector2d(0.5, 0.0);
  w.Run(0.2, 0.0, 0.0);
  EXPECT_EQ(w.last(), FixResult::kContradicting);
  w.fix_offset = Eigen::Vector2d::Zero();
  w.Run(0.2, 0.0, 0.0);
  EXPECT_EQ(w.last(), FixResult::kFused);
  for (int i = 0; i < 4; i++) {
    w.fix_offset = Eigen::Vector2d(0.5, 0.0);
    w.Run(0.2 * 4, 0.0, 0.0);
    w.fix_offset = Eigen::Vector2d::Zero();
    w.Run(0.2, 0.0, 0.0);
  }
  EXPECT_FALSE(w.est.TakeJump());
  EXPECT_NEAR(w.est.pose().x, before.x, 0.01);
  w.fix_offset = Eigen::Vector2d(0.08, 0.0);
  w.Run(2.0, 0.0, 0.0);
  EXPECT_EQ(w.last(), FixResult::kFused);
  EXPECT_FALSE(w.est.TakeJump());
}

TEST(L6Jumps, AfterAGapOnlyIfTheEstimateMoved) {
  World w;
  w.Ready();
  w.gnss = false;
  w.Run(6.0, 0.0, 0.0);
  w.gnss = true;
  w.Run(2.4, 0.0, 0.0);
  EXPECT_EQ(w.last(), FixResult::kInitialized);
  EXPECT_FALSE(w.est.TakeJump());
  // The wheels report half the way: 1 m driven, 0.5 m counted.
  w.gnss = false;
  w.odom_scale = 0.5;
  w.Run(6.0, 1.0 / 6.0, 0.0);
  w.odom_scale = 1.0;
  w.gnss = true;
  w.Run(2.4, 0.0, 0.0);
  const auto jump = w.est.TakeJump();
  ASSERT_TRUE(jump);
  EXPECT_EQ(jump->reason, WarmupReason::kGap);
  EXPECT_NEAR(jump->distance(), 0.5, 0.01);
}

// L7: the accuracy of the last fix grows with the way and the time and expires after gps_timeout. The antenna on
// base_link and a heading: the spread of the estimate stays below it.
TEST(L7Accuracy, GrowsWithWayAndTime) {
  World w({}, Eigen::Vector2d::Zero());
  w.Aligned();
  EXPECT_NEAR(w.est.PositionAccuracy(w.t), 0.02, 1e-9);
  w.gnss = false;
  w.Run(2.0, 0.0, 0.0);
  EXPECT_NEAR(w.est.PositionAccuracy(w.t), 0.02 + 0.02, 1e-9);
  w.Run(2.0, 0.3, 0.0);
  EXPECT_NEAR(w.est.PositionAccuracy(w.t), 0.02 + 0.04 + 0.03, 1e-9);
  // Backwards counts the same.
  w.Run(0.6, -0.3, 0.0);
  EXPECT_NEAR(w.est.PositionAccuracy(w.t), 0.02 + 0.046 + 0.039, 1e-9);
  w.gnss = true;
  w.Run(0.2, 0.0, 0.0);
  EXPECT_EQ(w.last(), FixResult::kFused);
  EXPECT_NEAR(w.est.PositionAccuracy(w.t), 0.02, 1e-9);
}

TEST(L7Accuracy, ExpiresAndTheLimitsOfTheLogicReplaceTheDefaults) {
  World w;
  w.Ready();
  const double last = w.t;
  EXPECT_LT(w.est.PositionAccuracy(last + 9.9), 1.0);
  EXPECT_TRUE(w.est.RecentGps(last + 9.9));
  EXPECT_EQ(w.est.PositionAccuracy(last + 10.0), kNoAccuracy);
  EXPECT_FALSE(w.est.RecentGps(last + 10.0));
  w.est.SetGpsLimits(0.1, 3.0);
  EXPECT_EQ(w.est.PositionAccuracy(last + 3.0), kNoAccuracy);
  w.est.SetGpsLimits(0.01, 3.0);
  w.Run(0.2, 0.0, 0.0);
  EXPECT_EQ(w.last(), FixResult::kInaccurate);
}

// L8: a valid heading needs a course, then stays.
TEST(L8Heading, NotValidStandingOrCreeping) {
  World w;
  w.Ready();
  w.Run(10.0, 0.0, 0.0);
  EXPECT_FALSE(w.est.heading_valid());
  w.Run(10.0, 0.05, 0.0);
  EXPECT_FALSE(w.est.heading_valid());
  EXPECT_EQ(w.est.HeadingAccuracy(), kNoAccuracy);
}

TEST(L8Heading, PausesDoNotBreakTheRun) {
  World w;
  w.Ready();
  // Short pushes of the joystick with stops between them, as in the app; each counts from 0.1 m of track on.
  w.Run(0.6, 0.3, 0.0);
  w.Run(2.0, 0.0, 0.0);
  w.Run(0.6, 0.3, 0.1);
  EXPECT_FALSE(w.est.heading_valid());
  w.Run(1.0, 0.0, 0.0);
  w.Run(0.4, 0.3, 0.0);
  EXPECT_TRUE(w.est.heading_valid());
  EXPECT_LT(std::abs(w.HeadingError()), 0.5 * kDeg);
}

TEST(L8Heading, AnInconsistentCourseBreaksTheRun) {
  World w;
  w.Ready();
  w.Run(0.8, 0.3, 0.0);
  // The wheels report three times the speed the antenna moves at: the four courses before do not count either.
  w.odom_scale = 3.0;
  w.Run(0.2, 0.3, 0.0);
  w.odom_scale = 1.0;
  w.Run(0.8, 0.3, 0.0);
  EXPECT_FALSE(w.est.heading_valid());
  w.Run(0.2, 0.3, 0.0);
  EXPECT_TRUE(w.est.heading_valid());
}

TEST(L8Heading, ValidWithTheFifthAgreeingCourseAndThenStays) {
  World w;
  w.Ready();
  // The first fix after the start has no track of 0.1 m yet.
  w.Run(1.0, 0.3, 0.0);
  EXPECT_FALSE(w.est.heading_valid());
  w.Run(0.2, 0.3, 0.0);
  EXPECT_TRUE(w.est.heading_valid());
  EXPECT_TRUE(w.est.TakeAligned());
  EXPECT_FALSE(w.est.TakeAligned());
  w.gnss = false;
  w.Run(30.0, 0.0, 0.0);
  EXPECT_TRUE(w.est.heading_valid());
}

// L9: courses correct the heading; slow ones do nothing.
TEST(L9Courses, CorrectTheHeadingOnly) {
  World w;
  w.Aligned();
  // A gyro off by 0.5 deg/s for 10 s without GNSS: about 5 degrees.
  w.gnss = false;
  w.gyro_bias = 0.5 * kDeg;
  w.Run(10.0, 0.3, 0.0);
  w.gyro_bias = 0.0;
  EXPECT_GT(std::abs(w.HeadingError()), 3.0 * kDeg);
  w.gnss = true;
  w.Run(10.0, 0.3, 0.0);
  EXPECT_LT(std::abs(w.HeadingError()), 0.5 * kDeg);
  EXPECT_LT(w.PositionError(), 0.02);
}

// Without a valid heading its part is left out (the station, the undock line); a valid one adds its spread over the
// lever.
TEST(L7Accuracy, TheHeadingCountsOnlyWhenValid) {
  World w;
  w.Ready();
  EXPECT_FALSE(w.est.heading_valid());
  EXPECT_NEAR(w.est.PositionAccuracy(w.t), 0.02, 1e-9);
  EXPECT_NEAR(w.est.PositionAccuracyAtHeading(w.t), 0.02, 1e-9);
  World aligned;
  aligned.Aligned();
  EXPECT_TRUE(aligned.est.heading_valid());
  EXPECT_LT(aligned.est.PositionAccuracy(aligned.t), 0.02 + 2 * 0.1);
}

// R3-10: a chord of positions is one course per window, and it belongs to the middle of its window.
TEST(CourseWindow, OverlappingWindowsCountOnce) {
  World w;
  w.Ready();
  w.chord = 1.0;
  w.Run(2.5, 0.3, 0.0);
  EXPECT_FALSE(w.est.heading_valid());
  w.Run(4.0, 0.3, 0.0);
  EXPECT_TRUE(w.est.heading_valid());
}

TEST(CourseWindow, InATurnTheChordCountsAtTheMiddleOfItsWindow) {
  World w;
  w.Ready();
  w.chord = 1.0;
  w.Run(7.0, 0.3, 0.04);
  ASSERT_TRUE(w.est.heading_valid());
  w.Run(4.0, 0.3, 0.04);
  EXPECT_LT(std::abs(w.HeadingError()), 0.008) << w.HeadingError();
  // Taken for the moment of the fix it lags by half its window times the turn rate.
  World lag;
  lag.Ready();
  lag.chord = 1.0;
  lag.report_window = false;
  lag.Run(11.0, 0.3, 0.04);
  ASSERT_TRUE(lag.est.heading_valid());
  EXPECT_GT(std::abs(lag.HeadingError()), 0.012) << lag.HeadingError();
}

// L10: the antenna offset moves base_link behind the last fix.
TEST(L10AntennaOffset, AChangeMovesBaseLinkBehindTheLastFix) {
  World w({}, Eigen::Vector2d(0.3, 0.0));
  w.x = 10.0;
  w.y = 5.0;
  w.Ready();
  w.est.SetAntennaOffset(0.1, 0.05, w.t);
  EXPECT_NEAR(w.est.pose().x, 10.2, 1e-9);
  EXPECT_NEAR(w.est.pose().y, 4.95, 1e-9);
  EXPECT_DOUBLE_EQ(w.est.pose().heading, 0.0);
  EXPECT_FALSE(w.est.heading_valid());
  EXPECT_NEAR(w.est.PositionAccuracy(w.t), 0.02, 1e-9);
}

TEST(L10AntennaOffset, WithAValidHeadingTheOffsetTurns) {
  World w({}, Eigen::Vector2d(0.3, 0.0));
  w.x = 10.0;
  w.y = 5.0;
  w.Ready();
  ASSERT_TRUE(w.est.SetHeading(M_PI / 2, w.t));
  w.est.SetAntennaOffset(0.2, -0.1, w.t);
  // Facing north the antenna is 0.2 m north and 0.1 m east of base_link; the last fix was at (10.3, 5).
  EXPECT_NEAR(w.est.pose().x, 10.2, 1e-9);
  EXPECT_NEAR(w.est.pose().y, 4.8, 1e-9);
  EXPECT_NEAR(w.est.pose().heading, M_PI / 2, 1e-12);
  EXPECT_TRUE(w.est.heading_valid());
}

TEST(L10AntennaOffset, WithoutARecentFixOnlyTheOffsetChanges) {
  World w({}, Eigen::Vector2d(0.3, 0.0));
  w.Ready();
  w.est.SetGpsLimits(0.1, 2.0);
  w.gnss = false;
  w.Run(2.4, 0.0, 0.0);
  const Pose2 before = w.est.pose();
  w.est.SetAntennaOffset(0.5, 0.0, w.t);
  EXPECT_DOUBLE_EQ(w.est.pose().x, before.x);
  EXPECT_DOUBLE_EQ(w.est.pose().y, before.y);
  EXPECT_DOUBLE_EQ(w.est.antenna_offset()(0), 0.5);
}

TEST(L10AntennaOffset, TheNewOffsetCountsForTheFixesThatFollow) {
  World w({}, Eigen::Vector2d(0.3, 0.0));
  w.x = 10.0;
  w.y = 5.0;
  w.Ready();
  // The estimate is told the antenna sits on base_link: the fixes at (10.3, 5) put base_link there and keep it.
  w.est.SetAntennaOffset(0.0, 0.0, w.t);
  w.Run(6.0, 0.0, 0.0);
  EXPECT_NEAR(w.est.pose().x, 10.3, 1e-3);
  EXPECT_NEAR(w.est.pose().y, 5.0, 1e-3);
}

// L11: set_pose starts again at a pose with a valid heading.
TEST(L11SetPose, StartsAgainThere) {
  World w;
  w.Aligned();
  w.est.SetPose(Pose2{1.0, 2.0, w.heading});
  EXPECT_DOUBLE_EQ(w.est.pose().x, 1.0);
  EXPECT_DOUBLE_EQ(w.est.pose().y, 2.0);
  EXPECT_DOUBLE_EQ(w.est.pose().heading, w.heading);
  EXPECT_DOUBLE_EQ(w.est.speed(), 0.0);
  EXPECT_TRUE(w.est.heading_valid());
  EXPECT_DOUBLE_EQ(w.est.covariance()(0, 1), 0.0);
  EXPECT_FALSE(w.est.gps_valid());
  EXPECT_EQ(w.est.PositionAccuracy(w.t), kNoAccuracy);
  // GNSS sets the position again after a warmup, as a first one: no jump.
  w.Run(2.4, 0.0, 0.0);
  EXPECT_EQ(w.last(), FixResult::kInitialized);
  EXPECT_LT(w.PositionError(), 1e-6);
  EXPECT_FALSE(w.est.TakeJump());
}

// L12: set_heading needs a recent fix the estimate took.
TEST(L12SetHeading, NeedsAFixTheEstimateTook) {
  World w;
  w.Geometry();
  w.Calibrate();
  EXPECT_FALSE(w.est.SetHeading(1.0, w.t));
  w.Run(1.0, 0.0, 0.0);
  EXPECT_TRUE(w.est.RecentGps(w.t));
  EXPECT_FALSE(w.est.SetHeading(1.0, w.t));
  EXPECT_FALSE(w.est.heading_valid());
  EXPECT_DOUBLE_EQ(w.est.pose().heading, 0.0);
}

TEST(L12SetHeading, PutsBaseLinkBehindTheAntennaOfTheLastFix) {
  World w({}, Eigen::Vector2d(0.3, 0.1));
  w.x = 10.0;
  w.y = 5.0;
  w.Ready();
  const auto pose = w.est.SetHeading(M_PI / 2, w.t);
  ASSERT_TRUE(pose);
  // The last fix put the antenna at (10.3, 5.1); facing north it is 0.3 m north and 0.1 m west of base_link.
  EXPECT_NEAR(pose->x, 10.4, 1e-9);
  EXPECT_NEAR(pose->y, 4.8, 1e-9);
  EXPECT_NEAR(w.est.pose().x, 10.4, 1e-9);
  EXPECT_NEAR(w.est.pose().heading, M_PI / 2, 1e-12);
  EXPECT_TRUE(w.est.heading_valid());
  // The fix, and the heading set (0.02 rad) over the lever.
  EXPECT_NEAR(w.est.PositionAccuracy(w.t), 0.02 + 0.02 * std::hypot(0.3, 0.1), 1e-9);
  EXPECT_NEAR(w.est.HeadingAccuracy(), 0.02, 1e-9);
}

TEST(L12SetHeading, NotWithoutARecentFixOrWithNan) {
  World w;
  w.Ready();
  EXPECT_FALSE(w.est.SetHeading(std::numeric_limits<double>::quiet_NaN(), w.t));
  w.est.SetGpsLimits(0.1, 2.0);
  w.gnss = false;
  w.Run(2.4, 0.0, 0.0);
  EXPECT_FALSE(w.est.SetHeading(2.0, w.t));
  EXPECT_FALSE(w.est.heading_valid());
  EXPECT_DOUBLE_EQ(w.est.pose().heading, 0.0);
}

// E1: five agreeing courses set the heading from any start, in about a second of driving.
TEST(E1Alignment, FromAHeading90DegreesOff) {
  World w;
  w.heading = M_PI / 2;
  w.Ready();
  w.Run(1.0, 0.3, 0.0);
  EXPECT_FALSE(w.est.heading_valid());
  w.Run(0.2, 0.3, 0.0);
  ASSERT_TRUE(w.est.heading_valid());
  EXPECT_LT(std::abs(w.HeadingError()), 0.5 * kDeg);
  w.Run(2.0, 0.3, 0.0);
  EXPECT_LT(w.PositionError(), 0.01);
}

TEST(E1Alignment, BackwardsAndOnACurve) {
  World w({}, Eigen::Vector2d(0.2, 0.05));
  w.heading = -2.0;
  w.Ready();
  w.Run(1.4, -0.2, 0.0);
  ASSERT_TRUE(w.est.heading_valid());
  EXPECT_LT(std::abs(w.HeadingError()), 0.5 * kDeg);
  World curve({}, Eigen::Vector2d(0.2, 0.05));
  curve.heading = 2.5;
  curve.Ready();
  curve.Run(1.2, 0.3, 0.3);
  ASSERT_TRUE(curve.est.heading_valid());
  EXPECT_LT(std::abs(curve.HeadingError()), 0.5 * kDeg);
}

TEST(E1Alignment, AValidHeadingTheCoursesContradictIsSetFromThem) {
  World w;
  w.Aligned();
  w.est.TakeAligned();
  // Turned 90 degrees by hand: the gyro did not see it.
  w.Place(w.x, w.y, w.heading + M_PI / 2);
  w.Run(1.2, 0.3, 0.0);
  EXPECT_TRUE(w.est.TakeAligned());
  w.Run(1.0, 0.3, 0.0);
  EXPECT_LT(std::abs(w.HeadingError()), 0.5 * kDeg);
  EXPECT_TRUE(w.est.heading_valid());
}

TEST(E1Alignment, SmallDifferencesGoThroughTheFilter) {
  World w;
  w.Aligned();
  w.est.TakeAligned();
  w.Place(w.x, w.y, w.heading + 3.0 * kDeg);
  w.Run(10.0, 0.3, 0.0);
  EXPECT_FALSE(w.est.TakeAligned());
  EXPECT_LT(std::abs(w.HeadingError()), 1.0 * kDeg);
}

// E2: standing, the heading holds and the gyro offset follows a changed bias.
TEST(E2StandingStill, AChangedBiasDoesNotTurnTheHeading) {
  World w;
  w.Aligned();
  const double heading = w.est.pose().heading;
  w.Run(2.0, 0.0, 0.0);
  w.gyro_bias = 0.3 * kDeg;
  w.Run(60.0, 0.0, 0.0);
  EXPECT_TRUE(w.est.holding());
  EXPECT_LT(std::abs(WrapAngle(w.est.pose().heading - heading)), 0.01 * kDeg);
  EXPECT_NEAR(w.est.gyro_offset(), 0.3 * kDeg, 0.01 * kDeg);
  // Driving on without GNSS the new offset keeps the heading.
  w.gnss = false;
  w.Run(10.0, 0.3, 0.0);
  EXPECT_LT(std::abs(w.HeadingError()), 0.1 * kDeg);
}

TEST(E2StandingStill, ATurnByHandIsIntegrated) {
  World w;
  w.Aligned();
  w.gnss = false;
  w.Run(2.0, 0.0, 0.0);
  // The wheels stand while somebody turns the mower at 10 deg/s.
  for (int i = 0; i < 300; ++i) {
    w.t += kStep;
    w.heading = WrapAngle(w.heading + 10.0 * kDeg * kStep);
    w.est.AddWheels(w.t, 0.0, 0.0);
    w.est.AddImu(w.t, 10.0 * kDeg);
  }
  EXPECT_LT(std::abs(w.HeadingError()), 0.1 * kDeg);
}

TEST(E2StandingStill, FixesBesideTheEstimateDoNotTurnALooseHeading) {
  World w;
  w.Aligned();
  w.Run(2.0, 0.0, 0.0);
  w.est.LoosenHeading();
  const double heading = w.est.pose().heading;
  // The fixes drift 5 cm to the side while the mower stands.
  w.fix_offset = Eigen::Vector2d(0.0, 0.05);
  w.Run(5.0, 0.0, 0.0);
  EXPECT_EQ(w.est.pose().heading, heading);
  EXPECT_NEAR(w.est.HeadingAccuracy(), 20.0 * kDeg, 0.5 * kDeg);
  // Driving on, the fixes and courses correct it.
  w.fix_offset = Eigen::Vector2d::Zero();
  w.Run(2.0, 0.3, 0.0);
  EXPECT_LT(std::abs(w.HeadingError()), 1.0 * kDeg);
  EXPECT_LT(w.est.HeadingAccuracy(), 5.0 * kDeg);
}

TEST(E2StandingStill, AfterATurnOnTheSpotWithAWrongOffsetTheHeadingHolds) {
  // The antenna 10 cm ahead where the geometry says 2 cm: the turn throws the position, which loosens the heading.
  World w({}, Eigen::Vector2d(0.02, 0.0));
  w.true_lever = Eigen::Vector2d(0.1, 0.0);
  w.Aligned();
  w.Run(3.0, 0.3, 0.0);
  w.Run(2.0, 0.0, 0.0);
  w.Run(3.0 * M_PI / 0.3, 0.0, -0.3);
  EXPECT_GT(w.est.HeadingAccuracy(), 10.0 * kDeg);
  w.Run(5.0, 0.0, 0.0);
  EXPECT_LT(std::abs(w.HeadingError()), 1.0 * kDeg);
}

// E3: the heading accuracy comes from the filter.
TEST(E3HeadingAccuracy, FromTheFilter) {
  World w;
  w.Ready();
  EXPECT_EQ(w.est.HeadingAccuracy(), kNoAccuracy);
  w.Run(1.2, 0.3, 0.0);
  ASSERT_TRUE(w.est.heading_valid());
  // Five courses of 0.3 m/s with 0.02 m/s noise.
  const double aligned = w.est.HeadingAccuracy();
  EXPECT_NEAR(aligned, 0.02 / 0.3 / std::sqrt(5.0), 0.005);
  // Standing it does not grow, driving without GNSS it does.
  w.gnss = false;
  w.Run(30.0, 0.0, 0.0);
  EXPECT_NEAR(w.est.HeadingAccuracy(), aligned, 0.002);
  w.Run(100.0, 0.3, 0.0);
  EXPECT_GT(w.est.HeadingAccuracy(), aligned + 0.01);
}

// E4: with gnss_latency late fixes are compared with the estimate of their moment.
TEST(E4GnssLatency, LateFixesDoNotHoldTheEstimateBack) {
  World late;
  late.gnss_delay = 0.2;
  late.Aligned();
  late.Run(5.0, 0.3, 0.0);
  EXPECT_LT(late.AlongError(), -0.01);
  EstimatorConfig config;
  config.gnss_latency = 0.2;
  World known(config);
  known.gnss_delay = 0.2;
  known.Aligned();
  known.Run(5.0, 0.3, 0.0);
  EXPECT_LT(std::abs(known.AlongError()), 0.003);
  EXPECT_LT(std::abs(known.HeadingError()), 0.2 * kDeg);
}

// E4: a course that sets the heading turns the way of the latency with it, which went along the heading before.
TEST(E4GnssLatency, AHeadingSetByTheCourseTurnsTheWaySinceTheFix) {
  EstimatorConfig config;
  config.gnss_latency = 0.2;
  World w(config);
  w.gnss_delay = 0.2;
  w.heading = 90.0 * kDeg;
  w.Ready();
  ASSERT_FALSE(w.est.heading_valid());
  for (int i = 0; i < 20 && !w.est.heading_valid(); ++i) w.Run(0.2, 0.5, 0.0);
  ASSERT_TRUE(w.est.heading_valid());
  EXPECT_LT(std::abs(w.HeadingError()), 0.5 * kDeg);
  // Without the turn the 0.1 m of the latency lie along the heading of before, 0.14 m off.
  EXPECT_LT(w.PositionError(), 0.01);
}

// E4: fixes later than gnss_latency say do not hold a turn back, on the spot or on an arc, with the wheels reporting
// too little (the docking rate of 02.11.: about 2 degrees after the arcs and the turn in front of the station).
TEST(E4GnssLatency, LateFixesDoNotHoldTurnsBack) {
  for (const double turn : {0.25, -0.25}) {
    World w;
    w.gnss_delay = 0.1;
    w.odom_scale = 0.94;
    w.Aligned();
    w.Run(2.0, 0.3, 0.0);
    ASSERT_LT(std::abs(w.HeadingError()), 0.3 * kDeg);
    w.Run(12.6, 0.11, turn);
    EXPECT_LT(std::abs(w.HeadingError()), 0.2 * kDeg) << turn;
    w.Run(12.6, 0.0, turn);
    w.Run(0.4, 0.0, 0.0);
    EXPECT_LT(std::abs(w.HeadingError()), 0.2 * kDeg) << turn;
  }
}

// The covariance fits the errors: noise as the filter assumes it, the normalized error squared averages about 3.
TEST(Consistency, CovarianceMatchesTheErrorsOfRandomRuns) {
  std::mt19937 rng(7);
  std::normal_distribution<double> normal(0.0, 1.0);
  const EstimatorConfig config;
  const Eigen::Vector2d lever(0.1, 0.0);
  double sum = 0.0;
  int samples = 0;
  for (int run = 0; run < 20; ++run) {
    Estimator est(config);
    est.SetAntennaOffset(lever(0), lever(1), 0.0);
    double t = 0.0;
    State truth(0.0, 0.0, normal(rng));
    for (int step = 0; step < 4000; ++step) {
      t += kStep;
      const bool driving = step > 400;
      const double speed = driving ? 0.3 : 0.0;
      const double turn = driving ? 0.2 * std::sin(0.05 * t + run) : 0.0;
      const double distance = speed * kStep;
      const double mid = truth(2) + 0.5 * turn * kStep;
      // The ground slips under the wheels and the gyro drifts, both as the random walks of the filter.
      const double slip = config.noise.odometry * std::sqrt(distance);
      truth(0) += distance * std::cos(mid) + slip * normal(rng);
      truth(1) += distance * std::sin(mid) + slip * normal(rng);
      truth(2) = WrapAngle(truth(2) + turn * kStep);
      est.AddWheels(t, speed, turn);
      const double gyro = turn + (driving ? config.noise.gyro / std::sqrt(kStep) * normal(rng) : 0.0);
      est.AddImu(t, gyro);
      if (step % 10 != 0) continue;
      const Eigen::Vector2d antenna = AntennaPosition(truth, lever);
      const Eigen::Vector2d velocity =
          AntennaPosition(State(0.0, 0.0, truth(2)), Eigen::Vector2d(speed - turn * lever(1), turn * lever(0)));
      GnssFix fix;
      fix.solution = kSolutionRtkFixed;
      fix.accuracy = 0.02;
      fix.x = antenna(0) + 0.02 * normal(rng);
      fix.y = antenna(1) + 0.02 * normal(rng);
      fix.motion_valid = true;
      fix.motion_x = velocity(0) + config.course.velocity_noise * normal(rng);
      fix.motion_y = velocity(1) + config.course.velocity_noise * normal(rng);
      est.AddFix(fix, t);
      if (!est.heading_valid() || step < 1000) continue;
      State error = State(est.pose().x, est.pose().y, est.pose().heading) - truth;
      error(2) = WrapAngle(error(2));
      sum += error.dot(est.covariance().inverse() * error);
      samples++;
    }
  }
  ASSERT_GT(samples, 1000);
  const double mean = sum / samples;
  RecordProperty("mean_nees", std::to_string(mean));
  EXPECT_GT(mean, 2.0);
  EXPECT_LT(mean, 4.0);
}

// E5: courses against the track of the fixes (LG290P 0.9.50: 155-164 deg whatever the way, at the right speed).
TEST(E5CourseTrack, AWrongCourseNeverAlignsAnUnknownHeading) {
  World w;
  w.Ready();
  w.course_error = 160.0 * kDeg;
  w.Run(6.0, 0.3, 0.0);
  EXPECT_FALSE(w.est.heading_valid());
  EXPECT_GE(w.est.TakeCourseRejections(), 25);
  EXPECT_EQ(w.est.TakeCourseRejections(), 0);
  // The right course aligns as before once it comes.
  w.course_error = 0.0;
  w.Run(1.0, 0.3, 0.0);
  ASSERT_TRUE(w.est.heading_valid());
  EXPECT_LT(std::abs(w.HeadingError()), 0.5 * kDeg);
}

TEST(E5CourseTrack, AWrongCourseDoesNotResetAValidHeading) {
  World w;
  w.Aligned();
  w.est.TakeAligned();
  w.course_error = 160.0 * kDeg;
  w.Run(6.0, 0.3, 0.0);
  EXPECT_FALSE(w.est.TakeAligned());
  EXPECT_TRUE(w.est.heading_valid());
  EXPECT_LT(std::abs(w.HeadingError()), 0.5 * kDeg);
  EXPECT_LT(w.PositionError(), 0.02);
  EXPECT_GE(w.est.TakeCourseRejections(), 25);
}

TEST(E5CourseTrack, ASmallErrorStillAlignsOnACurve) {
  // 20 deg off on a curve of 0.5 rad/s: within the agreement, the run sets the heading.
  World w;
  w.Ready();
  w.course_error = 20.0 * kDeg;
  w.Run(1.2, 0.3, 0.5);
  EXPECT_TRUE(w.est.heading_valid());
  EXPECT_EQ(w.est.TakeCourseRejections(), 0);
}

// E6: after a jump of the position or a lift the heading is uncertain, so that the next course corrects it.
TEST(E6DisturbedHeading, CarriedWithATiltedGyroTheNextCoursesCorrectIt) {
  World w;
  w.Aligned();
  w.Run(2.0, 0.0, 0.0);
  EXPECT_LT(w.est.HeadingAccuracy(), 2.0 * kDeg);
  // Carried 1.2 m without GNSS; the gyro, tilted, missed 10 degrees of the turn.
  w.gnss = false;
  w.Run(6.0, 0.0, 0.0);
  w.Place(w.x + 1.2, w.y, w.heading + 10.0 * kDeg);
  w.gnss = true;
  w.Run(2.4, 0.0, 0.0);
  ASSERT_TRUE(w.est.TakeJump());
  EXPECT_TRUE(w.est.heading_valid());
  EXPECT_NEAR(w.est.HeadingAccuracy(), 20.0 * kDeg, 0.5 * kDeg);
  // Two courses along the track, not the five of a run.
  w.Run(0.6, 0.3, 0.0);
  EXPECT_LT(std::abs(w.HeadingError()), 1.0 * kDeg);
  EXPECT_LT(w.est.HeadingAccuracy(), 5.0 * kDeg);
}

TEST(E6DisturbedHeading, ALiftLoosensAValidHeadingOnly) {
  World w;
  w.Ready();
  w.est.LoosenHeading();
  EXPECT_FALSE(w.est.heading_valid());
  EXPECT_EQ(w.est.HeadingAccuracy(), kNoAccuracy);
  w.Run(1.2, 0.3, 0.0);
  ASSERT_TRUE(w.est.heading_valid());
  const double heading = w.est.pose().heading;
  w.est.LoosenHeading();
  EXPECT_NEAR(w.est.HeadingAccuracy(), 20.0 * kDeg, 0.5 * kDeg);
  EXPECT_DOUBLE_EQ(w.est.pose().heading, heading);
  EXPECT_TRUE(w.est.heading_valid());
  // A larger uncertainty stays as it is.
  w.est.LoosenHeading();
  EXPECT_NEAR(w.est.HeadingAccuracy(), 20.0 * kDeg, 0.5 * kDeg);
}

// E7: the antenna above the ground leans downhill on a slope; the tilt of the IMU turns the lever, the pose stays over
// the ground point under base_link.
EstimatorConfig HighAntenna() {
  EstimatorConfig config;
  config.antenna_height = 0.3;
  return config;
}

TEST(E7AntennaHeight, OnASideSlopeThePoseStaysOverTheGroundPoint) {
  World w(HighAntenna());
  // Right side 10 degrees down: the antenna 5.2 cm to the right of the level lever.
  w.true_lever = Eigen::Vector2d(0.1, -0.3 * std::sin(10.0 * kDeg));
  w.tilt = Tilt{10.0 * kDeg, 0.0};
  w.Aligned();
  w.Run(4.0, 0.3, 0.0);
  w.Run(1.0, 0.0, 0.0);
  EXPECT_LT(w.PositionError(), 0.005);
  EXPECT_LT(std::abs(w.HeadingError()), 1.0 * kDeg);
}

TEST(E7AntennaHeight, WithoutTheTiltTheLeverStaysLevel) {
  World w(HighAntenna());
  w.true_lever = Eigen::Vector2d(0.1, -0.3 * std::sin(10.0 * kDeg));
  w.Aligned();
  w.Run(4.0, 0.3, 0.0);
  w.Run(1.0, 0.0, 0.0);
  EXPECT_NEAR(w.PositionError(), 0.052, 0.005);
}

TEST(E7AntennaHeight, FrontDownAlongTheWayAndTurning) {
  World w(HighAntenna());
  w.true_lever = Eigen::Vector2d(0.1 * std::cos(10.0 * kDeg) + 0.3 * std::sin(10.0 * kDeg), 0.0);
  w.tilt = Tilt{0.0, 10.0 * kDeg};
  w.Aligned();
  w.Run(3.0, 0.3, 0.3);
  w.Run(1.0, 0.0, 0.0);
  EXPECT_LT(w.PositionError(), 0.005);
  EXPECT_LT(std::abs(w.HeadingError()), 1.0 * kDeg);
}

TEST(E7AntennaHeight, LevelItIsTheSameAsWithoutHeight) {
  World high(HighAntenna());
  high.tilt = Tilt{};
  World flat;
  for (World* w : {&high, &flat}) {
    w->Aligned();
    w->Run(3.0, 0.3, 0.2);
  }
  EXPECT_EQ(high.est.pose().x, flat.est.pose().x);
  EXPECT_EQ(high.est.pose().y, flat.est.pose().y);
  EXPECT_EQ(high.est.pose().heading, flat.est.pose().heading);
}

TEST(E7AntennaHeight, AnOldTiltCountsLevel) {
  World high(HighAntenna());
  World flat;
  // A tilt longer than max_imu_gap ago.
  high.est.AddTilt(high.t - 1.0, 10.0 * kDeg, 10.0 * kDeg);
  high.est.AddTilt(high.t, NAN, 0.0);
  for (World* w : {&high, &flat}) {
    w->Aligned();
    w->Run(1.0, 0.3, 0.0);
  }
  EXPECT_EQ(high.est.pose().x, flat.est.pose().x);
  EXPECT_EQ(high.est.pose().y, flat.est.pose().y);
}

TEST(E7AntennaHeight, LateFixesTakeTheTiltOfTheirMoment) {
  EstimatorConfig config = HighAntenna();
  config.gnss_latency = 0.2;
  World w(config);
  w.gnss_delay = 0.2;
  w.Aligned();
  // Rocking between level and 10 degrees: a fix 0.2 s late belongs to the tilt before the change.
  double worst = 0.0;
  for (int i = 0; i < 10; ++i) {
    const double roll = i % 2 ? 10.0 * kDeg : 0.0;
    w.tilt = Tilt{roll, 0.0};
    w.true_lever = Eigen::Vector2d(0.1, -0.3 * std::sin(roll));
    for (int j = 0; j < 6; ++j) {
      w.Run(0.1, 0.0, 0.0);
      worst = std::max(worst, w.PositionError());
    }
  }
  EXPECT_LT(worst, 0.005);
}

}  // namespace

// E8: wheels that turn the base less than they report slip, and a base that swings around the wheel that grips moves
// without the wheels reporting it.

// Without fixes the accuracy grows by half the track times what the gyro stays behind the wheels: the limit of
// 0.10 m comes while the real error is still below it.
TEST(E8Slip, WithoutFixesTheAccuracyGrowsWithWhatTheGyroStaysBehindTheWheels) {
  World w;
  w.est.SetWheelDistance(0.4);
  w.Aligned();
  w.gnss = false;
  const double start = w.est.PositionAccuracy(w.t);
  // The left wheel loses 0.8 of 0.6 rad/s: the base turns at 0.36 rad/s and moves at 0.048 m/s.
  double worst = -1.0;
  double stopped = 0;
  for (int i = 0; i < 25 && stopped == 0; i++) {
    w.Spin(0.2, 0.6, 0.8, 0.0);
    worst = std::max(worst, w.PositionError() - w.est.PositionAccuracy(w.t));
    if (w.est.PositionAccuracy(w.t) > 0.10) stopped = w.PositionError();
  }
  // The accuracy stays ahead of the error and passes the limit before the error does.
  EXPECT_LT(worst, 0.0);
  EXPECT_GT(stopped, 0.0);
  EXPECT_LT(stopped, 0.10);
  // 0.2 m * 0.24 rad/s = 0.048 m per s on top of the growth with time.
  World same;
  same.est.SetWheelDistance(0.4);
  same.Aligned();
  same.gnss = false;
  same.Spin(1.0, 0.6, 0.8, 0.0);
  EXPECT_NEAR(same.est.PositionAccuracy(same.t), start + 0.01 + 0.048, 1e-6);
}

// Wheels and gyro that agree show no slip, and without the track of the wheels nobody knows: as before.
TEST(E8Slip, NoneWhileWheelsAndGyroAgreeOrTheTrackIsUnknown) {
  World w;
  w.est.SetWheelDistance(0.4);
  w.Aligned();
  w.gnss = false;
  const double start = w.est.PositionAccuracy(w.t);
  w.Run(2.0, 0.0, 0.6);
  EXPECT_NEAR(w.est.PositionAccuracy(w.t), start + 0.02, 1e-9);
  World unknown;
  unknown.Aligned();
  unknown.gnss = false;
  unknown.Spin(2.0, 0.6, 0.8, 0.0);
  EXPECT_NEAR(unknown.est.PositionAccuracy(unknown.t), start + 0.02, 1e-9);
  unknown.est.SetWheelDistance(std::nan(""));
  unknown.Spin(1.0, 0.6, 0.8, 0.0);
  EXPECT_NEAR(unknown.est.PositionAccuracy(unknown.t), start + 0.03, 1e-9);
}

// With fixes the estimate follows a base that slips away while it turns on the spot; without the track of the
// wheels the fixes end as contradictions and a jump.
TEST(E8Slip, WithFixesTheEstimateFollowsTheSlidingBase) {
  World blind;
  blind.Aligned();
  blind.Run(20.0, 0.0, 0.0);
  double blind_worst = 0;
  bool jumped = false;
  for (int i = 0; i < 40; i++) {
    blind.Spin(0.2, 0.6, 0.8, 0.0);
    blind_worst = std::max(blind_worst, blind.PositionError());
    jumped = jumped || blind.est.TakeJump().has_value();
  }
  EXPECT_GT(blind_worst, 0.10);
  EXPECT_TRUE(jumped);

  World w;
  w.est.SetWheelDistance(0.4);
  w.Aligned();
  w.Run(20.0, 0.0, 0.0);
  double worst = 0;
  for (int i = 0; i < 40; i++) {
    w.Spin(0.2, 0.6, 0.8, 0.0);
    worst = std::max(worst, w.PositionError());
    EXPECT_EQ(w.last(), FixResult::kFused);
    EXPECT_FALSE(w.est.TakeJump());
  }
  EXPECT_LT(worst, 0.03);
  EXPECT_LT(std::abs(w.HeadingError()), 1.0 * kDeg);
}

TEST(L7Accuracy, TheAgeOfTheLastFixTheEstimateTook) {
  World w;
  EXPECT_EQ(w.est.FixAge(w.t), kNoAccuracy);
  w.Ready();
  EXPECT_NEAR(w.est.FixAge(w.t), 0.0, 1e-9);
  w.gnss = false;
  w.Run(1.2, 0.0, 0.0);
  EXPECT_NEAR(w.est.FixAge(w.t), 1.2, 1e-9);
  // A fix the estimate does not take leaves the age.
  w.gnss = true;
  w.accuracy = 0.3;
  w.Run(0.4, 0.0, 0.0);
  EXPECT_NEAR(w.est.FixAge(w.t), 1.6, 1e-9);
  w.accuracy = 0.02;
  w.Run(0.2, 0.0, 0.0);
  EXPECT_NEAR(w.est.FixAge(w.t), 0.0, 1e-9);
}

// A gap of the gyro: heading and position count as disturbed until the next fix and course.
TEST(Estimator, AGapOfTheGyroDisturbsHeadingAndPosition) {
  World w;
  w.Aligned();
  ASSERT_LT(w.est.PositionAccuracy(w.t), 0.1);
  w.t += 1.0;
  w.est.AddImu(w.t, 0.0);
  EXPECT_EQ(w.est.PositionAccuracy(w.t), kNoAccuracy);
  EXPECT_GE(w.est.covariance()(2, 2), 0.9 * std::pow(20.0 * kDeg, 2));
}

// Wheels that stopped reporting: the mower may drive on, so the position gets less accurate as if it did.
TEST(Estimator, StaleWheelsMakeThePositionLessAccurate) {
  World w;
  w.Aligned();
  w.gnss = false;
  const double before = w.est.PositionAccuracy(w.t);
  for (int i = 0; i < 100; ++i) {
    w.t += kStep;
    w.est.AddImu(w.t, 0.0);
  }
  EXPECT_GE(w.est.PositionAccuracy(w.t), before + 0.6 * 1.5);
}

// The same lever again moves nothing; a new one after a drive without a fix only takes the lever.
TEST(Estimator, TheSameLeverAgainKeepsThePose) {
  World w;
  w.Aligned();
  w.gnss = false;
  w.Run(2.0, 0.3, 0.0);
  const Pose2 driven = w.est.pose();
  w.Geometry();
  EXPECT_NEAR(w.est.pose().x, driven.x, 1e-9);
  EXPECT_NEAR(w.est.pose().y, driven.y, 1e-9);
  w.est.SetAntennaOffset(0.2, 0.0, w.t);
  EXPECT_NEAR(w.est.pose().x, driven.x, 1e-9);
}

TEST(Estimator, APoseOrLimitsThatAreNoNumberChangeNothing) {
  World w;
  w.Aligned();
  const Pose2 before = w.est.pose();
  w.est.SetPose(Pose2{std::nan(""), 1.0, 0.0});
  EXPECT_DOUBLE_EQ(w.est.pose().x, before.x);
  EXPECT_TRUE(std::isfinite(w.est.pose().x));
  // A timeout that is no number would keep the position forever.
  w.est.SetGpsLimits(std::nan(""), std::numeric_limits<double>::infinity());
  w.gnss = false;
  w.Run(11.0, 0.0, 0.0);
  EXPECT_EQ(w.est.PositionAccuracy(w.t), kNoAccuracy);
}
