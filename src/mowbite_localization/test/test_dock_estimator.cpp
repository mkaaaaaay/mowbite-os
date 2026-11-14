#include <gtest/gtest.h>

#include <cmath>
#include <deque>
#include <random>

#include "mowbite_localization/dock_estimator.h"
#include "mowbite_localization/estimator.h"
#include "mowbite_localization/position_accuracy.h"

using namespace mowbite_localization;

namespace {

constexpr double kStep = 0.02;
constexpr double kDeg = M_PI / 180.0;

// The Tron: contacts behind, so the docked mower faces against the way in; the marker 0.175 m behind its base_link.
DockConfig TronConfig() {
  DockConfig config;
  config.marker_x = -0.175;
  config.marker_y = 0.0;
  config.marker_yaw = 0.0;
  return config;
}

// A mower in the station frame on a unicycle; motions at 50 Hz, an image of the marker every image_period s, taken
// delay s before it arrives.
class DockWorld {
 public:
  explicit DockWorld(const DockConfig& config = TronConfig()) : est(config), config_(config) {
    est.SetDockedHeading(M_PI);
  }

  // Where the marker is in the station frame.
  State Marker() const {
    return State(-config_.marker_x, -config_.marker_y, WrapAngle(M_PI + config_.marker_yaw));
  }

  // The image of now, as the camera sees it: the marker in base_link.
  MarkerImage Image(double sigma = 0.0, double yaw_sigma = 0.0) {
    const State m = Marker();
    const double c = std::cos(heading);
    const double s = std::sin(heading);
    const double dx = m(0) - x;
    const double dy = m(1) - y;
    MarkerImage image;
    image.time = t;
    image.x = c * dx + s * dy + sigma * noise_(rng_);
    image.y = -s * dx + c * dy + sigma * noise_(rng_);
    image.yaw_valid = yaw_valid;
    image.yaw = yaw_valid ? WrapAngle(m(2) - heading + yaw_bias + yaw_sigma * noise_(rng_)) : 0.0;
    const double variance = std::max(sigma * sigma, 1e-6);
    const double yaw_spread = yaw_reported > 0.0 ? yaw_reported : yaw_sigma;
    image.covariance = {variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0,
                        yaw_valid ? std::max(yaw_spread * yaw_spread, 1e-6) : 0.0};
    return image;
  }

  // Moves for seconds at speed and turn; with images, one every image_period s, delivered delay s late.
  void Run(double seconds, double speed, double turn, bool images = false, double sigma = 0.0,
           double yaw_sigma = 0.0) {
    for (long i = 0; i < std::lround(seconds / kStep); ++i) {
      t += kStep;
      const double mid = heading + 0.5 * turn * kStep;
      x += speed * kStep * std::cos(mid);
      y += speed * kStep * std::sin(mid);
      heading = WrapAngle(heading + turn * kStep);
      const bool standing = speed == 0.0 && turn == 0.0;
      still_for_ = standing ? still_for_ + kStep : 0.0;
      est.AddMotion(Motion{t, speed * kStep, turn * kStep, still_for_ >= 1.0 ? 0.0 : kStep, still_for_ >= 1.0});
      if (images && ++steps_ % std::lround(image_period / kStep) == 0) pending_.push_back(Image(sigma, yaw_sigma));
      while (!pending_.empty() && pending_.front().time <= t - delay + 1e-9) {
        results.push_back(est.AddMarker(pending_.front(), t));
        pending_.pop_front();
      }
    }
  }

  // Stands until the gyro offset holds.
  void Settle() { Run(1.2, 0.0, 0.0); }

  // Started from images at rest with their direction.
  void Started() {
    Settle();
    Run(1.2, 0.0, 0.0, true);
    ASSERT_TRUE(est.has_estimate());
  }

  double PositionError() const { return std::hypot(est.pose().x - x, est.pose().y - y); }
  double HeadingError() const { return WrapAngle(est.pose().heading - heading); }
  MarkerResult last() const { return results.empty() ? MarkerResult::kNoFrame : results.back(); }

  DockEstimator est;
  // The mower 0.8 m before the docked pose, facing as it stands there.
  double t = 50.0;
  double x = -0.8;
  double y = 0.0;
  double heading = M_PI;
  bool yaw_valid = true;
  // The error of the direction that stays at one place, and the spread an image reports (0: that of its noise).
  double yaw_bias = 0.0;
  double yaw_reported = 0.0;
  double image_period = 0.1;
  double delay = 0.0;
  std::vector<MarkerResult> results;

 private:
  DockConfig config_;
  std::mt19937 rng_{7};
  std::normal_distribution<double> noise_{0.0, 1.0};
  double still_for_ = 0.0;
  long steps_ = 0;
  std::deque<MarkerImage> pending_;
};

}  // namespace

TEST(DockEstimator, NothingWithoutTheHeadingOfTheDockedMower) {
  DockEstimator est(TronConfig());
  MarkerImage image;
  image.covariance[0] = image.covariance[4] = 1e-4;
  EXPECT_EQ(est.AddMarker(image, 0.0), MarkerResult::kNoFrame);
}

TEST(DockEstimator, StartsAtRestFromTenImagesWithTheirDirection) {
  DockWorld w;
  w.y = 0.03;
  w.heading = M_PI + 3.0 * kDeg;
  w.Settle();
  w.Run(0.9, 0.0, 0.0, true, 0.003, 1.0 * kDeg);
  EXPECT_FALSE(w.est.has_estimate());
  EXPECT_EQ(w.results.size(), 9u);
  EXPECT_EQ(w.est.PositionAccuracy(w.t), kNoAccuracy);
  w.Run(0.1, 0.0, 0.0, true, 0.003, 1.0 * kDeg);
  ASSERT_EQ(w.last(), MarkerResult::kStarted);
  EXPECT_FALSE(w.est.heading_from_hint());
  EXPECT_LT(w.PositionError(), 0.02);
  EXPECT_LT(std::abs(w.HeadingError()), 1.0 * kDeg);
  EXPECT_TRUE(w.est.Good(w.t));
  EXPECT_LT(w.est.PositionAccuracy(w.t), 0.03);
  EXPECT_NEAR(w.est.HeadingAccuracy(), 1.0 * kDeg, 1e-9) << "the spread of one image, not of ten";
}

TEST(DockEstimator, ImagesOfOnePlaceAreOneHeading) {
  // 4 degrees off at this place, 0.1 degrees from image to image, 3.5 degrees reported (as measured 13./14.11.).
  DockWorld w;
  w.yaw_bias = 4.0 * kDeg;
  w.yaw_reported = 3.5 * kDeg;
  w.Settle();
  w.Run(1.0, 0.0, 0.0, true, 0.003, 0.1 * kDeg);
  ASSERT_TRUE(w.est.has_estimate());
  EXPECT_NEAR(w.HeadingError(), -4.0 * kDeg, 0.2 * kDeg);
  EXPECT_NEAR(w.est.HeadingAccuracy(), 3.5 * kDeg, 1e-9);
  // Standing, more images of the same place neither correct the heading nor make it surer.
  w.Run(10.0, 0.0, 0.0, true, 0.003, 0.1 * kDeg);
  EXPECT_EQ(w.est.yaw_fixes(), 0);
  EXPECT_GE(w.est.HeadingAccuracy(), 3.5 * kDeg * 0.999);
  EXPECT_LE(std::abs(w.HeadingError()), 3.0 * w.est.HeadingAccuracy());
  for (MarkerResult result : w.results) EXPECT_NE(result, MarkerResult::kOutlier);
}

TEST(DockEstimator, ADirectionCountsOnlyAfterTheMowerMoved) {
  DockConfig config = TronConfig();
  // A gyro this noisy makes the heading less sure than an image while driving.
  config.noise.gyro = 0.05;
  DockWorld w(config);
  w.yaw_reported = 2.0 * kDeg;
  w.Settle();
  w.Run(1.0, 0.0, 0.0, true, 0.003, 0.1 * kDeg);
  ASSERT_TRUE(w.est.has_estimate());
  w.Run(0.4, -0.1, 0.0, true, 0.003, 0.1 * kDeg);
  EXPECT_EQ(w.est.yaw_fixes(), 0) << "0.04 m since the start";
  w.Run(0.2, -0.1, 0.0, true, 0.003, 0.1 * kDeg);
  EXPECT_EQ(w.est.yaw_fixes(), 1);
  w.Run(0.3, -0.1, 0.0, true, 0.003, 0.1 * kDeg);
  EXPECT_EQ(w.est.yaw_fixes(), 1) << "0.03 m since the last one";
  // Turning 2 degrees on the spot counts as well.
  w.Run(0.2, 0.0, 0.2, true, 0.003, 0.1 * kDeg);
  EXPECT_EQ(w.est.yaw_fixes(), 2);
}

TEST(DockEstimator, ABiasThatChangesWithThePlaceStaysWithinTheReportedSpread) {
  for (const double amplitude : {3.5, 6.0}) {
    DockWorld w;
    w.yaw_reported = 3.0 * kDeg;
    w.yaw_bias = amplitude * kDeg;
    w.Settle();
    w.Run(1.0, 0.0, 0.0, true, 0.003, 0.1 * kDeg);
    ASSERT_TRUE(w.est.has_estimate());
    double worst = 0.0;
    for (int i = 0; i < 70; ++i) {
      // Changes slowly along the way in: a different error every few cm, 0.1 degrees from image to image.
      w.yaw_bias = amplitude * kDeg * std::cos(10.0 * (w.x + 0.8));
      w.Run(0.1, -0.1, 0.0, true, 0.003, 0.1 * kDeg);
      worst = std::max(worst, std::abs(w.HeadingError()) / w.est.HeadingAccuracy());
      ASSERT_TRUE(w.est.has_estimate()) << amplitude << " " << i;
    }
    for (MarkerResult result : w.results) EXPECT_NE(result, MarkerResult::kDropped) << amplitude;
    EXPECT_LE(worst, 3.0) << amplitude;
    EXPECT_LE(w.PositionError(), 3.0 * w.est.PositionAccuracy(w.t)) << amplitude;
  }
}

TEST(DockEstimator, TheStartImagesAgreeInARow) {
  DockWorld w;
  w.Settle();
  w.Run(0.5, 0.0, 0.0, true);
  MarkerImage outlier = w.Image();
  outlier.x += 0.3;
  EXPECT_EQ(w.est.AddMarker(outlier, w.t), MarkerResult::kCollecting);
  // The window starts again with the outlier, and again with the first image after it.
  w.Run(0.9, 0.0, 0.0, true);
  EXPECT_FALSE(w.est.has_estimate());
  w.Run(0.1, 0.0, 0.0, true);
  ASSERT_TRUE(w.est.has_estimate());
  EXPECT_LT(w.PositionError(), 0.002);
}

TEST(DockEstimator, NoStartWhileMoving) {
  DockWorld w;
  w.Settle();
  w.Run(2.0, -0.05, 0.0, true);
  EXPECT_FALSE(w.est.has_estimate());
  for (MarkerResult result : w.results) EXPECT_EQ(result, MarkerResult::kCollecting);
  // Standing again, the images count from the second the gyro offset holds.
  w.Run(1.0, 0.0, 0.0, true);
  EXPECT_FALSE(w.est.has_estimate());
  w.Run(1.0, 0.0, 0.0, true);
  EXPECT_TRUE(w.est.has_estimate());
}

TEST(DockEstimator, WithoutDirectionTheStartNeedsAFreshHint) {
  DockWorld w;
  w.yaw_valid = false;
  w.Settle();
  w.Run(2.0, 0.0, 0.0, true);
  EXPECT_FALSE(w.est.has_estimate());
  // A hint 2 degrees off; the mower turns and stands again, the tenth image comes 3.2 s after the hint.
  w.est.SetHint(WrapAngle(w.heading + 2.0 * kDeg), w.t);
  w.Run(1.0, 0.0, 0.3);
  w.Run(1.2, 0.0, 0.0);
  w.Run(1.0, 0.0, 0.0, true);
  EXPECT_FALSE(w.est.has_estimate()) << "the hint is older than hint_hold";
  w.est.SetHint(WrapAngle(w.heading + 2.0 * kDeg), w.t);
  w.Run(0.1, 0.0, 0.0, true);
  ASSERT_TRUE(w.est.has_estimate());
  EXPECT_TRUE(w.est.heading_from_hint());
  EXPECT_NEAR(w.HeadingError(), 2.0 * kDeg, 1e-6);
  // The heading spread of the hint shows in the position: 0.97 m of lever times 5 degrees.
  EXPECT_NEAR(w.est.PositionAccuracy(w.t), 0.97 * 5.0 * kDeg, 0.01);
}

TEST(DockEstimator, TheHintIsCarriedByTheGyro) {
  DockWorld w;
  w.yaw_valid = false;
  w.Settle();
  w.est.SetHint(w.heading, w.t);
  w.Run(0.5, 0.0, 0.6);
  w.Run(1.2, 0.0, 0.0);
  w.Run(1.0, 0.0, 0.0, true);
  ASSERT_TRUE(w.est.has_estimate());
  EXPECT_NEAR(w.HeadingError(), 0.0, 1e-9);
}

TEST(DockEstimator, FollowsTheWayInWithLateImages) {
  DockWorld w;
  w.delay = 0.15;
  w.Started();
  double worst = 0.0;
  for (int i = 0; i < 70; ++i) {
    w.Run(0.1, -0.1, 0.02 * std::sin(0.2 * i), true, 0.003, 2.0 * kDeg);
    worst = std::max(worst, w.PositionError());
    EXPECT_TRUE(w.est.Good(w.t)) << i;
  }
  EXPECT_LT(worst, 0.01);
  EXPECT_LT(std::abs(w.HeadingError()), 1.5 * kDeg);
}

TEST(DockEstimator, LateImagesCountAtTheirTime) {
  // Fused as if taken on arrival they would lag 0.15 s at 0.1 m/s, 15 mm.
  DockWorld w;
  w.delay = 0.15;
  w.Started();
  w.Run(5.0, -0.1, 0.0, true);
  EXPECT_LT(w.PositionError(), 0.003);
}

TEST(DockEstimator, ALateImageLeavesTheNoiseOfTheWayDrivenSinceItWasTaken) {
  // Taken 0.28 s before it arrives at 0.17 m/s: no image knows the last 0.048 m, their odometry noise stays.
  DockWorld w;
  w.delay = 0.28;
  w.Settle();
  w.Run(1.5, 0.0, 0.0, true);
  ASSERT_TRUE(w.est.has_estimate());
  const double since = std::pow(TronConfig().noise.odometry, 2) * 0.17 * 0.28;
  w.Run(0.3, -0.17, 0.0, true, 0.003, 2.0 * kDeg);
  for (int i = 0; i < 30; ++i) {
    w.Run(0.1, -0.17, 0.0, true, 0.003, 2.0 * kDeg);
    EXPECT_GE(w.est.covariance()(0, 0), 0.999 * since) << i;
    EXPECT_GE(w.est.covariance()(1, 1), 0.999 * since) << i;
  }
  EXPECT_LT(w.PositionError(), 3.0 * w.est.PositionAccuracy(w.t));
}

TEST(DockEstimator, DropsImagesOlderThanMaxAgeOrFromTheFutureOrOutOfOrder) {
  DockWorld w;
  w.Started();
  w.Run(0.5, 0.0, 0.0);
  MarkerImage image = w.Image();
  image.time = w.t - 0.31;
  EXPECT_EQ(w.est.AddMarker(image, w.t), MarkerResult::kTooOld);
  image.time = w.t + 0.1;
  EXPECT_EQ(w.est.AddMarker(image, w.t), MarkerResult::kTooOld);
  image.time = w.t - 0.29;
  EXPECT_EQ(w.est.AddMarker(image, w.t), MarkerResult::kFused);
  image.time = w.t - 0.295;
  EXPECT_EQ(w.est.AddMarker(image, w.t), MarkerResult::kTooOld) << "taken before the image of the last fix";
  image.covariance[4] = 0.0;
  EXPECT_EQ(w.est.AddMarker(image, w.t), MarkerResult::kInvalid);
  image.covariance[4] = 1e-4;
  image.x = std::nan("");
  EXPECT_EQ(w.est.AddMarker(image, w.t), MarkerResult::kInvalid);
}

TEST(DockEstimator, SingleOutliersDoNotMoveTheEstimate) {
  DockWorld w;
  w.Started();
  for (int i = 0; i < 10; ++i) {
    const Pose2 before = w.est.pose();
    MarkerImage outlier = w.Image();
    outlier.x += 0.1;
    EXPECT_EQ(w.est.AddMarker(outlier, w.t), MarkerResult::kOutlier);
    EXPECT_EQ(w.est.pose().x, before.x);
    EXPECT_EQ(w.est.pose().y, before.y);
    w.Run(0.1, 0.0, 0.0, true);
    EXPECT_EQ(w.last(), MarkerResult::kFused);
  }
  // A direction 10 degrees off is one as well.
  MarkerImage turned = w.Image();
  turned.yaw = WrapAngle(turned.yaw + 10.0 * kDeg);
  EXPECT_EQ(w.est.AddMarker(turned, w.t), MarkerResult::kOutlier);
  EXPECT_TRUE(w.est.Good(w.t));
  EXPECT_LT(w.PositionError(), 0.002);
}

TEST(DockEstimator, OutliersInARowDropTheEstimate) {
  DockWorld w;
  w.Started();
  MarkerImage outlier = w.Image();
  outlier.y += 0.1;
  EXPECT_EQ(w.est.AddMarker(outlier, w.t), MarkerResult::kOutlier);
  EXPECT_EQ(w.est.AddMarker(outlier, w.t), MarkerResult::kOutlier);
  EXPECT_EQ(w.est.AddMarker(outlier, w.t), MarkerResult::kDropped);
  EXPECT_FALSE(w.est.has_estimate());
  EXPECT_FALSE(w.est.Good(w.t));
  EXPECT_EQ(w.est.HeadingAccuracy(), kNoAccuracy);
  // A new start, at rest, from new images.
  w.Run(1.0, 0.0, 0.0, true);
  EXPECT_TRUE(w.est.has_estimate());
  EXPECT_LT(w.PositionError(), 0.002);
}

TEST(DockEstimator, GoodOnlyNearTheLastFix) {
  DockWorld w;
  w.Started();
  w.Run(0.48, -0.1, 0.0);
  EXPECT_TRUE(w.est.Good(w.t));
  w.Run(0.04, -0.1, 0.0);
  EXPECT_FALSE(w.est.Good(w.t)) << "more than 0.05 m since the image of the last fix";
  EXPECT_EQ(w.est.PositionAccuracy(w.t), kNoAccuracy);
  EXPECT_TRUE(w.est.has_estimate());
  w.Run(0.2, 0.0, 0.0, true);
  EXPECT_TRUE(w.est.Good(w.t));
  w.Run(1.02, 0.0, 0.0);
  EXPECT_FALSE(w.est.Good(w.t)) << "more than 1 s since the image of the last fix";
}

TEST(DockEstimator, TheFinalStretchIsGoodWithoutImages) {
  DockWorld w;
  w.x = -0.1;
  w.Started();
  w.Run(0.2, -0.1, 0.0, true);
  // Images end 0.08 m before the docked pose; from there blind to the contacts and 0.05 m past them.
  w.Run(0.2, -0.05, 0.0);
  EXPECT_TRUE(w.est.Good(w.t));
  w.Run(2.0, -0.05, 0.0);
  EXPECT_TRUE(w.est.Good(w.t)) << w.est.pose().x;
  w.Run(8.0, 0.0, 0.0);
  EXPECT_TRUE(w.est.Good(w.t));
  w.Run(2.0, 0.0, 0.0);
  EXPECT_FALSE(w.est.Good(w.t)) << "more than final_time since the last fix";
}

TEST(DockEstimator, TheFinalStretchNeedsAFixNearIt) {
  DockWorld w;
  w.x = -0.3;
  w.Started();
  // Without images from 0.3 m on: not good after 0.05 m, and not again in the blind distance.
  w.Run(5.0, -0.05, 0.0);
  EXPECT_GT(w.est.pose().x, -0.07);
  EXPECT_FALSE(w.est.Good(w.t));
}

TEST(DockEstimator, TheFinalStretchEndsAfterItsWay) {
  DockWorld w;
  w.x = -0.1;
  w.Started();
  w.Run(0.2, -0.1, 0.0, true);
  w.Run(2.6, -0.05, 0.0);
  EXPECT_TRUE(w.est.Good(w.t));
  w.Run(0.4, -0.05, 0.0);
  EXPECT_FALSE(w.est.Good(w.t)) << "more than final_way since the last fix";
}

TEST(DockEstimator, ANewDockedHeadingDropsTheEstimate) {
  DockWorld w;
  w.Started();
  w.est.SetDockedHeading(M_PI);
  EXPECT_TRUE(w.est.has_estimate());
  w.est.SetDockedHeading(0.0);
  EXPECT_FALSE(w.est.has_estimate());
}

TEST(DockEstimator, MovesWithTheMotionsOfTheMapEstimate) {
  // Gyro offset and wheel scale are those of the Estimator: turned and driven the same.
  EstimatorConfig map_config;
  map_config.gyro.calibration_time = 1.0;
  Estimator map(map_config);
  map.SetWheelScale(1000.0, 950.0);
  DockWorld w;
  w.Settle();
  w.Run(1.0, 0.0, 0.0, true);
  ASSERT_TRUE(w.est.has_estimate());
  const Pose2 dock_before = w.est.pose();
  double t = w.t;
  const auto step = [&](double speed, double rate) {
    t += kStep;
    map.AddWheels(t, speed, 0.0);
    if (map.AddImu(t, rate + 0.01)) w.est.AddMotion(map.motion());
  };
  for (int i = 0; i < 100; ++i) step(0.0, 0.0);
  ASSERT_TRUE(map.calibrated());
  const Pose2 map_before = map.pose();
  for (int i = 0; i < 100; ++i) step(0.1, 0.2);
  EXPECT_NEAR(WrapAngle(w.est.pose().heading - dock_before.heading),
              WrapAngle(map.pose().heading - map_before.heading), 1e-12);
  EXPECT_NEAR(std::hypot(w.est.pose().x - dock_before.x, w.est.pose().y - dock_before.y),
              std::hypot(map.pose().x - map_before.x, map.pose().y - map_before.y), 1e-12);
}
