#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <utility>

#include "dock_scene.h"
#include "tron_vision/dock_pose.h"

namespace tron_vision {
namespace {

using test::Behind;
using test::RearScene;
using test::Scene;
using test::SceneMarker;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kFocal = 1540.0;

// Directions of the corners as a perfect detector would report them.
std::array<Point2, 4> Corners(const Scene& s, const SceneMarker& m) {
  std::array<Point2, 4> rays{};
  const std::array<Point2, 4> px = test::CornerPixels(s, m);
  for (size_t k = 0; k < 4; k++) EXPECT_TRUE(ToRay(s.lens, px[k], rays[k]));
  return rays;
}

// The yaw spread from the corners alone, as beyond the measured range.
PoseLimits CornersOnly() {
  PoseLimits limits;
  limits.measured_yaw_range = 0.0;
  return limits;
}

double Range(const Scene& s, const MarkerPose& pose) {
  return std::hypot(pose.x - s.mount.x, pose.y - s.mount.y, pose.z - s.mount.z);
}

// Independent corners with the same standard deviation, given in pixels.
std::array<double, 64> Covariance(double sigma_px) {
  std::array<double, 64> covariance{};
  for (size_t i = 0; i < 8; i++) covariance[i * 9] = sigma_px * sigma_px / (kFocal * kFocal);
  return covariance;
}

TEST(DockPose, RecoversPositionAndYaw) {
  const Scene s = RearScene();
  for (const double distance : {1.5, 0.8, 0.4, 0.15}) {
    for (const double yaw : {0.0, 12.0 * kDeg, -30.0 * kDeg}) {
      for (const double lateral : {0.0, 0.2 * distance, -0.1 * distance}) {
        const SceneMarker m = Behind(s, distance, lateral, yaw);
        const MarkerPose pose =
            FitMarkerPose(Corners(s, m), Covariance(0.02), kFocal, s.mount, s.side, PoseLimits{});
        ASSERT_TRUE(pose.valid) << distance << " " << yaw << " " << lateral;
        EXPECT_TRUE(pose.yaw_valid) << distance << " " << yaw << " " << lateral;
        EXPECT_NEAR(pose.x, m.x, 1e-6);
        EXPECT_NEAR(pose.y, m.y, 1e-6);
        EXPECT_NEAR(pose.z, m.z, 1e-6);
        EXPECT_NEAR(pose.yaw, m.yaw, 1e-5);
        EXPECT_LT(pose.fit_error_px, 1e-4);
      }
    }
  }
}

TEST(DockPose, YawIsTheDirectionTheMarkerLooksIn) {
  // Straight behind the mower and facing it: 0. A camera at the front sees a marker facing back at the mower: pi.
  Scene s = RearScene();
  SceneMarker m = Behind(s, 0.5, 0.0, 0.0);
  m.y = 0.0;
  MarkerPose pose = FitMarkerPose(Corners(s, m), Covariance(0.02), kFocal, s.mount, s.side, PoseLimits{});
  ASSERT_TRUE(pose.valid);
  EXPECT_NEAR(pose.yaw, std::atan2(-m.y, -m.x), 1e-5);
  EXPECT_NEAR(pose.yaw, 0.0, 1e-5);

  s.mount = {0.3, 0.0, 0.25, 0.0, 0.0, 0.0};
  m.x = 0.9;
  m.yaw = kPi;
  pose = FitMarkerPose(Corners(s, m), Covariance(0.02), kFocal, s.mount, s.side, PoseLimits{});
  ASSERT_TRUE(pose.valid);
  EXPECT_NEAR(std::fabs(pose.yaw), kPi, 1e-5);
}

TEST(DockPose, CovarianceGrowsWithTheCornerScatterAndTheDistance) {
  const Scene s = RearScene();
  const SceneMarker near = Behind(s, 0.3, 0.02, 10.0 * kDeg);
  const SceneMarker far = Behind(s, 0.9, 0.06, 10.0 * kDeg);
  const MarkerPose fine = FitMarkerPose(Corners(s, near), Covariance(0.1), kFocal, s.mount, s.side, CornersOnly());
  const MarkerPose coarse = FitMarkerPose(Corners(s, near), Covariance(0.2), kFocal, s.mount, s.side, CornersOnly());
  const MarkerPose away = FitMarkerPose(Corners(s, far), Covariance(0.1), kFocal, s.mount, s.side, CornersOnly());
  ASSERT_TRUE(fine.valid && coarse.valid && away.valid);
  for (const size_t i : {0U, 4U, 8U}) {
    EXPECT_NEAR(coarse.covariance[i] / fine.covariance[i], 4.0, 1e-3) << i;
    EXPECT_GT(away.covariance[i], 4.0 * fine.covariance[i]) << i;
  }
  // Symmetric, and the distance is the least certain part.
  EXPECT_NEAR(fine.covariance[1], fine.covariance[3], 1e-15);
  EXPECT_NEAR(fine.covariance[2], fine.covariance[6], 1e-15);
  EXPECT_GT(fine.covariance[0], fine.covariance[4]);
}

TEST(DockPose, ReportedCovarianceMatchesTheScatterOfNoisyCorners) {
  const Scene s = RearScene();
  const SceneMarker m = Behind(s, 0.5, 0.04, 15.0 * kDeg);
  const std::array<Point2, 4> exact = Corners(s, m);
  constexpr double kSigmaPx = 0.2;
  constexpr int kRuns = 600;
  std::mt19937 rng(7);
  std::array<double, 3> sum{};
  std::array<double, 3> squares{};
  std::array<double, 3> reported{};
  for (int run = 0; run < kRuns; run++) {
    std::array<Point2, 4> corners = exact;
    for (Point2& c : corners) {
      // Sum of twelve uniform numbers less 6: close to a standard normal one.
      for (double* value : {&c.x, &c.y}) {
        double noise = -6.0;
        for (int i = 0; i < 12; i++) noise += static_cast<double>(rng()) / 4294967296.0;
        *value += noise * kSigmaPx / kFocal;
      }
    }
    const MarkerPose pose = FitMarkerPose(corners, Covariance(kSigmaPx), kFocal, s.mount, s.side, CornersOnly());
    ASSERT_TRUE(pose.valid && pose.yaw_valid);
    const std::array<double, 3> error = {pose.x - m.x, pose.y - m.y, pose.yaw - m.yaw};
    for (size_t i = 0; i < 3; i++) {
      sum[i] += error[i];
      squares[i] += error[i] * error[i];
      reported[i] += pose.covariance[i * 4];
    }
  }
  for (size_t i = 0; i < 3; i++) {
    const double mean = sum[i] / kRuns;
    const double scatter = std::sqrt(squares[i] / kRuns - mean * mean);
    const double claimed = std::sqrt(reported[i] / kRuns);
    // The covariance is widened whenever a fit happens to be worse than expected, so it leans to the safe side.
    EXPECT_GT(scatter / claimed, 0.75) << i;
    EXPECT_LT(scatter / claimed, 1.1) << i;
    EXPECT_LT(std::fabs(mean), 0.2 * scatter) << i;
  }
}

TEST(DockPose, YawDoesNotCountWhileItsMirrorImageFitsAsWell) {
  // Far away and seen from the side: turned the other way the square looks almost the same.
  const Scene s = RearScene();
  const SceneMarker m = Behind(s, 1.2, 0.0, 25.0 * kDeg);
  const MarkerPose vague = FitMarkerPose(Corners(s, m), Covariance(0.3), kFocal, s.mount, s.side, PoseLimits{});
  ASSERT_TRUE(vague.valid);
  EXPECT_FALSE(vague.yaw_valid);
  EXPECT_NEAR(vague.x, m.x, 1e-6);
  EXPECT_NEAR(vague.y, m.y, 1e-6);
  EXPECT_GT(vague.covariance[0], 0.0);
  EXPECT_GT(vague.covariance[4], 0.0);
  for (const size_t i : {2U, 5U, 6U, 7U, 8U}) EXPECT_EQ(vague.covariance[i], 0.0) << i;
  // The same square with corners good enough to tell the two apart.
  const MarkerPose sharp = FitMarkerPose(Corners(s, m), Covariance(0.03), kFocal, s.mount, s.side, PoseLimits{});
  ASSERT_TRUE(sharp.valid);
  EXPECT_TRUE(sharp.yaw_valid);
  EXPECT_NEAR(sharp.yaw, m.yaw, 1e-5);
  // And from near by even with the rough corners.
  const SceneMarker near = Behind(s, 0.3, 0.0, 25.0 * kDeg);
  EXPECT_TRUE(FitMarkerPose(Corners(s, near), Covariance(0.3), kFocal, s.mount, s.side, PoseLimits{}).yaw_valid);
}

TEST(DockPose, WithinTheMeasuredRangeTheYawHasTheMeasuredSpread) {
  const Scene s = RearScene();
  const PoseLimits limits;
  for (const double distance : {0.15, 0.5, 0.8}) {
    for (const double sigma_px : {0.02, 0.3}) {
      const SceneMarker m = Behind(s, distance, 0.05, 15.0 * kDeg);
      const MarkerPose corners = FitMarkerPose(Corners(s, m), Covariance(sigma_px), kFocal, s.mount, s.side,
                                               CornersOnly());
      const MarkerPose pose = FitMarkerPose(Corners(s, m), Covariance(sigma_px), kFocal, s.mount, s.side, limits);
      ASSERT_TRUE(pose.valid && pose.yaw_valid) << distance << " " << sigma_px;
      ASSERT_LE(Range(s, pose), limits.measured_yaw_range);
      EXPECT_NEAR(std::sqrt(pose.covariance[8]), 4.0 * kDeg, 1e-12) << distance << " " << sigma_px;
      // Position as from the corners, the correlations with the yaw kept.
      for (const size_t i : {0U, 1U, 3U, 4U}) EXPECT_EQ(pose.covariance[i], corners.covariance[i]) << i;
      for (const size_t i : {2U, 5U}) {
        const size_t row = i == 2U ? 0U : 4U;
        const auto correlation = [&](const MarkerPose& p) {
          return p.covariance[i] / std::sqrt(p.covariance[row] * p.covariance[8]);
        };
        EXPECT_NEAR(correlation(pose), correlation(corners), 1e-9) << i;
      }
      for (const auto& [a, b] : {std::pair<size_t, size_t>{2U, 6U}, {5U, 7U}}) {
        EXPECT_NEAR(pose.covariance[a], pose.covariance[b], 1e-9 * std::fabs(pose.covariance[a])) << a;
      }
      // Still a covariance: positive definite.
      const std::array<double, 9>& c = pose.covariance;
      const double det = c[0] * (c[4] * c[8] - c[5] * c[7]) - c[1] * (c[3] * c[8] - c[5] * c[6]) +
                         c[2] * (c[3] * c[7] - c[4] * c[6]);
      EXPECT_GT(c[0] * c[4] - c[1] * c[3], 0.0);
      EXPECT_GT(det, 0.0);
    }
  }
}

TEST(DockPose, BeyondTheMeasuredRangeTheCornersGiveTheYawSpread) {
  const Scene s = RearScene();
  const SceneMarker m = Behind(s, 0.9, 0.05, 15.0 * kDeg);
  const MarkerPose corners = FitMarkerPose(Corners(s, m), Covariance(0.05), kFocal, s.mount, s.side, CornersOnly());
  const MarkerPose pose = FitMarkerPose(Corners(s, m), Covariance(0.05), kFocal, s.mount, s.side, PoseLimits{});
  ASSERT_TRUE(pose.valid && pose.yaw_valid);
  ASSERT_GT(Range(s, pose), PoseLimits{}.measured_yaw_range);
  EXPECT_EQ(pose.covariance, corners.covariance);
}

TEST(DockPose, TheRangeIsTheDistanceFromTheCamera) {
  // 0.84 m behind the camera, 0.2 m to the side: further than 0.85 m from it.
  const Scene s = RearScene();
  const SceneMarker m = Behind(s, 0.84, 0.2, 0.0);
  const MarkerPose corners = FitMarkerPose(Corners(s, m), Covariance(0.05), kFocal, s.mount, s.side, CornersOnly());
  const MarkerPose pose = FitMarkerPose(Corners(s, m), Covariance(0.05), kFocal, s.mount, s.side, PoseLimits{});
  ASSERT_TRUE(pose.valid && pose.yaw_valid);
  EXPECT_GT(Range(s, pose), 0.85);
  EXPECT_EQ(pose.covariance[8], corners.covariance[8]);
}

TEST(DockPose, AYawThatDoesNotCountStaysWithoutSpreadWithinTheRange) {
  // Seen from the side with rough corners the mirror image fits as well, also within the measured range.
  const Scene s = RearScene();
  bool seen = false;
  for (const double sigma_px : {0.5, 1.0, 2.0, 4.0}) {
    const SceneMarker m = Behind(s, 0.8, 0.0, 25.0 * kDeg);
    const MarkerPose pose = FitMarkerPose(Corners(s, m), Covariance(sigma_px), kFocal, s.mount, s.side, PoseLimits{});
    if (!pose.valid || pose.yaw_valid) continue;
    seen = true;
    ASSERT_LE(Range(s, pose), PoseLimits{}.measured_yaw_range);
    for (const size_t i : {2U, 5U, 6U, 7U, 8U}) EXPECT_EQ(pose.covariance[i], 0.0) << i;
  }
  EXPECT_TRUE(seen) << "no corner spread made the yaw uncertain";
}

TEST(DockPose, RejectsWhatIsNoUprightSquareOfThatSide) {
  const Scene s = RearScene();
  // Leaning back by 20 degrees.
  SceneMarker m = Behind(s, 0.3, 0.0, 0.0);
  m.lean = 20.0 * kDeg;
  MarkerPose pose = FitMarkerPose(Corners(s, m), Covariance(0.05), kFocal, s.mount, s.side, PoseLimits{});
  EXPECT_FALSE(pose.valid);
  EXPECT_GT(pose.fit_error_px, 2.5);
  // One corner off by a twentieth of the side.
  m.lean = 0.0;
  std::array<Point2, 4> corners = Corners(s, m);
  corners[2].x += 0.05 * (corners[2].x - corners[3].x);
  pose = FitMarkerPose(corners, Covariance(0.05), kFocal, s.mount, s.side, PoseLimits{});
  EXPECT_FALSE(pose.valid);
  // A wider limit lets it pass, with a covariance widened by the misfit.
  PoseLimits wide;
  wide.fit_error_share = 0.05;
  const MarkerPose exact = FitMarkerPose(Corners(s, m), Covariance(0.05), kFocal, s.mount, s.side, wide);
  pose = FitMarkerPose(corners, Covariance(0.05), kFocal, s.mount, s.side, wide);
  ASSERT_TRUE(pose.valid && exact.valid);
  EXPECT_GT(pose.covariance[0], 100.0 * exact.covariance[0]);
}

TEST(DockPose, NeedsSideFocalLengthAndCornersInFrontOfTheCamera) {
  const Scene s = RearScene();
  const std::array<Point2, 4> corners = Corners(s, Behind(s, 0.5, 0.0, 0.0));
  EXPECT_FALSE(FitMarkerPose(corners, Covariance(0.05), kFocal, s.mount, 0.0, PoseLimits{}).valid);
  EXPECT_FALSE(FitMarkerPose(corners, Covariance(0.05), 0.0, s.mount, s.side, PoseLimits{}).valid);
  EXPECT_FALSE(FitMarkerPose(corners, std::array<double, 64>{}, kFocal, s.mount, s.side, PoseLimits{}).valid);
  const std::array<Point2, 4> point = {corners[0], corners[0], corners[0], corners[0]};
  EXPECT_FALSE(FitMarkerPose(point, Covariance(0.05), kFocal, s.mount, s.side, PoseLimits{}).valid);
}

}  // namespace
}  // namespace tron_vision
