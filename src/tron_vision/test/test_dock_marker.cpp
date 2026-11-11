#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "dock_scene.h"
#include "tron_vision/dock_marker.h"
#include "tron_vision/dock_pose.h"

namespace tron_vision {
namespace {

using test::Behind;
using test::RearScene;
using test::Render;
using test::Scene;
using test::SceneImage;
using test::SceneMarker;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

Scene SceneWith(const SceneMarker& m) {
  Scene s = RearScene();
  s.markers = {m};
  return s;
}

MarkerParams Params() {
  MarkerParams params;
  params.pattern = test::kDockPattern;
  return params;
}

MarkerResult ResultOf(const Scene& s) {
  return DetectMarker(Render(s).View(), s.lens, Params()).result;
}

double WorstCorner(const MarkerDetection& found, const Scene& s) {
  const std::array<Point2, 4> truth = test::CornerPixels(s, s.markers[0]);
  double worst = 0.0;
  for (size_t k = 0; k < 4; k++) {
    worst = std::max(worst, std::hypot(found.corners_px[k].x - truth[k].x, found.corners_px[k].y - truth[k].y));
  }
  return worst;
}

TEST(DockMarker, FindsCornersAndPoseOverDistanceAndAngle) {
  struct View {
    double yaw;
    double lateral;  // per meter of distance
  };
  for (const double distance : {1.2, 0.8, 0.5, 0.3, 0.18}) {
    for (const View& view : {View{0.0, 0.0}, View{12.0 * kDeg, 0.12}, View{-20.0 * kDeg, -0.08}}) {
      SCOPED_TRACE(testing::Message() << "distance " << distance << " yaw " << view.yaw);
      const Scene s = SceneWith(Behind(RearScene(), distance, view.lateral * distance, view.yaw));
      const SceneMarker& m = s.markers[0];
      const MarkerDetection found = DetectMarker(Render(s).View(), s.lens, Params());
      ASSERT_EQ(found.result, MarkerResult::kFound);
      EXPECT_LT(WorstCorner(found, s), 0.12);
      EXPECT_NEAR(found.side_px, s.lens.fx * s.side / distance, 0.12 * s.lens.fx * s.side / distance);
      EXPECT_NEAR(found.white, 200.0, 8.0);
      EXPECT_NEAR(found.black, 30.0, 8.0);
      EXPECT_EQ(found.saturated, 0.0);
      EXPECT_LT(found.edge_px, 4.0);

      const MarkerPose pose =
          FitMarkerPose(found.corners, found.corner_covariance, s.lens.fx, s.mount, s.side, PoseLimits{});
      ASSERT_TRUE(pose.valid);
      EXPECT_NEAR(pose.x, m.x, 0.0003 + 0.0015 * distance * distance);
      EXPECT_NEAR(pose.y, m.y, 0.0003);
      EXPECT_LT(pose.fit_error_px, 0.1);
      if (distance <= 0.5) {
        EXPECT_TRUE(pose.yaw_valid);
        EXPECT_NEAR(pose.yaw, m.yaw, 0.5 * kDeg);
      }
      if (pose.yaw_valid) {
        EXPECT_NEAR(pose.yaw, m.yaw, 2.0 * kDeg);
      }
    }
  }
}

TEST(DockMarker, KeepsItsAccuracyInBadLight) {
  const Scene plain = SceneWith(Behind(RearScene(), 0.5, 0.03, 8.0 * kDeg));
  const Point2 mid = test::CornerPixels(plain, plain.markers[0])[0];
  std::vector<Scene> scenes(5, plain);
  scenes[0].noise = 5.0;
  scenes[1].blur = 1.8;
  scenes[2].exposure = 0.4;
  // A shadow with a hard edge right across the marker.
  scenes[3].shadow = 0.5;
  scenes[3].shadow_x = mid.x + 20.0;
  scenes[3].shadow_lean = 0.4;
  // Housing in the sun as bright as the white, and an image that is light all over.
  scenes[4].housing_luma = 190.0;
  scenes[4].background = 180.0;
  for (size_t i = 0; i < scenes.size(); i++) {
    SCOPED_TRACE(i);
    const Scene& s = scenes[i];
    const MarkerDetection found = DetectMarker(Render(s).View(), s.lens, Params());
    ASSERT_EQ(found.result, MarkerResult::kFound);
    EXPECT_LT(WorstCorner(found, s), 0.3);
    const MarkerPose pose =
        FitMarkerPose(found.corners, found.corner_covariance, s.lens.fx, s.mount, s.side, PoseLimits{});
    ASSERT_TRUE(pose.valid);
    EXPECT_NEAR(pose.x, s.markers[0].x, 0.002);
    EXPECT_NEAR(pose.y, s.markers[0].y, 0.0005);
  }
}

TEST(DockMarker, ReportedCovarianceMatchesTheScatterOverNoise) {
  Scene s = SceneWith(Behind(RearScene(), 0.6, 0.03, 10.0 * kDeg));
  s.noise = 6.0;
  const SceneMarker& m = s.markers[0];
  MarkerParams params = Params();
  // Only what the edge points show, so that the two can be compared.
  params.corner_sigma_floor_px = 0.0F;
  const std::vector<float> exposed = test::Expose(s);
  constexpr int kRuns = 30;
  std::array<double, 3> sum{};
  std::array<double, 3> squares{};
  std::array<double, 3> reported{};
  for (int run = 0; run < kRuns; run++) {
    const SceneImage image = test::Develop(exposed, s, 100 + static_cast<uint32_t>(run));
    const MarkerDetection found = DetectMarker(image.View(), s.lens, params);
    ASSERT_EQ(found.result, MarkerResult::kFound);
    const MarkerPose pose =
        FitMarkerPose(found.corners, found.corner_covariance, s.lens.fx, s.mount, s.side, PoseLimits{});
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
    EXPECT_GT(scatter / claimed, 0.5) << i;
    EXPECT_LT(scatter / claimed, 1.6) << i;
  }
}

TEST(DockMarker, TheFloorAddsToTheCornerScatter) {
  const Scene s = SceneWith(Behind(RearScene(), 0.5, 0.0, 0.0));
  const SceneImage image = Render(s);
  MarkerParams params = Params();
  params.corner_sigma_floor_px = 0.0F;
  const MarkerDetection bare = DetectMarker(image.View(), s.lens, params);
  params.corner_sigma_floor_px = 0.2F;
  const MarkerDetection floored = DetectMarker(image.View(), s.lens, params);
  ASSERT_EQ(bare.result, MarkerResult::kFound);
  ASSERT_EQ(floored.result, MarkerResult::kFound);
  EXPECT_LT(bare.corner_sigma_px, 0.05);
  EXPECT_NEAR(floored.corner_sigma_px, std::hypot(0.2, bare.corner_sigma_px), 1e-6);
}

TEST(DockMarker, ReadsOnlyTheOnePattern) {
  Scene s = SceneWith(Behind(RearScene(), 0.5, 0.0, 0.0));
  EXPECT_EQ(ResultOf(s), MarkerResult::kFound);
  // One cell different, on the marker or in what is asked for.
  for (int bit = 0; bit < 16; bit += 5) {
    s.markers[0].pattern = test::kDockPattern ^ static_cast<uint16_t>(1U << bit);
    EXPECT_EQ(ResultOf(s), MarkerResult::kNoMarker) << bit;
  }
  s.markers[0].pattern = test::kDockPattern;
  MarkerParams other = Params();
  other.pattern = 0xB533;
  EXPECT_EQ(DetectMarker(Render(s).View(), s.lens, other).result, MarkerResult::kNoMarker);
}

TEST(DockMarker, ReadsItUprightOnly) {
  Scene s = SceneWith(Behind(RearScene(), 0.5, 0.0, 0.0));
  for (int turns = 1; turns <= 3; turns++) {
    s.markers[0].quarter_turns = turns;
    EXPECT_EQ(ResultOf(s), MarkerResult::kNoMarker) << turns;
  }
  // Upright again, but the camera rolled by 40 degrees.
  s.markers[0].quarter_turns = 0;
  s.mount.roll = 40.0 * kDeg;
  EXPECT_EQ(ResultOf(s), MarkerResult::kNoMarker);
}

TEST(DockMarker, TwoMarkersAreNoMeasurement) {
  Scene s = SceneWith(Behind(RearScene(), 0.5, -0.1, 0.0));
  s.markers.push_back(Behind(s, 0.7, 0.15, 5.0 * kDeg));
  const MarkerDetection found = DetectMarker(Render(s).View(), s.lens, Params());
  EXPECT_EQ(found.result, MarkerResult::kAmbiguous);
  EXPECT_EQ(found.side_px, 0.0);
  // The second one with another pattern does not count.
  s.markers[1].pattern = 0x1234;
  EXPECT_EQ(ResultOf(s), MarkerResult::kFound);
}

TEST(DockMarker, AMarkerCutOffByTheImageIsReportedAsSuch) {
  // Close before the contacts the marker runs out at the top: first its white field, then the square itself.
  Scene s = SceneWith(Behind(RearScene(), 0.15, 0.0, 0.0));
  MarkerDetection found = DetectMarker(Render(s).View(), s.lens, Params());
  EXPECT_EQ(found.result, MarkerResult::kClipped);
  EXPECT_GT(test::CornerPixels(s, s.markers[0])[0].y, 20.0);
  // Three fifths of the square left.
  s.markers[0].z += 0.021;
  found = DetectMarker(Render(s).View(), s.lens, Params());
  EXPECT_EQ(found.result, MarkerResult::kClipped);
  EXPECT_NEAR(found.side_px, s.lens.fx * s.side / 0.15, 0.1 * s.lens.fx * s.side / 0.15);
  EXPECT_NEAR(found.white, 200.0, 10.0);
  // With a fifth of it left there is nothing to read.
  s.markers[0].z += 0.015;
  EXPECT_EQ(ResultOf(s), MarkerResult::kNoMarker);
  // Out to the side: three fifths left, then a fifth.
  s = SceneWith(Behind(RearScene(), 0.3, 0.202, 0.0));
  EXPECT_EQ(ResultOf(s), MarkerResult::kClipped);
  s = SceneWith(Behind(RearScene(), 0.3, 0.242, 0.0));
  EXPECT_EQ(ResultOf(s), MarkerResult::kNoMarker);
}

TEST(DockMarker, ACoveredMarkerIsNoMeasurement) {
  Scene s = SceneWith(Behind(RearScene(), 0.5, 0.0, 0.0));
  // The right half behind something gray.
  s.covers = {{0.0, 1.5, -1.5, 1.5, 90.0}};
  EXPECT_EQ(ResultOf(s), MarkerResult::kNoMarker);
  // A dark thing over one corner of the square and its field.
  s.covers = {{0.3, 0.9, 0.3, 0.9, 25.0}};
  EXPECT_EQ(ResultOf(s), MarkerResult::kNoMarker);
  // A leaf over one cell.
  s.covers = {{-0.3, -0.17, 0.2, 0.33, 120.0}};
  EXPECT_EQ(ResultOf(s), MarkerResult::kNoMarker);
  // A blade of grass across the bottom edge is left out of that edge: found, and still in the right place.
  s.covers = {{0.1, 0.13, -0.8, -0.3, 60.0}};
  const MarkerDetection found = DetectMarker(Render(s).View(), s.lens, Params());
  ASSERT_EQ(found.result, MarkerResult::kFound);
  EXPECT_LT(WorstCorner(found, s), 0.2);
}

TEST(DockMarker, TooLittleContrastOrSizeIsNoMarker) {
  Scene s = SceneWith(Behind(RearScene(), 0.5, 0.0, 0.0));
  s.white = 70.0;
  s.black = 50.0;
  EXPECT_EQ(ResultOf(s), MarkerResult::kNoMarker);
  s = SceneWith(Behind(RearScene(), 2.0, 0.0, 0.0));
  EXPECT_EQ(ResultOf(s), MarkerResult::kNoMarker);
}

TEST(DockMarker, ExposureFaultsAreNamed) {
  // The marker glows far above what the sensor takes: read, but not measured.
  Scene s = SceneWith(Behind(RearScene(), 0.5, 0.0, 0.0));
  s.exposure = 2.5;
  s.background = 4.0;
  MarkerDetection found = DetectMarker(Render(s).View(), s.lens, Params());
  EXPECT_EQ(found.result, MarkerResult::kTooBright);
  EXPECT_GT(found.side_px, 0.0);
  EXPECT_GT(found.saturated, 0.9);
  EXPECT_GT(found.image_saturated, 0.0005);
  EXPECT_LT(found.image_mean, 25.0);
  // No marker in an image that is all bright or all dark.
  s = RearScene();
  s.background = 240.0;
  found = DetectMarker(Render(s).View(), s.lens, Params());
  EXPECT_EQ(found.result, MarkerResult::kTooBright);
  EXPECT_EQ(found.side_px, 0.0);
  EXPECT_NEAR(found.image_mean, 240.0, 0.5);
  s.background = 8.0;
  EXPECT_EQ(ResultOf(s), MarkerResult::kTooDark);
  s.background = 110.0;
  EXPECT_EQ(ResultOf(s), MarkerResult::kNoMarker);
}

TEST(DockMarker, ABlurredMarkerIsReadButNotMeasured) {
  // Edges wider than half a cell.
  Scene s = SceneWith(Behind(RearScene(), 0.8, 0.0, 0.0));
  s.blur = 3.5;
  MarkerDetection found = DetectMarker(Render(s).View(), s.lens, Params());
  EXPECT_EQ(found.result, MarkerResult::kBlurred);
  EXPECT_GT(found.edge_px, 0.5 * found.side_px / 6.0);
  // The same blur on a near marker, as from a lens out of focus there, leaves its large cells apart.
  s = SceneWith(Behind(RearScene(), 0.2, 0.0, 0.0));
  s.blur = 3.5;
  found = DetectMarker(Render(s).View(), s.lens, Params());
  EXPECT_EQ(found.result, MarkerResult::kFound);
  EXPECT_LT(WorstCorner(found, s), 0.2);
}

TEST(DockMarker, AMarkerThatLeansDoesNotPassTheFit) {
  Scene s = SceneWith(Behind(RearScene(), 0.3, 0.0, 0.0));
  s.markers[0].lean = 20.0 * kDeg;
  const MarkerDetection found = DetectMarker(Render(s).View(), s.lens, Params());
  ASSERT_EQ(found.result, MarkerResult::kFound);
  EXPECT_FALSE(FitMarkerPose(found.corners, found.corner_covariance, s.lens.fx, s.mount, s.side, PoseLimits{}).valid);
}

TEST(DockMarker, WorksOnASmallImageFromItsFirstLevel) {
  // A quarter of the size in both directions, searched from the image itself on.
  Scene s = SceneWith(Behind(RearScene(), 0.4, 0.02, 5.0 * kDeg));
  s.lens.fx /= 4.0;
  s.lens.fy /= 4.0;
  s.lens.cx = (s.lens.cx - 1.5) / 4.0;
  s.lens.cy = (s.lens.cy - 1.5) / 4.0;
  s.width = 480;
  s.height = 270;
  MarkerParams params = Params();
  params.first_level = 0;
  params.last_level = 2;
  params.min_side_px = 20.0F;
  const MarkerDetection found = DetectMarker(Render(s).View(), s.lens, params);
  ASSERT_EQ(found.result, MarkerResult::kFound);
  EXPECT_LT(WorstCorner(found, s), 0.1);
}

TEST(DockMarker, HonorsTheStrideOfTheImage) {
  const Scene s = SceneWith(Behind(RearScene(), 0.5, 0.02, 5.0 * kDeg));
  const SceneImage image = Render(s);
  const uint32_t stride = image.width + 64;
  std::vector<uint8_t> padded(static_cast<size_t>(stride) * image.height, 255);
  for (uint32_t y = 0; y < image.height; y++) {
    std::copy_n(image.px.begin() + static_cast<std::ptrdiff_t>(static_cast<size_t>(y) * image.width), image.width,
                padded.begin() + static_cast<std::ptrdiff_t>(static_cast<size_t>(y) * stride));
  }
  const MarkerDetection plain = DetectMarker(image.View(), s.lens, Params());
  const MarkerDetection wide = DetectMarker({padded.data(), image.width, image.height, stride}, s.lens, Params());
  ASSERT_EQ(plain.result, MarkerResult::kFound);
  ASSERT_EQ(wide.result, MarkerResult::kFound);
  for (size_t k = 0; k < 4; k++) {
    EXPECT_EQ(plain.corners_px[k].x, wide.corners_px[k].x);
    EXPECT_EQ(plain.corners_px[k].y, wide.corners_px[k].y);
  }
}

TEST(DockMarker, RefusesWhatIsNoImage) {
  const Scene s = SceneWith(Behind(RearScene(), 0.5, 0.0, 0.0));
  const SceneImage image = Render(s);
  EXPECT_EQ(DetectMarker({nullptr, 1920, 1080, 1920}, s.lens, Params()).result, MarkerResult::kNoMarker);
  EXPECT_EQ(DetectMarker({image.px.data(), 32, 32, 1920}, s.lens, Params()).result, MarkerResult::kNoMarker);
  EXPECT_EQ(DetectMarker({image.px.data(), 1920, 1080, 1000}, s.lens, Params()).result, MarkerResult::kNoMarker);
  EXPECT_EQ(DetectMarker(image.View(), LensModel{}, Params()).result, MarkerResult::kNoMarker);
}

}  // namespace
}  // namespace tron_vision
