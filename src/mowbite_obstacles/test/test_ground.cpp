#include <gtest/gtest.h>

#include <cmath>

#include "mowbite_obstacles/ground.h"

namespace mowbite_obstacles {
namespace {

constexpr double kDeg = M_PI / 180.0;
constexpr double kWheel = 0.1125;

Mount Level() {
  Mount mount;
  mount.x = 0.466;
  mount.z = 0.25 - kWheel;
  return mount;
}

Mount Tilted() {
  Mount mount = Level();
  mount.roll = 2.0 * kDeg;
  mount.pitch = 12.0 * kDeg;
  mount.yaw = 5.0 * kDeg;
  return mount;
}

Ray See(const Mount& mount, double x, double y, double z) {
  Ray ray;
  EXPECT_TRUE(RayOf(mount, {x, y, z}, &ray));
  return ray;
}

TEST(Ground, ALevelCameraLooksAlongXWithTheImageUpright) {
  const Vec3 ahead = RayInBase(Level(), {0.0, 0.0});
  EXPECT_NEAR(ahead.x, 1.0, 1e-12);
  EXPECT_NEAR(ahead.y, 0.0, 1e-12);
  EXPECT_NEAR(ahead.z, 0.0, 1e-12);
  const Vec3 right_and_down = RayInBase(Level(), {0.5, 0.25});
  EXPECT_NEAR(right_and_down.y, -0.5, 1e-12);
  EXPECT_NEAR(right_and_down.z, -0.25, 1e-12);
}

TEST(Ground, PitchUpwardsOfZeroLooksDown) {
  Mount mount = Level();
  mount.pitch = 10.0 * kDeg;
  EXPECT_LT(RayInBase(mount, {0.0, 0.0}).z, 0.0);
}

TEST(Ground, ARayAndItsPointOnTheGroundGoBothWays) {
  for (const Mount& mount : {Level(), Tilted()}) {
    for (const auto& [x, y] : {std::pair{1.0, 0.0}, {2.5, 0.8}, {1.2, -0.6}, {3.0, 1.5}}) {
      Vec3 hit;
      ASSERT_TRUE(HitGround(mount, FlatGround(kWheel), See(mount, x, y, -kWheel), &hit));
      EXPECT_NEAR(hit.x, x, 1e-9);
      EXPECT_NEAR(hit.y, y, 1e-9);
      EXPECT_NEAR(hit.z, -kWheel, 1e-9);
    }
  }
}

TEST(Ground, APointBehindTheCameraHasNoRay) {
  Ray ray;
  EXPECT_FALSE(RayOf(Level(), {0.0, 0.0, 0.0}, &ray));
}

TEST(Ground, ARayAtOrAboveTheHorizonMissesTheGround) {
  Vec3 hit;
  EXPECT_FALSE(HitGround(Level(), FlatGround(kWheel), {0.0, 0.0}, &hit));
  EXPECT_FALSE(HitGround(Level(), FlatGround(kWheel), {0.0, -0.2}, &hit));
  EXPECT_TRUE(HitGround(Level(), FlatGround(kWheel), {0.0, 0.2}, &hit));
}

TEST(Ground, ACameraUnderTheGroundSeesNoGround) {
  Mount mount = Level();
  mount.z = -0.2;
  Vec3 hit;
  EXPECT_FALSE(HitGround(mount, FlatGround(kWheel), {0.0, 0.3}, &hit));
}

// A standing cylinder at (cx, cy) as a camera boxes it: the lower edge at its near side, the corners at its sides in
// the row of the lower edge, the upper edge at its top on the near side.
BoxRays BoxOf(const Mount& mount, double cx, double cy, double r, double h) {
  const double dx = cx - mount.x, dy = cy - mount.y, d = std::hypot(dx, dy);
  const double ux = dx / d, uy = dy / d;
  BoxRays box;
  box.bottom = See(mount, cx - r * ux, cy - r * uy, -kWheel);
  box.top = See(mount, cx - r * ux, cy - r * uy, -kWheel + h);
  box.bottom_left = {See(mount, cx - r * uy, cy + r * ux, -kWheel).x, box.bottom.y};
  box.bottom_right = {See(mount, cx + r * uy, cy - r * ux, -kWheel).x, box.bottom.y};
  return box;
}

TEST(Ground, AFootprintHasTheMiddleTheRadiusAndTheHeightOfTheThing) {
  for (const Mount& mount : {Level(), Tilted()}) {
    const Footprint foot = FootprintOf(mount, FlatGround(kWheel), BoxOf(mount, 2.0, 0.3, 0.2, 0.4), {}, 5.0);
    ASSERT_EQ(foot.result, GroundResult::kOk);
    EXPECT_NEAR(foot.spot.x, 2.0, 0.02);
    EXPECT_NEAR(foot.spot.y, 0.3, 0.02);
    EXPECT_NEAR(foot.radius, 0.2, 0.02);
    EXPECT_NEAR(foot.height, 0.4, 1e-6);
  }
}

TEST(Ground, AFootprintFartherThanTheRangeIsTooFar) {
  const Footprint foot = FootprintOf(Level(), FlatGround(kWheel), BoxOf(Level(), 4.0, 0.0, 0.1, 0.2), {}, 3.0);
  EXPECT_EQ(foot.result, GroundResult::kTooFar);
  EXPECT_NEAR(foot.spot.x, 4.0, 0.02);
}

TEST(Ground, ALowerEdgeAboveTheHorizonHasNoFootprint) {
  BoxRays box;
  box.bottom = {0.0, -0.1};
  const Footprint foot = FootprintOf(Level(), FlatGround(kWheel), box, {}, 5.0);
  EXPECT_EQ(foot.result, GroundResult::kAboveHorizon);
  EXPECT_TRUE(std::isnan(foot.height));
}

TEST(Ground, TheSpreadOfAFootprintLiesAlongTheViewAndGrowsWithTheDistance) {
  const Errors errors{2.0 / 450.0, 0.5 * kDeg, 0.01};
  const Footprint near = FootprintOf(Level(), FlatGround(kWheel), BoxOf(Level(), 1.5, 0.0, 0.1, 0.2), errors, 5.0);
  const Footprint far = FootprintOf(Level(), FlatGround(kWheel), BoxOf(Level(), 3.0, 0.0, 0.1, 0.2), errors, 5.0);
  EXPECT_GT(near.spot.cov_xx, near.spot.cov_yy);
  EXPECT_GT(far.spot.cov_xx, near.spot.cov_xx);
  // 1 sigma along the view at 2.5 m ahead of the camera: some decimeters from the pitch alone.
  EXPECT_GT(std::sqrt(far.spot.cov_xx), 0.1);
  EXPECT_LT(std::sqrt(far.spot.cov_yy), 0.05);
  EXPECT_GE(far.spot.cov_xx * far.spot.cov_yy - far.spot.cov_xy * far.spot.cov_xy, 0.0);
}

TEST(Ground, WithoutErrorsAFootprintHasNoSpread) {
  const Footprint foot = FootprintOf(Tilted(), FlatGround(kWheel), BoxOf(Tilted(), 2.0, 0.5, 0.1, 0.2), {}, 5.0);
  EXPECT_EQ(foot.spot.cov_xx, 0.0);
  EXPECT_EQ(foot.spot.cov_yy, 0.0);
}

TEST(Ground, AStereoSpotIsWhereTheDepthPutsItWithItsSpreadAlongTheView) {
  const Mount mount = Tilted();
  const Vec3 point{2.2, 0.4, 0.05};
  const Ray ray = See(mount, point.x, point.y, point.z);
  const Vec3 direction = RayInBase(mount, ray);
  // The depth is the distance along the view, the ray has 1 there.
  const Vec3 v{point.x - mount.x, point.y - mount.y, point.z - mount.z};
  const double along = std::sqrt((v.x * v.x + v.y * v.y + v.z * v.z) /
                                  (direction.x * direction.x + direction.y * direction.y + direction.z * direction.z));
  const Spot spot = StereoSpot(mount, ray, along, 0.08, 0.001);
  EXPECT_NEAR(spot.x, point.x, 1e-9);
  EXPECT_NEAR(spot.y, point.y, 1e-9);
  EXPECT_GT(spot.cov_xx, 0.5 * 0.08 * 0.08);
  EXPECT_LT(spot.cov_yy, spot.cov_xx);
}

TEST(Ground, TwoAgreeingSpotsFuseToTheirWeightedMiddle) {
  const Spot a{2.0, 0.0, 0.04, 0.0, 0.01};
  const Spot b{2.2, 0.0, 0.04, 0.0, 0.01};
  const Fused fused = Fuse(a, b, 9.21);
  EXPECT_TRUE(fused.agrees);
  EXPECT_NEAR(fused.spot.x, 2.1, 1e-12);
  EXPECT_NEAR(fused.spot.cov_xx, 0.02, 1e-12);
  EXPECT_NEAR(fused.spot.cov_yy, 0.005, 1e-12);
}

TEST(Ground, TheSharperSpotWeighsMore) {
  const Spot sharp{2.0, 0.0, 0.0001, 0.0, 0.0001};
  const Spot vague{2.3, 0.0, 0.09, 0.0, 0.09};
  const Fused fused = Fuse(sharp, vague, 9.21);
  EXPECT_TRUE(fused.agrees);
  EXPECT_NEAR(fused.spot.x, 2.0, 0.001);
  EXPECT_LT(fused.spot.cov_xx, 0.0001);
}

TEST(Ground, DisagreeingSpotsKeepTheNearerWidenedByTheirDifference) {
  const Spot near{1.5, 0.0, 0.0004, 0.0, 0.0004};
  const Spot far{2.5, 0.2, 0.0004, 0.0, 0.0004};
  for (const Fused& fused : {Fuse(near, far, 9.21), Fuse(far, near, 9.21)}) {
    EXPECT_FALSE(fused.agrees);
    EXPECT_EQ(fused.spot.x, 1.5);
    EXPECT_NEAR(fused.spot.cov_xx, 0.0004 + 1.0, 1e-12);
    EXPECT_NEAR(fused.spot.cov_xy, 0.2, 1e-12);
  }
}

TEST(Ground, SpotsWithoutSpreadAgreeOnlyWhenEqual) {
  EXPECT_TRUE(Fuse({1.0, 1.0, 0, 0, 0}, {1.0, 1.0, 0, 0, 0}, 9.21).agrees);
  EXPECT_FALSE(Fuse({1.0, 1.0, 0, 0, 0}, {1.1, 1.0, 0, 0, 0}, 9.21).agrees);
}

TEST(Ground, TheViewOfALevelCameraStartsWhereTheLowerEdgeMeetsTheGround) {
  const double half_width = std::tan(43.0 * kDeg), half_height = std::tan(31.5 * kDeg);
  View view;
  ASSERT_TRUE(ViewOf(Level(), FlatGround(kWheel), {-half_width, 0.0}, {half_width, 0.0}, {0.0, half_height}, 4.0,
                     &view));
  EXPECT_NEAR(view.x, 0.466, 1e-12);
  EXPECT_NEAR(view.angle_left, 43.0 * kDeg, 1e-9);
  EXPECT_NEAR(view.angle_right, -43.0 * kDeg, 1e-9);
  EXPECT_NEAR(view.range_min, 0.25 / std::tan(31.5 * kDeg), 1e-9);
  EXPECT_EQ(view.range_max, 4.0);
}

TEST(Ground, ACameraThatSeesNoGroundHasNoView) {
  Mount mount = Level();
  mount.pitch = -40.0 * kDeg;
  View view;
  EXPECT_FALSE(ViewOf(mount, FlatGround(kWheel), {-0.9, 0.0}, {0.9, 0.0}, {0.0, 0.6}, 4.0, &view));
}

}  // namespace
}  // namespace mowbite_obstacles
