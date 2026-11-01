#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "mowbite_follower/steering.h"
#include "paths.h"

using mowbite_follower::DesiredCourse;
using mowbite_follower::NormalizeAngle;
using mowbite_follower::Path;
using mowbite_follower::Projection;
using mowbite_follower::SteeringConfig;
using mowbite_follower::TurnRate;
using namespace mowbite_follower::test;

namespace {

Projection Foot(double heading, double cross, double curvature = 0) {
  Projection foot;
  foot.heading = heading;
  foot.cross = cross;
  foot.curvature = curvature;
  return foot;
}

}  // namespace

TEST(Steering, CourseTowardsThePathGrowsWithTheDistance) {
  const SteeringConfig config;
  EXPECT_NEAR(DesiredCourse(config, Foot(0.3, 0.0)), 0.3, 1e-12);
  // Left of the path: head right of it, half the approach angle at approach_distance.
  EXPECT_NEAR(DesiredCourse(config, Foot(0.0, 0.15)), -22.5 * kDeg, 1e-12);
  EXPECT_NEAR(DesiredCourse(config, Foot(0.0, -0.15)), 22.5 * kDeg, 1e-12);
  EXPECT_LT(DesiredCourse(config, Foot(0.0, 100.0)), -44.0 * kDeg);
  EXPECT_GT(DesiredCourse(config, Foot(0.0, 100.0)), -45.0 * kDeg);
}

TEST(Steering, OnThePathNoTurnOnALineTheArcsOwnOnAnArc) {
  const SteeringConfig config;
  EXPECT_NEAR(TurnRate(config, Foot(0.5, 0.0), 0.5, 0.3), 0.0, 1e-12);
  EXPECT_NEAR(TurnRate(config, Foot(0.5, 0.0, 1.0 / 0.44), 0.5, 0.11), 0.25, 1e-12);
  EXPECT_NEAR(TurnRate(config, Foot(0.5, 0.0, -1.0 / 0.44), 0.5, 0.11), -0.25, 1e-12);
  // Standing, only the heading error counts.
  EXPECT_NEAR(TurnRate(config, Foot(0.0, 0.0), 0.1, 0.0), -0.2, 1e-12);
}

namespace {

// A unicycle steered onto the line y = 0 heading east from (0, y0) facing yaw0, slower on the same curve where the
// turn rate would pass 0.3 rad/s, as the follower does; the largest |y| after the first crossing of the line, and |y|
// after length m.
std::pair<double, double> ApproachLine(double y0, double yaw0, double speed, double length) {
  const SteeringConfig config;
  double x = 0;
  double y = y0;
  double yaw = yaw0;
  const double dt = 0.05;
  double overshoot = 0;
  bool crossed = false;
  while (x < length) {
    double v = speed;
    double w = TurnRate(config, Foot(0.0, y), yaw, v);
    if (std::abs(w) > 0.3) {
      v *= 0.3 / std::abs(w);
      w = std::copysign(0.3, w);
    }
    yaw = NormalizeAngle(yaw + w * dt);
    x += v * dt * std::cos(yaw);
    y += v * dt * std::sin(yaw);
    if ((y > 0) != (y0 > 0)) crossed = true;
    if (crossed) overshoot = std::max(overshoot, std::abs(y));
  }
  return {overshoot, std::abs(y)};
}

}  // namespace

// Onto the line without swinging over it, and close after a few approach distances.
TEST(Steering, ConvergesWithoutOvershoot) {
  for (const double speed : {0.06, 0.15, 0.3}) {
    for (const double y0 : {0.05, 0.15, -0.3, 0.8}) {
      SCOPED_TRACE(::testing::Message() << speed << " m/s from " << y0 << " m");
      const auto [overshoot, after] = ApproachLine(y0, 0.0, speed, 4.0);
      EXPECT_LT(overshoot, 0.005);
      EXPECT_LT(after, 0.01);
    }
  }
  // Facing 40 deg towards the line from 0.15 m: it turns away in time.
  const auto [overshoot, after] = ApproachLine(0.15, -40.0 * kDeg, 0.3, 4.0);
  EXPECT_LT(overshoot, 0.03);
  EXPECT_LT(after, 0.01);
}
