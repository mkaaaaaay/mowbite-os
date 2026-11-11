#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

#include "mowbite_logic/direct_route.h"
#include "mowbite_logic/undocking.h"

namespace mowbite_logic {
namespace {

// The Tron: its front corners 0.644 m from base_link, 0.15 m of margin, so 0.794 m have to be free.
constexpr double kRadius = 0.644;
constexpr double kMargin = 0.15;
constexpr double kClear = kRadius + kMargin;

Ring Box(double x0, double y0, double x1, double y1) {
  return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
}

const Ring kLawn = Box(0, 0, 10, 10);

DirectRouteCheck Check(const Point2& from, const Point2& to, const std::vector<Ring>& drivable,
                       const std::vector<Ring>& obstacles = {}) {
  return CheckDirectRoute(from, to, kRadius, kMargin, drivable, obstacles);
}

TEST(DirectRoute, AWayThroughTheMiddleOfAnAreaIsFree) {
  const DirectRouteCheck check = Check({2, 5}, {8, 5}, {kLawn});
  EXPECT_TRUE(check.free) << check.why;
  EXPECT_TRUE(check.why.empty());
  EXPECT_TRUE(Check({2, 2}, {8, 7}, {kLawn}).free);
}

TEST(DirectRoute, NeverFreeWithoutTheSizeOfTheBodyOrAMargin) {
  const double no_number = std::nan("");
  const double endless = HUGE_VAL;
  const std::vector<std::pair<double, double>> sizes = {
      {0, kMargin}, {-1, kMargin}, {no_number, kMargin}, {endless, kMargin}, {kRadius, 0}, {kRadius, -0.1},
      {kRadius, no_number}};
  for (const auto& [radius, margin] : sizes) {
    const DirectRouteCheck check = CheckDirectRoute({2, 5}, {8, 5}, radius, margin, {kLawn}, {});
    EXPECT_FALSE(check.free) << radius << " " << margin;
    EXPECT_EQ(check.why, "the size of the body is not known");
  }
  EXPECT_FALSE(Check({2, 5}, {no_number, 5}, {kLawn}).free);
  EXPECT_FALSE(Check({endless, 5}, {8, 5}, {kLawn}).free);
}

TEST(DirectRoute, TheStartHasToLieInAnAreaAndInNoObstacle) {
  DirectRouteCheck check = Check({-1, 5}, {8, 5}, {kLawn});
  EXPECT_FALSE(check.free);
  EXPECT_EQ(check.why, "the mower stands outside of the mowing and navigation areas");
  EXPECT_FALSE(Check({2, 5}, {8, 5}, {}).free);
  // An obstacle so large that none of its edges is near the way.
  check = Check({5, 5}, {5.5, 5}, {Box(-20, -20, 30, 30)}, {Box(-10, -10, 20, 20)});
  EXPECT_FALSE(check.free);
  EXPECT_EQ(check.why, "the mower stands in an obstacle");
}

TEST(DirectRoute, AnObstacleBesideTheWayNeedsTheWholeClearance) {
  const auto beside = [](double gap) { return Box(4, 5 + gap, 6, 6 + gap); };
  const DirectRouteCheck near = Check({2, 5}, {8, 5}, {kLawn}, {beside(kClear - 0.01)});
  EXPECT_FALSE(near.free);
  EXPECT_EQ(near.why, "an obstacle 0.78 m from the way at (4.00, 5.78), less than 0.80 m");
  // The samples keep 1 cm more than needed: just beyond the clearance still counts as too close.
  EXPECT_FALSE(Check({2, 5}, {8, 5}, {kLawn}, {beside(kClear + 0.005)}).free);
  EXPECT_TRUE(Check({2, 5}, {8, 5}, {kLawn}, {beside(kClear + 0.03)}).free);
  // On the other side, and against the direction.
  EXPECT_FALSE(Check({8, 5}, {2, 5}, {kLawn}, {Box(4, 3.3, 6, 4.3)}).free);
  EXPECT_TRUE(Check({8, 5}, {2, 5}, {kLawn}, {Box(4, 3.0, 6, 4.1)}).free);
}

TEST(DirectRoute, AnObstacleAcrossTheWayBlocksIt) {
  const DirectRouteCheck check = Check({2, 5}, {8, 5}, {kLawn}, {Box(4.9, 3, 5.1, 7)});
  EXPECT_FALSE(check.free);
  EXPECT_NE(check.why.find("an obstacle 0.00 m from the way"), std::string::npos) << check.why;
  // A corner that reaches into the lane.
  EXPECT_FALSE(Check({2, 5}, {8, 5}, {kLawn}, {{{5, 5.5}, {6, 7}, {4, 7}}}).free);
}

TEST(DirectRoute, TheCirclesAtBothEndsHaveToBeFreeToo) {
  // Behind the goal and before the start the body swings as far as beside the way.
  EXPECT_FALSE(Check({2, 5}, {8, 5}, {kLawn}, {Box(8.5, 4.5, 9.5, 5.5)}).free);
  EXPECT_TRUE(Check({2, 5}, {8, 5}, {kLawn}, {Box(8.9, 4.5, 9.5, 5.5)}).free);
  EXPECT_FALSE(Check({2, 5}, {8, 5}, {kLawn}, {Box(0.5, 4.5, 1.5, 5.5)}).free);
  EXPECT_TRUE(Check({2, 5}, {8, 5}, {kLawn}, {Box(0.5, 4.5, 1.1, 5.5)}).free);
  // Diagonally off the end: 0.6 m in x and in y is 0.85 m away, 0.5 m each is 0.71 m.
  EXPECT_TRUE(Check({2, 5}, {8, 5}, {kLawn}, {Box(8.6, 5.6, 9, 6)}).free);
  EXPECT_FALSE(Check({2, 5}, {8, 5}, {kLawn}, {Box(8.5, 5.5, 9, 6)}).free);
}

TEST(DirectRoute, TheEdgeOfTheAreasBlocksAtTheStartInTheMiddleAndAtTheGoal) {
  DirectRouteCheck check = Check({0.5, 5}, {8, 5}, {kLawn});
  EXPECT_FALSE(check.free);
  EXPECT_NE(check.why.find("the areas end 0.50 m from the way at (0.00, 5.00)"), std::string::npos) << check.why;
  EXPECT_FALSE(Check({2, 5}, {9.5, 5}, {kLawn}).free);
  EXPECT_TRUE(Check({0.85, 5}, {9.15, 5}, {kLawn}).free);
  // A notch from above that ends 0.5 m beside the way.
  const Ring notched = {{0, 0}, {10, 0}, {10, 10}, {6, 10}, {6, 5.5}, {4, 5.5}, {4, 10}, {0, 10}};
  EXPECT_FALSE(Check({2, 5}, {8, 5}, {notched}).free);
  EXPECT_TRUE(Check({2, 4.5}, {8, 4.5}, {notched}).free);
}

TEST(DirectRoute, AWayAcrossTheNotchOfAnLShapeIsNotFree) {
  const Ring l_shape = {{0, 0}, {10, 0}, {10, 4}, {4, 4}, {4, 10}, {0, 10}};
  EXPECT_FALSE(Check({2, 8}, {8, 2}, {l_shape}).free);
  EXPECT_TRUE(Check({2, 8}, {2, 2}, {l_shape}).free);
  EXPECT_TRUE(Check({2, 2}, {8, 2}, {l_shape}).free);
}

TEST(DirectRoute, OverlappingAreasCountAsOne) {
  EXPECT_TRUE(Check({2, 5}, {10, 5}, {Box(0, 0, 6, 10), Box(5, 0, 12, 10)}).free);
  // An area inside another one: its edges end nothing.
  EXPECT_TRUE(Check({2, 5}, {8, 5}, {kLawn, Box(3, 4, 7, 6)}).free);
  EXPECT_TRUE(Check({2, 5}, {8, 5}, {Box(3, 4, 7, 6), kLawn}).free);
  // The overlap is narrower than the lane where the way crosses it.
  EXPECT_FALSE(Check({2, 5}, {10, 5}, {Box(0, 0, 6, 10), Box(5, 4.5, 12, 5.5)}).free);
}

TEST(DirectRoute, AreasThatOnlyTouchOrNearlyOverlapCountAsApart) {
  EXPECT_FALSE(Check({2, 5}, {10, 5}, {Box(0, 0, 6, 10), Box(6, 0, 12, 10)}).free);
  EXPECT_FALSE(Check({2, 5}, {10, 5}, {Box(0, 0, 6, 10), Box(6.01, 0, 12, 10)}).free);
  // 5 mm of overlap is below what the check takes for covered.
  EXPECT_FALSE(Check({2, 5}, {10, 5}, {Box(0, 0, 6, 10), Box(5.995, 0, 12, 10)}).free);
  EXPECT_TRUE(Check({2, 5}, {10, 5}, {Box(0, 0, 6, 10), Box(5.95, 0, 12, 10)}).free);
}

TEST(DirectRoute, AWayWithoutLengthChecksTheCircle) {
  EXPECT_TRUE(Check({5, 5}, {5, 5}, {kLawn}).free);
  EXPECT_FALSE(Check({0.5, 0.5}, {0.5, 0.5}, {kLawn}).free);
  EXPECT_FALSE(Check({5, 5}, {5, 5}, {kLawn}, {Box(5.5, 5.5, 6, 6)}).free);
  EXPECT_TRUE(Check({5, 5}, {5.05, 5}, {kLawn}, {Box(5.65, 5.6, 6, 6)}).free);
}

TEST(DirectRoute, ALargerBodyOrMarginNeedsMoreRoom) {
  const std::vector<Ring> obstacles = {Box(4, 5.85, 6, 7)};
  EXPECT_TRUE(CheckDirectRoute({2, 5}, {8, 5}, kRadius, kMargin, {kLawn}, obstacles).free);
  EXPECT_FALSE(CheckDirectRoute({2, 5}, {8, 5}, kRadius + 0.1, kMargin, {kLawn}, obstacles).free);
  EXPECT_FALSE(CheckDirectRoute({2, 5}, {8, 5}, kRadius, kMargin + 0.1, {kLawn}, obstacles).free);
}

TEST(DirectRoute, RingsWithFewerThanThreePointsStillBlockAsEdges) {
  EXPECT_FALSE(Check({2, 5}, {8, 5}, {kLawn}, {{{5, 5.3}}}).free);
  EXPECT_FALSE(Check({2, 5}, {8, 5}, {kLawn}, {{{5, 4}, {5, 6}}}).free);
  EXPECT_TRUE(Check({2, 5}, {8, 5}, {kLawn}, {{}}).free);
  EXPECT_TRUE(Check({2, 5}, {8, 5}, {kLawn, {}}).free);
}

// Random gardens of overlapping boxes with obstacles: wherever a way counts as free, every point the body can reach
// lies in an area and in no obstacle.
TEST(DirectRoute, AFreeWayNeverLetsTheBodyLeaveTheAreasOrTouchAnObstacle) {
  uint32_t state = 12345;
  const auto random = [&state](double low, double high) {
    state = state * 1664525u + 1013904223u;
    return low + (high - low) * (state >> 8) / 16777216.0;
  };
  int free_ways = 0;
  for (int scene = 0; scene < 600; scene++) {
    std::vector<Ring> drivable;
    std::vector<Ring> obstacles;
    const int areas = 1 + static_cast<int>(random(0, 3));
    for (int i = 0; i < areas; i++) {
      const double x = random(0, 6);
      const double y = random(0, 6);
      drivable.push_back(Box(x, y, x + random(3, 9), y + random(3, 9)));
    }
    const int blocks = static_cast<int>(random(0, 3));
    for (int i = 0; i < blocks; i++) {
      const double x = random(0, 12);
      const double y = random(0, 12);
      obstacles.push_back(Box(x, y, x + random(0.1, 1.5), y + random(0.1, 1.5)));
    }
    // From a point of the first area to a point of any of them.
    const auto inside = [&random](const Ring& box) {
      return Point2{random(box[0].x, box[2].x), random(box[0].y, box[2].y)};
    };
    const Point2 from = inside(drivable[0]);
    const Point2 to = inside(drivable[static_cast<size_t>(random(0, areas))]);
    if (!Check(from, to, drivable, obstacles).free) continue;
    free_ways++;
    for (int i = 0; i < 300; i++) {
      const double t = random(0, 1);
      const double angle = random(0, 2 * M_PI);
      const double reach = random(0, kClear);
      const double x = from.x + t * (to.x - from.x) + reach * std::cos(angle);
      const double y = from.y + t * (to.y - from.y) + reach * std::sin(angle);
      ASSERT_TRUE(Reachable(drivable, obstacles, x, y))
          << "scene " << scene << ": (" << x << ", " << y << ") on the way from (" << from.x << ", " << from.y
          << ") to (" << to.x << ", " << to.y << ")";
    }
  }
  // Enough of the scenes have a free way for the check above to mean something.
  EXPECT_GT(free_ways, 30);
}

}  // namespace
}  // namespace mowbite_logic
