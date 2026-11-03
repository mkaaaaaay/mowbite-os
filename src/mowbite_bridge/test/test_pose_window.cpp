#include <gtest/gtest.h>

#include <cmath>

#include "mowbite_bridge/pose_window.h"

using namespace mowbite_bridge;

// B12
TEST(PoseWindow, NothingWithoutAPose) {
  PoseWindow window;
  EXPECT_FALSE(window.Take());
  window.Add({1, 2, 0.5});
  ASSERT_TRUE(window.Take());
  EXPECT_FALSE(window.Take());
}

TEST(PoseWindow, EachAxisOnItsOwn) {
  PoseWindow window;
  window.Add({5, -1, 0.3});
  window.Add({1, -3, 0.1});
  window.Add({3, -2, 0.2});
  const auto pose = window.Take();
  ASSERT_TRUE(pose);
  EXPECT_DOUBLE_EQ(pose->x, 3);
  EXPECT_DOUBLE_EQ(pose->y, -2);
  EXPECT_NEAR(pose->heading, 0.2, 1e-12);
}

TEST(PoseWindow, EvenCountGivesTheUpperMiddle) {
  PoseWindow window;
  for (double x : {4.0, 1.0, 3.0, 2.0}) window.Add({x, -x, 0});
  const auto pose = window.Take();
  EXPECT_DOUBLE_EQ(pose->x, 3);
  EXPECT_DOUBLE_EQ(pose->y, -2);
}

TEST(PoseWindow, HeadingsAroundPiStayTogether) {
  PoseWindow window;
  const double d = 0.03;
  window.Add({0, 0, -M_PI + d});
  window.Add({0, 0, M_PI - d});
  window.Add({0, 0, M_PI - 2 * d});
  const auto pose = window.Take();
  EXPECT_NEAR(pose->heading, M_PI - d, 1e-9);
  // A plain middle value would be near 0, the opposite direction.
}

TEST(PoseWindow, OneOutlierDoesNotMoveTheMiddle) {
  PoseWindow window;
  for (int i = 0; i < 4; ++i) window.Add({1.0 + 0.001 * i, 2.0, 0.0});
  window.Add({40.0, -30.0, 3.0});
  const auto pose = window.Take();
  EXPECT_NEAR(pose->x, 1.002, 1e-9);
  EXPECT_DOUBLE_EQ(pose->y, 2.0);
  EXPECT_NEAR(pose->heading, 0.0, 1e-12);
}

TEST(PoseWindow, KeepsOnlyTheNewestBeyondItsCapacity) {
  PoseWindow window(3);
  for (double x : {100.0, 100.0, 1.0, 2.0, 3.0}) window.Add({x, 0, 0});
  EXPECT_DOUBLE_EQ(window.Take()->x, 2.0);
  PoseWindow none(0);
  none.Add({1, 1, 1});
  EXPECT_FALSE(none.Take());
}
