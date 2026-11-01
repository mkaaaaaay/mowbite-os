#include <gtest/gtest.h>

#include <cmath>

#include "mowbite_localization/course_alignment.h"
#include "mowbite_localization/pose_filter.h"

using namespace mowbite_localization;

namespace {

constexpr double kDeg = M_PI / 180.0;

Eigen::Vector2d Velocity(double speed, double course) {
  return {speed * std::cos(course), speed * std::sin(course)};
}

// L8 and L9: the heading a course says.
TEST(L8HeadingFromCourse, ForwardItIsTheCourse) {
  const auto heading = HeadingFromCourse(Velocity(0.3, 1.0), 0.3, 0.0, Eigen::Vector2d(0.1, 0.0), {});
  ASSERT_TRUE(heading);
  EXPECT_NEAR(*heading, 1.0, 1e-12);
}

TEST(L8HeadingFromCourse, BackwardsItIsTheOtherWay) {
  const auto heading = HeadingFromCourse(Velocity(0.2, 1.0), -0.2, 0.0, Eigen::Vector2d(0.1, 0.0), {});
  ASSERT_TRUE(heading);
  EXPECT_NEAR(*heading, WrapAngle(1.0 + M_PI), 1e-12);
}

TEST(L8HeadingFromCourse, AnAntennaAheadSwingsOutInATurn) {
  // Facing east, turning left at 0.5 rad/s at 0.2 m/s: the antenna 0.2 m ahead also moves 0.1 m/s north.
  const double heading = 0.0;
  const Eigen::Vector2d velocity(0.2, 0.1);
  const auto result = HeadingFromCourse(velocity, 0.2, 0.5, Eigen::Vector2d(0.2, 0.0), {});
  ASSERT_TRUE(result);
  EXPECT_NEAR(*result, heading, 1e-12);
  // An antenna to the left of base_link moves slower in a left turn.
  const auto left = HeadingFromCourse(Eigen::Vector2d(0.15, 0.0), 0.2, 0.5, Eigen::Vector2d(0.0, 0.1), {});
  ASSERT_TRUE(left);
  EXPECT_NEAR(*left, heading, 1e-12);
}

TEST(L9HeadingFromCourse, TooSlowGivesNothing) {
  EXPECT_FALSE(HeadingFromCourse(Velocity(0.09, 1.0), 0.3, 0.0, Eigen::Vector2d::Zero(), {}));
  EXPECT_FALSE(HeadingFromCourse(Velocity(0.3, 1.0), 0.09, 0.0, Eigen::Vector2d::Zero(), {}));
  // Turning on the spot moves the antenna, but says nothing that can be trusted.
  EXPECT_FALSE(HeadingFromCourse(Velocity(0.1, 1.0), 0.0, 0.5, Eigen::Vector2d(0.2, 0.0), {}));
}

TEST(L9HeadingFromCourse, AnAntennaNotAsFastAsTheWheelsGivesNothing) {
  // The wheels spin at 0.3 m/s while the mower creeps sideways, or GNSS speed belongs to someone carrying it.
  EXPECT_FALSE(HeadingFromCourse(Velocity(0.12, 1.0), 0.3, 0.0, Eigen::Vector2d::Zero(), {}));
  EXPECT_FALSE(HeadingFromCourse(Velocity(0.6, 1.0), 0.3, 0.0, Eigen::Vector2d::Zero(), {}));
  // Wheels that report 6 % too little still count.
  EXPECT_TRUE(HeadingFromCourse(Velocity(0.3, 1.0), 0.282, 0.0, Eigen::Vector2d::Zero(), {}));
}

TEST(L9HeadingFromCourse, VarianceShrinksWithSpeed) {
  CourseConfig config;
  EXPECT_NEAR(CourseVariance(0.2, config), 0.01, 1e-12);
  EXPECT_NEAR(CourseVariance(0.4, config), 0.0025, 1e-12);
  // Below the least speed it stays at the value of that speed.
  EXPECT_NEAR(CourseVariance(0.0, config), 0.04, 1e-12);
}

// E1: five agreeing differences in a row give the correction, from any start.
TEST(E1CourseAlignment, FiveInARowThatAgreeGiveTheirMean) {
  CourseAlignment alignment({});
  for (double d : {88.0, 92.0, 90.0, 91.0}) EXPECT_FALSE(alignment.Add(d * kDeg));
  const auto mean = alignment.Add(89.0 * kDeg);
  ASSERT_TRUE(mean);
  EXPECT_NEAR(*mean, 90.0 * kDeg, 1e-9);
}

TEST(E1CourseAlignment, TheyMustAgreeWithinTenDegrees) {
  CourseAlignment alignment({});
  for (double d : {0.0, 15.0, 5.0, 3.0, 2.0, 4.0}) EXPECT_FALSE(alignment.Add(d * kDeg));
  // The 15 degrees leave the window with the next one.
  const auto mean = alignment.Add(6.0 * kDeg);
  ASSERT_TRUE(mean);
  EXPECT_NEAR(*mean, 4.0 * kDeg, 1e-9);
}

TEST(E1CourseAlignment, AcrossPiTheMeanIsRight) {
  CourseAlignment alignment({});
  std::optional<double> mean;
  for (double d : {178.0, -178.0, 179.0, -179.0, 180.0}) mean = alignment.Add(d * kDeg);
  ASSERT_TRUE(mean);
  EXPECT_NEAR(std::abs(*mean), M_PI, 1e-9);
}

TEST(E1CourseAlignment, ClearBreaksTheRun) {
  CourseAlignment alignment({});
  for (int i = 0; i < 4; ++i) alignment.Add(0.5);
  alignment.Clear();
  EXPECT_FALSE(alignment.Add(0.5));
  for (int i = 0; i < 3; ++i) EXPECT_FALSE(alignment.Add(0.5));
  EXPECT_TRUE(alignment.Add(0.5));
}

}  // namespace
