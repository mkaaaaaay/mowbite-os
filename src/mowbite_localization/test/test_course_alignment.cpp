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

bool Usable(const CourseHeading& course) {
  return course.kind == CourseKind::kUsable;
}

// L8 and L9: the heading a course says.
TEST(L8HeadingFromCourse, ForwardItIsTheCourse) {
  const auto course = HeadingFromCourse(Velocity(0.3, 1.0), 0.3, 0.0, Eigen::Vector2d(0.1, 0.0), {});
  ASSERT_TRUE(Usable(course));
  EXPECT_NEAR(course.heading, 1.0, 1e-12);
}

TEST(L8HeadingFromCourse, BackwardsItIsTheOtherWay) {
  const auto course = HeadingFromCourse(Velocity(0.2, 1.0), -0.2, 0.0, Eigen::Vector2d(0.1, 0.0), {});
  ASSERT_TRUE(Usable(course));
  EXPECT_NEAR(course.heading, WrapAngle(1.0 + M_PI), 1e-12);
}

TEST(L8HeadingFromCourse, AnAntennaAheadSwingsOutInATurn) {
  // Facing east, turning left at 0.5 rad/s at 0.2 m/s: the antenna 0.2 m ahead also moves 0.1 m/s north.
  const double heading = 0.0;
  const Eigen::Vector2d velocity(0.2, 0.1);
  const auto result = HeadingFromCourse(velocity, 0.2, 0.5, Eigen::Vector2d(0.2, 0.0), {});
  ASSERT_TRUE(Usable(result));
  EXPECT_NEAR(result.heading, heading, 1e-12);
  // An antenna to the left of base_link moves slower in a left turn.
  const auto left = HeadingFromCourse(Eigen::Vector2d(0.15, 0.0), 0.2, 0.5, Eigen::Vector2d(0.0, 0.1), {});
  ASSERT_TRUE(Usable(left));
  EXPECT_NEAR(left.heading, heading, 1e-12);
}

TEST(L9HeadingFromCourse, TooSlowSaysNothing) {
  const Eigen::Vector2d none = Eigen::Vector2d::Zero();
  EXPECT_EQ(HeadingFromCourse(Velocity(0.09, 1.0), 0.3, 0.0, none, {}).kind, CourseKind::kTooSlow);
  EXPECT_EQ(HeadingFromCourse(Velocity(0.3, 1.0), 0.09, 0.0, none, {}).kind, CourseKind::kTooSlow);
  // Turning on the spot moves the antenna, but says nothing that can be trusted.
  EXPECT_EQ(HeadingFromCourse(Velocity(0.1, 1.0), 0.0, 0.5, Eigen::Vector2d(0.2, 0.0), {}).kind, CourseKind::kTooSlow);
}

TEST(L9HeadingFromCourse, AnAntennaNotAsFastAsTheWheelsIsInconsistent) {
  // The wheels spin at 0.3 m/s while the mower creeps sideways, or GNSS speed belongs to someone carrying it.
  const Eigen::Vector2d none = Eigen::Vector2d::Zero();
  EXPECT_EQ(HeadingFromCourse(Velocity(0.12, 1.0), 0.3, 0.0, none, {}).kind, CourseKind::kInconsistent);
  EXPECT_EQ(HeadingFromCourse(Velocity(0.6, 1.0), 0.3, 0.0, none, {}).kind, CourseKind::kInconsistent);
  // Wheels that report 6 % too little still count.
  EXPECT_TRUE(Usable(HeadingFromCourse(Velocity(0.3, 1.0), 0.282, 0.0, none, {})));
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
  double t = 0.0;
  for (double d : {88.0, 92.0, 90.0, 91.0}) EXPECT_FALSE(alignment.Add(d * kDeg, t += 0.2));
  const auto mean = alignment.Add(89.0 * kDeg, t += 0.2);
  ASSERT_TRUE(mean);
  EXPECT_NEAR(*mean, 90.0 * kDeg, 1e-9);
}

TEST(E1CourseAlignment, TheyMustAgreeWithinTenDegrees) {
  CourseAlignment alignment({});
  double t = 0.0;
  for (double d : {0.0, 15.0, 5.0, 3.0, 2.0, 4.0}) EXPECT_FALSE(alignment.Add(d * kDeg, t += 0.2));
  // The 15 degrees leave the window with the next one.
  const auto mean = alignment.Add(6.0 * kDeg, t += 0.2);
  ASSERT_TRUE(mean);
  EXPECT_NEAR(*mean, 4.0 * kDeg, 1e-9);
}

TEST(E1CourseAlignment, AcrossPiTheMeanIsRight) {
  CourseAlignment alignment({});
  std::optional<double> mean;
  double t = 0.0;
  for (double d : {178.0, -178.0, 179.0, -179.0, 180.0}) mean = alignment.Add(d * kDeg, t += 0.2);
  ASSERT_TRUE(mean);
  EXPECT_NEAR(std::abs(*mean), M_PI, 1e-9);
}

TEST(E1CourseAlignment, ClearBreaksTheRun) {
  CourseAlignment alignment({});
  double t = 0.0;
  for (int i = 0; i < 4; ++i) alignment.Add(0.5, t += 0.2);
  alignment.Clear();
  EXPECT_FALSE(alignment.Add(0.5, t += 0.2));
  for (int i = 0; i < 3; ++i) EXPECT_FALSE(alignment.Add(0.5, t += 0.2));
  EXPECT_TRUE(alignment.Add(0.5, t += 0.2));
}

TEST(E1CourseAlignment, OldOnesLeaveTheRun) {
  CourseAlignment alignment({});
  double t = 0.0;
  for (int i = 0; i < 4; ++i) alignment.Add(0.5, t += 0.2);
  // After 30 s the four are too old: the gyro may have drifted since.
  EXPECT_FALSE(alignment.Add(0.5, t += 30.0));
  for (int i = 0; i < 3; ++i) EXPECT_FALSE(alignment.Add(0.5, t += 0.2));
  EXPECT_TRUE(alignment.Add(0.5, t += 0.2));
}

// E5: a course counts only along the track of the fixes.
TEST(E5CourseTrack, NeedsATenthOfAMetreWithinTheWindow) {
  CourseTrack track({});
  const Eigen::Vector2d east(0.3, 0.0);
  EXPECT_EQ(track.Check(east), TrackCheck::kNoTrack);
  track.Add({0.0, 0.0}, 0.0);
  track.Add({0.06, 0.0}, 0.2);
  EXPECT_EQ(track.Check(east), TrackCheck::kNoTrack);
  track.Add({0.12, 0.0}, 0.4);
  EXPECT_EQ(track.Check(east), TrackCheck::kAgrees);
  // 0.1 m over more than 1.5 s is no track.
  CourseTrack slow({});
  for (int i = 0; i <= 8; ++i) slow.Add({0.0125 * i, 0.0}, 0.2 * i);
  EXPECT_EQ(slow.Check(east), TrackCheck::kNoTrack);
}

TEST(E5CourseTrack, ThirtyDegreesBothWays) {
  CourseTrack track({});
  track.Add({0.0, 0.0}, 0.0);
  track.Add({0.2, 0.0}, 0.4);
  const auto at = [](double deg) { return Eigen::Vector2d(0.3 * std::cos(deg * kDeg), 0.3 * std::sin(deg * kDeg)); };
  EXPECT_EQ(track.Check(at(25.0)), TrackCheck::kAgrees);
  EXPECT_EQ(track.Check(at(-25.0)), TrackCheck::kAgrees);
  EXPECT_EQ(track.Check(at(35.0)), TrackCheck::kContradicts);
  EXPECT_EQ(track.Check(at(-35.0)), TrackCheck::kContradicts);
  // The wrong course of the LG290P: the right speed, 160 deg off.
  EXPECT_EQ(track.Check(at(160.0)), TrackCheck::kContradicts);
  // Backwards the antenna goes the way of its velocity as well.
  CourseTrack back({});
  back.Add({0.0, 0.0}, 0.0);
  back.Add({-0.2, 0.0}, 0.4);
  EXPECT_EQ(back.Check(at(180.0)), TrackCheck::kAgrees);
}

TEST(E5CourseTrack, TheLatestChordFarEnough) {
  CourseTrack track({});
  // North, then east: the chord from the last fix 0.1 m back says east.
  track.Add({0.0, 0.0}, 0.0);
  track.Add({0.0, 0.3}, 0.6);
  track.Add({0.06, 0.3}, 0.8);
  track.Add({0.12, 0.3}, 1.0);
  EXPECT_EQ(track.Check({0.3, 0.0}), TrackCheck::kAgrees);
  EXPECT_EQ(track.Check({0.0, 0.3}), TrackCheck::kContradicts);
}

TEST(E5CourseTrack, ClearAndTimeBackStartAgain) {
  CourseTrack track({});
  track.Add({0.0, 0.0}, 1.0);
  track.Add({0.2, 0.0}, 1.4);
  track.Clear();
  track.Add({0.4, 0.0}, 1.6);
  EXPECT_EQ(track.Check({0.3, 0.0}), TrackCheck::kNoTrack);
  track.Add({0.6, 0.0}, 2.0);
  track.Add({0.0, 0.0}, 0.5);
  EXPECT_EQ(track.Check({0.3, 0.0}), TrackCheck::kNoTrack);
}

}  // namespace
