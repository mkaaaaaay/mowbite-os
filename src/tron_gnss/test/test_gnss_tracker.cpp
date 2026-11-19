#include <gtest/gtest.h>

#include <cmath>
#include <optional>

#include "tron_gnss/gnss_tracker.h"

namespace tron {
namespace {

// Without a time each GGA is 0.2 s after the one before, as from a receiver at 5 Hz. Corrections 1 s old by default.
NmeaMessage Gga(int quality, double lat, double lon, double alt = 60.0, double utc = -1.0,
                std::optional<double> age = 1.0) {
  static double next_utc = 43200.0;
  if (utc < 0) utc = next_utc += 0.2;
  NmeaMessage m;
  m.type = NmeaMessage::Type::kGga;
  m.gga.quality = quality;
  m.gga.lat_deg = lat;
  m.gga.lon_deg = lon;
  m.gga.altitude_m = alt;
  m.gga.utc_s = utc;
  m.gga.satellites = 18;
  m.gga.correction_age_s = age;
  return m;
}

NmeaMessage Vtg(double speed, double course, bool valid = true) {
  NmeaMessage m;
  m.type = NmeaMessage::Type::kVtg;
  m.vtg.valid = valid;
  m.vtg.speed_mps = speed;
  m.vtg.course_deg = course;
  return m;
}

NmeaMessage Rmc(double speed, double course, bool valid = true) {
  NmeaMessage m;
  m.type = NmeaMessage::Type::kRmc;
  m.rmc.valid = valid;
  m.rmc.speed_mps = speed;
  m.rmc.course_deg = course;
  return m;
}

GnssConfig ReceiverMotion() {
  GnssConfig config;
  config.motion_from_fixes = false;
  return config;
}

// The motion vector from speed and course of the receiver; the poses themselves do not depend on it.
class TrackerTest : public ::testing::Test {
 protected:
  GnssTracker tracker{ReceiverMotion(), LocalFrame(52.0, 9.0, 60.0)};
};

TEST_F(TrackerTest, FixTypesGiveSolutionAndAccuracy) {
  auto fixed = tracker.Update(Gga(4, 52.0, 9.0), 0);
  ASSERT_TRUE(fixed);
  EXPECT_EQ(fixed->solution, Solution::kRtkFixed);
  EXPECT_DOUBLE_EQ(fixed->accuracy_m, 0.02);

  auto flt = tracker.Update(Gga(5, 52.0, 9.0), 0);
  ASSERT_TRUE(flt);
  EXPECT_EQ(flt->solution, Solution::kRtkFloat);
  EXPECT_DOUBLE_EQ(flt->accuracy_m, 0.3);

  auto dgps = tracker.Update(Gga(2, 52.0, 9.0), 0);
  ASSERT_TRUE(dgps);
  EXPECT_EQ(dgps->solution, Solution::kDgps);
  EXPECT_DOUBLE_EQ(dgps->accuracy_m, 1.0);

  auto single = tracker.Update(Gga(1, 52.0, 9.0), 0);
  ASSERT_TRUE(single);
  EXPECT_EQ(single->solution, Solution::kSingle);
  EXPECT_DOUBLE_EQ(single->accuracy_m, 3.0);

  auto dr = tracker.Update(Gga(6, 52.0, 9.0), 0);
  ASSERT_TRUE(dr);
  EXPECT_EQ(dr->solution, Solution::kDeadReckoning);
}

TEST_F(TrackerTest, NoPoseWithoutFixOrPosition) {
  EXPECT_FALSE(tracker.Update(Gga(0, 52.0, 9.0), 0));
  EXPECT_FALSE(tracker.Update(Gga(3, 52.0, 9.0), 0));
  EXPECT_FALSE(tracker.Update(Gga(8, 52.0, 9.0), 0));
  NmeaMessage empty = Gga(4, 0, 0);
  empty.gga.lat_deg.reset();
  EXPECT_FALSE(tracker.Update(empty, 0));
  empty = Gga(4, 0, 0);
  empty.gga.lon_deg.reset();
  EXPECT_FALSE(tracker.Update(empty, 0));
}

TEST_F(TrackerTest, MotionMessagesAloneGiveNoPose) {
  EXPECT_FALSE(tracker.Update(Vtg(1.0, 90.0), 0));
  EXPECT_FALSE(tracker.Update(Rmc(1.0, 90.0), 0));
}

TEST_F(TrackerTest, PositionIsEnuFromTheDatum) {
  auto p = tracker.Update(Gga(4, 52.0003, 9.0004, 61.0, 43200.5), 0);
  ASSERT_TRUE(p);
  EXPECT_GT(p->position.north, 30.0);
  EXPECT_GT(p->position.east, 20.0);
  EXPECT_NEAR(p->position.up, 1.0, 1e-3);
  EXPECT_EQ(p->sensor_stamp_ms, 43200500u);
  EXPECT_EQ(p->satellites, 18);
}

TEST_F(TrackerTest, CourseNorthEastSouthWest) {
  struct Case {
    double course, east, north, heading;
  } cases[] = {
      {0.0, 0.0, 2.0, M_PI / 2},
      {90.0, 2.0, 0.0, 0.0},
      {180.0, 0.0, -2.0, -M_PI / 2},
      {270.0, -2.0, 0.0, M_PI},
      {45.0, std::sqrt(2.0), std::sqrt(2.0), M_PI / 4},
  };
  for (const auto& c : cases) {
    tracker.Update(Vtg(2.0, c.course), 0);
    auto p = tracker.Update(Gga(4, 52.0, 9.0), 0.1);
    ASSERT_TRUE(p) << c.course;
    EXPECT_TRUE(p->motion_valid) << c.course;
    EXPECT_NEAR(p->motion_east_mps, c.east, 1e-9) << c.course;
    EXPECT_NEAR(p->motion_north_mps, c.north, 1e-9) << c.course;
    EXPECT_NEAR(std::cos(p->motion_heading), std::cos(c.heading), 1e-9) << c.course;
    EXPECT_NEAR(std::sin(p->motion_heading), std::sin(c.heading), 1e-9) << c.course;
  }
}

TEST_F(TrackerTest, RmcWorksLikeVtg) {
  tracker.Update(Rmc(1.0, 90.0), 0);
  auto p = tracker.Update(Gga(4, 52.0, 9.0), 0.1);
  ASSERT_TRUE(p);
  EXPECT_TRUE(p->motion_valid);
  EXPECT_NEAR(p->motion_east_mps, 1.0, 1e-9);
}

TEST_F(TrackerTest, StaleMotionIsNotUsed) {
  tracker.Update(Vtg(1.0, 90.0), 0);
  auto p = tracker.Update(Gga(4, 52.0, 9.0), 1.5);
  ASSERT_TRUE(p);
  EXPECT_FALSE(p->motion_valid);
}

TEST_F(TrackerTest, InvalidMotionRemovesEarlierMotion) {
  tracker.Update(Vtg(1.0, 90.0), 0);
  tracker.Update(Vtg(1.0, 90.0, false), 0.1);
  auto p = tracker.Update(Gga(4, 52.0, 9.0), 0.2);
  ASSERT_TRUE(p);
  EXPECT_FALSE(p->motion_valid);
}

TEST_F(TrackerTest, RepeatedGgaGivesNoPose) {
  EXPECT_TRUE(tracker.Update(Gga(4, 52.0, 9.0, 60.0, 50000.0), 0));
  EXPECT_FALSE(tracker.Update(Gga(4, 52.0, 9.0, 60.0, 50000.0), 0.2));
  EXPECT_FALSE(tracker.Update(Gga(4, 52.0, 9.0, 60.0, 50000.0), 0.4));
  EXPECT_EQ(tracker.stale_gga(), 2u);
  EXPECT_TRUE(tracker.Update(Gga(4, 52.0, 9.0, 60.0, 50000.2), 0.6));
}

TEST_F(TrackerTest, GgaTimeAcrossMidnightAdvances) {
  EXPECT_TRUE(tracker.Update(Gga(4, 52.0, 9.0, 60.0, 86399.8), 0));
  EXPECT_TRUE(tracker.Update(Gga(4, 52.0, 9.0, 60.0, 0.0), 0.2));
  EXPECT_EQ(tracker.stale_gga(), 0u);
}

TEST_F(TrackerTest, StepBackIsDroppedOnceThenFollowed) {
  EXPECT_TRUE(tracker.Update(Gga(4, 52.0, 9.0, 60.0, 50000.0), 0));
  EXPECT_FALSE(tracker.Update(Gga(4, 52.0, 9.0, 60.0, 49990.0), 0.2));
  EXPECT_TRUE(tracker.Update(Gga(4, 52.0, 9.0, 60.0, 49990.2), 0.4));
  EXPECT_EQ(tracker.stale_gga(), 1u);
}

TEST_F(TrackerTest, GgaWithoutFixStillCountsForTheTime) {
  EXPECT_FALSE(tracker.Update(Gga(0, 52.0, 9.0, 60.0, 50000.0), 0));
  EXPECT_FALSE(tracker.Update(Gga(4, 52.0, 9.0, 60.0, 50000.0), 0.2));
  EXPECT_TRUE(tracker.Update(Gga(4, 52.0, 9.0, 60.0, 50000.2), 0.4));
}

TEST_F(TrackerTest, FixedOnOldCorrectionsCountsAsFloat) {
  auto fresh = tracker.Update(Gga(4, 52.0, 9.0, 60.0, -1.0, 30.0), 0);
  ASSERT_TRUE(fresh);
  EXPECT_EQ(fresh->solution, Solution::kRtkFixed);
  auto old = tracker.Update(Gga(4, 52.0, 9.0, 60.0, -1.0, 30.5), 0.2);
  ASSERT_TRUE(old);
  EXPECT_EQ(old->solution, Solution::kRtkFloat);
  EXPECT_DOUBLE_EQ(old->accuracy_m, 0.3);
  EXPECT_EQ(tracker.old_corrections(), 1u);
  ASSERT_TRUE(tracker.last_old_correction_age());
  EXPECT_DOUBLE_EQ(*tracker.last_old_correction_age(), 30.5);
  // Fresh corrections again: fixed with the next sentence.
  auto again = tracker.Update(Gga(4, 52.0, 9.0, 60.0, -1.0, 1.0), 0.4);
  ASSERT_TRUE(again);
  EXPECT_EQ(again->solution, Solution::kRtkFixed);
}

TEST_F(TrackerTest, FixedWithoutCorrectionAgeCountsAsFloat) {
  auto p = tracker.Update(Gga(4, 52.0, 9.0, 60.0, -1.0, std::nullopt), 0);
  ASSERT_TRUE(p);
  EXPECT_EQ(p->solution, Solution::kRtkFloat);
  EXPECT_DOUBLE_EQ(p->accuracy_m, 0.3);
  EXPECT_EQ(tracker.old_corrections(), 1u);
  EXPECT_FALSE(tracker.last_old_correction_age());
}

TEST_F(TrackerTest, CorrectionAgeLeavesOtherFixTypesAlone) {
  auto flt = tracker.Update(Gga(5, 52.0, 9.0, 60.0, -1.0, 90.0), 0);
  ASSERT_TRUE(flt);
  EXPECT_EQ(flt->solution, Solution::kRtkFloat);
  auto single = tracker.Update(Gga(1, 52.0, 9.0, 60.0, -1.0, std::nullopt), 0);
  ASSERT_TRUE(single);
  EXPECT_DOUBLE_EQ(single->accuracy_m, 3.0);
  EXPECT_EQ(tracker.old_corrections(), 0u);
}

TEST(Tracker, CorrectionAgeLimitIsAParameter) {
  GnssConfig config;
  config.max_correction_age_s = 10.0;
  GnssTracker tracker{config, LocalFrame(52.0, 9.0, 60.0)};
  auto p = tracker.Update(Gga(4, 52.0, 9.0, 60.0, -1.0, 12.0), 0);
  ASSERT_TRUE(p);
  EXPECT_EQ(p->solution, Solution::kRtkFloat);
}

TEST_F(TrackerTest, MissingCourseGivesNoMotion) {
  NmeaMessage m = Vtg(0.0, 0.0);
  m.vtg.course_deg.reset();
  tracker.Update(m, 0);
  auto p = tracker.Update(Gga(4, 52.0, 9.0), 0.1);
  ASSERT_TRUE(p);
  EXPECT_FALSE(p->motion_valid);
}

TEST_F(TrackerTest, HostAgeOfTheCorrectionsCountsWhenLarger) {
  tracker.SetHostCorrectionsRequired(true);
  tracker.SetHostCorrectionAge(31.0);
  auto p = tracker.Update(Gga(4, 52.0, 9.0, 60.0, -1.0, 1.0), 0);
  ASSERT_TRUE(p);
  EXPECT_EQ(p->solution, Solution::kRtkFloat);
  ASSERT_TRUE(tracker.last_old_correction_age());
  EXPECT_DOUBLE_EQ(*tracker.last_old_correction_age(), 31.0);
  tracker.SetHostCorrectionAge(0.5);
  p = tracker.Update(Gga(4, 52.0, 9.0, 60.0, -1.0, 2.0), 0.2);
  ASSERT_TRUE(p);
  EXPECT_EQ(p->solution, Solution::kRtkFixed);
  // The receiver's age stays the stricter one, and fixed without it counts as float as before.
  p = tracker.Update(Gga(4, 52.0, 9.0, 60.0, -1.0, 40.0), 0.4);
  EXPECT_EQ(p->solution, Solution::kRtkFloat);
  p = tracker.Update(Gga(4, 52.0, 9.0, 60.0, -1.0, std::nullopt), 0.6);
  EXPECT_EQ(p->solution, Solution::kRtkFloat);
}

TEST_F(TrackerTest, WithNtripSetUpFixedWaitsForCorrectionsFromTheHost) {
  tracker.SetHostCorrectionsRequired(true);
  // Fixed with fresh corrections of the receiver's own, e.g. from a LoRa base.
  auto p = tracker.Update(Gga(4, 52.0, 9.0, 60.0, -1.0, 1.0), 0);
  ASSERT_TRUE(p);
  EXPECT_EQ(p->solution, Solution::kRtkFloat);
  EXPECT_DOUBLE_EQ(p->accuracy_m, 0.3);
  EXPECT_EQ(tracker.fixed_before_host(), 1u);
  EXPECT_EQ(tracker.old_corrections(), 0u);
  // The first corrections from NTRIP went to the receiver.
  tracker.SetHostCorrectionAge(0.2);
  p = tracker.Update(Gga(4, 52.0, 9.0, 60.0, -1.0, 1.0), 0.2);
  ASSERT_TRUE(p);
  EXPECT_EQ(p->solution, Solution::kRtkFixed);
  EXPECT_EQ(tracker.fixed_before_host(), 1u);
  // Other fix types stay as they are.
  p = tracker.Update(Gga(1, 52.0, 9.0, 60.0, -1.0, std::nullopt), 0.4);
  ASSERT_TRUE(p);
  EXPECT_DOUBLE_EQ(p->accuracy_m, 3.0);
}

TEST_F(TrackerTest, WithoutNtripFixedNeedsNoCorrectionsFromTheHost) {
  auto p = tracker.Update(Gga(4, 52.0, 9.0, 60.0, -1.0, 1.0), 0);
  ASSERT_TRUE(p);
  EXPECT_EQ(p->solution, Solution::kRtkFixed);
  // Set up and removed again, e.g. ntrip.delete.
  tracker.SetHostCorrectionsRequired(true);
  tracker.SetHostCorrectionsRequired(false);
  p = tracker.Update(Gga(4, 52.0, 9.0, 60.0, -1.0, 1.0), 0.2);
  ASSERT_TRUE(p);
  EXPECT_EQ(p->solution, Solution::kRtkFixed);
  EXPECT_EQ(tracker.fixed_before_host(), 0u);
}

// Corrections the host passed while NTRIP was set up do not age a fix over LoRa once the access is removed.
TEST_F(TrackerTest, AfterNtripDeleteTheOldHostAgeNoLongerCounts) {
  tracker.SetHostCorrectionsRequired(true);
  tracker.SetHostCorrectionAge(0.5);
  auto p = tracker.Update(Gga(4, 52.0, 9.0, 60.0, -1.0, 1.0), 0);
  ASSERT_TRUE(p);
  EXPECT_EQ(p->solution, Solution::kRtkFixed);
  tracker.SetHostCorrectionsRequired(false);
  tracker.SetHostCorrectionAge(120.0);
  p = tracker.Update(Gga(4, 52.0, 9.0, 60.0, -1.0, 1.0), 120.0);
  ASSERT_TRUE(p);
  EXPECT_EQ(p->solution, Solution::kRtkFixed);
  // Set up again: the old age counts until fresh corrections come.
  tracker.SetHostCorrectionsRequired(true);
  p = tracker.Update(Gga(4, 52.0, 9.0, 60.0, -1.0, 1.0), 120.2);
  ASSERT_TRUE(p);
  EXPECT_EQ(p->solution, Solution::kRtkFloat);
}

TEST_F(TrackerTest, BoardReportGivesNoPose) {
  NmeaMessage m;
  m.type = NmeaMessage::Type::kBoardRtk;
  m.board.on = true;
  EXPECT_FALSE(tracker.Update(m, 0));
}

// The motion vector from the RTK fixed positions at 10 Hz, as on the Tron; positions in m east and north of the datum.
class FixMotionTest : public ::testing::Test {
 protected:
  static constexpr double kM = 1.0 / 111320.0;

  std::optional<GnssPose> At(double east, double north, int quality = 4) {
    utc_ += 0.1;
    return tracker.Update(Gga(quality, 52.0 + north * kM, 9.0 + east * kM / std::cos(52.0 * M_PI / 180.0), 60.0, utc_),
                          utc_);
  }

  // Drives from (east, north) in direction heading (rad from east) at speed for seconds; the last pose.
  std::optional<GnssPose> Drive(double& east, double& north, double heading, double speed, double seconds,
                                int* valid = nullptr) {
    std::optional<GnssPose> pose;
    for (int i = 0; i < std::lround(seconds * 10.0); i++) {
      east += std::cos(heading) * speed * 0.1;
      north += std::sin(heading) * speed * 0.1;
      // The board of the Tron reports this course whatever the way.
      tracker.Update(Rmc(speed, 285.91), utc_);
      pose = At(east, north);
      if (valid && pose && pose->motion_valid) (*valid)++;
    }
    return pose;
  }

  // One epoch that gives no pose within a straight drive, then 0.5 s more of the drive.
  void DriveAcrossEpochWithoutPose(int quality, bool with_position) {
    double e = 0.0;
    double n = 0.0;
    ASSERT_TRUE(Drive(e, n, 0.0, 0.25, 2.0)->motion_valid);
    e += 0.025;
    utc_ += 0.1;
    NmeaMessage gap = Gga(quality, 52.0, 9.0 + e * kM / std::cos(52.0 * M_PI / 180.0), 60.0, utc_);
    if (!with_position) {
      gap.gga.lat_deg.reset();
      gap.gga.lon_deg.reset();
    }
    ASSERT_FALSE(tracker.Update(gap, utc_));
    int valid = 0;
    // 0.075 m of fixed positions after it: not yet.
    Drive(e, n, 0.0, 0.25, 0.3, &valid);
    EXPECT_EQ(valid, 0);
    EXPECT_TRUE(Drive(e, n, 0.0, 0.25, 0.2)->motion_valid);
  }

  GnssTracker tracker{GnssConfig{}, LocalFrame(52.0, 9.0, 60.0)};
  double utc_ = 43200.0;
};

TEST_F(FixMotionTest, TheWayGivesTheMotionNotTheCourseOfTheBoard) {
  for (const double heading : {0.0, 0.9, M_PI / 2, 2.8, -2.0}) {
    double e = 0.0;
    double n = 0.0;
    const auto pose = Drive(e, n, heading, 0.25, 3.0);
    ASSERT_TRUE(pose && pose->motion_valid) << heading;
    EXPECT_NEAR(std::hypot(pose->motion_east_mps, pose->motion_north_mps), 0.25, 0.005) << heading;
    EXPECT_NEAR(std::remainder(pose->motion_heading - heading, 2.0 * M_PI), 0.0, 0.01) << heading;
  }
}

TEST_F(FixMotionTest, NothingBeforeTheWayAndNothingStanding) {
  double e = 0.0;
  double n = 0.0;
  int valid = 0;
  // 0.09 m in 0.9 s: not yet.
  Drive(e, n, 0.0, 0.1, 0.9, &valid);
  EXPECT_EQ(valid, 0);
  const auto pose = Drive(e, n, 0.0, 0.1, 0.2, &valid);
  EXPECT_TRUE(pose && pose->motion_valid);
  // Standing with 5 mm of noise, from 0.3 s after the stop on.
  valid = 0;
  for (int i = 0; i < 50; i++) {
    const auto p = At(e + 0.005 * std::sin(i * 1.7), n + 0.005 * std::cos(i * 2.3));
    if (i >= 3 && p && p->motion_valid) valid++;
  }
  EXPECT_EQ(valid, 0);
}

TEST_F(FixMotionTest, StoppingEndsTheMotionAtOnce) {
  double e = 0.0;
  double n = 0.0;
  ASSERT_TRUE(Drive(e, n, 0.0, 0.25, 2.0)->motion_valid);
  // Standing still: the window still reaches back into the drive, but its second half has no way.
  int valid = 0;
  for (int i = 0; i < 20; i++) {
    const auto p = At(e, n);
    if (i >= 3 && p && p->motion_valid) valid++;
  }
  EXPECT_EQ(valid, 0);
}

TEST_F(FixMotionTest, TooSlowForTheWayInTheWindowGivesNone) {
  double e = 0.0;
  double n = 0.0;
  int valid = 0;
  // 0.06 m/s: 0.09 m within the 1.5 s.
  Drive(e, n, 0.0, 0.06, 5.0, &valid);
  EXPECT_EQ(valid, 0);
}

TEST_F(FixMotionTest, ATurnGivesNone) {
  // 0.25 m/s on a circle of 0.3 m: the halves of a 0.1 m way bend by about 10 degrees.
  int valid = 0;
  for (int i = 1; i <= 60; i++) {
    const double angle = 0.25 * 0.1 * i / 0.3;
    const auto pose = At(0.3 * std::sin(angle), 0.3 - 0.3 * std::cos(angle));
    if (pose && pose->motion_valid) valid++;
  }
  EXPECT_EQ(valid, 0);
}

TEST_F(FixMotionTest, AFixThatIsNotRtkFixedStartsAgain) {
  double e = 0.0;
  double n = 0.0;
  ASSERT_TRUE(Drive(e, n, 0.0, 0.25, 2.0)->motion_valid);
  e += 0.025;
  const auto floating = At(e, n, 5);
  ASSERT_TRUE(floating);
  EXPECT_FALSE(floating->motion_valid);
  int valid = 0;
  // 0.075 m of fixed positions after it: not yet.
  Drive(e, n, 0.0, 0.25, 0.3, &valid);
  EXPECT_EQ(valid, 0);
  EXPECT_TRUE(Drive(e, n, 0.0, 0.25, 0.2)->motion_valid);
}

TEST_F(FixMotionTest, AnEpochWithoutFixStartsAgain) {
  DriveAcrossEpochWithoutPose(0, true);
}

TEST_F(FixMotionTest, AnEpochOfUnknownQualityStartsAgain) {
  DriveAcrossEpochWithoutPose(9, true);
}

TEST_F(FixMotionTest, AnEpochWithoutPositionStartsAgain) {
  DriveAcrossEpochWithoutPose(4, false);
}

TEST_F(FixMotionTest, ReceiverCourseWhenSwitchedBack) {
  GnssTracker receiver(ReceiverMotion(), LocalFrame(52.0, 9.0, 60.0));
  receiver.Update(Rmc(0.25, 285.91), 0.0);
  const auto pose = receiver.Update(Gga(4, 52.0, 9.0), 0.1);
  ASSERT_TRUE(pose && pose->motion_valid);
  EXPECT_NEAR(pose->motion_heading, (90.0 - 285.91) * M_PI / 180.0 + 2.0 * M_PI, 1e-9);
}

}  // namespace
}  // namespace tron
