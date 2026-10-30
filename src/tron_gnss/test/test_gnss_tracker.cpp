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

class TrackerTest : public ::testing::Test {
 protected:
  GnssTracker tracker{GnssConfig{}, LocalFrame(52.0, 9.0, 60.0)};
};

TEST_F(TrackerTest, FixTypesGiveFlagsAndAccuracy) {
  auto fixed = tracker.Update(Gga(4, 52.0, 9.0), 0);
  ASSERT_TRUE(fixed);
  EXPECT_EQ(fixed->flags, kFlagRtk | kFlagRtkFixed);
  EXPECT_DOUBLE_EQ(fixed->accuracy_m, 0.02);

  auto flt = tracker.Update(Gga(5, 52.0, 9.0), 0);
  ASSERT_TRUE(flt);
  EXPECT_EQ(flt->flags, kFlagRtk | kFlagRtkFloat);
  EXPECT_DOUBLE_EQ(flt->accuracy_m, 0.3);

  auto dgps = tracker.Update(Gga(2, 52.0, 9.0), 0);
  ASSERT_TRUE(dgps);
  EXPECT_EQ(dgps->flags, 0);
  EXPECT_DOUBLE_EQ(dgps->accuracy_m, 1.0);

  auto single = tracker.Update(Gga(1, 52.0, 9.0), 0);
  ASSERT_TRUE(single);
  EXPECT_EQ(single->flags, 0);
  EXPECT_DOUBLE_EQ(single->accuracy_m, 3.0);

  auto dr = tracker.Update(Gga(6, 52.0, 9.0), 0);
  ASSERT_TRUE(dr);
  EXPECT_EQ(dr->flags, kFlagDeadReckoning);
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
  EXPECT_EQ(fresh->flags, kFlagRtk | kFlagRtkFixed);
  auto old = tracker.Update(Gga(4, 52.0, 9.0, 60.0, -1.0, 30.5), 0.2);
  ASSERT_TRUE(old);
  EXPECT_EQ(old->flags, kFlagRtk | kFlagRtkFloat);
  EXPECT_DOUBLE_EQ(old->accuracy_m, 0.3);
  EXPECT_EQ(tracker.old_corrections(), 1u);
  ASSERT_TRUE(tracker.last_old_correction_age());
  EXPECT_DOUBLE_EQ(*tracker.last_old_correction_age(), 30.5);
  // Fresh corrections again: fixed with the next sentence.
  auto again = tracker.Update(Gga(4, 52.0, 9.0, 60.0, -1.0, 1.0), 0.4);
  ASSERT_TRUE(again);
  EXPECT_EQ(again->flags, kFlagRtk | kFlagRtkFixed);
}

TEST_F(TrackerTest, FixedWithoutCorrectionAgeCountsAsFloat) {
  auto p = tracker.Update(Gga(4, 52.0, 9.0, 60.0, -1.0, std::nullopt), 0);
  ASSERT_TRUE(p);
  EXPECT_EQ(p->flags, kFlagRtk | kFlagRtkFloat);
  EXPECT_DOUBLE_EQ(p->accuracy_m, 0.3);
  EXPECT_EQ(tracker.old_corrections(), 1u);
  EXPECT_FALSE(tracker.last_old_correction_age());
}

TEST_F(TrackerTest, CorrectionAgeLeavesOtherFixTypesAlone) {
  auto flt = tracker.Update(Gga(5, 52.0, 9.0, 60.0, -1.0, 90.0), 0);
  ASSERT_TRUE(flt);
  EXPECT_EQ(flt->flags, kFlagRtk | kFlagRtkFloat);
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
  EXPECT_EQ(p->flags, kFlagRtk | kFlagRtkFloat);
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
  tracker.SetHostCorrectionAge(31.0);
  auto p = tracker.Update(Gga(4, 52.0, 9.0, 60.0, -1.0, 1.0), 0);
  ASSERT_TRUE(p);
  EXPECT_EQ(p->flags, kFlagRtk | kFlagRtkFloat);
  ASSERT_TRUE(tracker.last_old_correction_age());
  EXPECT_DOUBLE_EQ(*tracker.last_old_correction_age(), 31.0);
  tracker.SetHostCorrectionAge(0.5);
  p = tracker.Update(Gga(4, 52.0, 9.0, 60.0, -1.0, 2.0), 0.2);
  ASSERT_TRUE(p);
  EXPECT_EQ(p->flags, kFlagRtk | kFlagRtkFixed);
  // The receiver's age stays the stricter one, and fixed without it counts as float as before.
  p = tracker.Update(Gga(4, 52.0, 9.0, 60.0, -1.0, 40.0), 0.4);
  EXPECT_EQ(p->flags, kFlagRtk | kFlagRtkFloat);
  p = tracker.Update(Gga(4, 52.0, 9.0, 60.0, -1.0, std::nullopt), 0.6);
  EXPECT_EQ(p->flags, kFlagRtk | kFlagRtkFloat);
}

TEST_F(TrackerTest, BoardReportGivesNoPose) {
  NmeaMessage m;
  m.type = NmeaMessage::Type::kBoardRtk;
  m.board.on = true;
  EXPECT_FALSE(tracker.Update(m, 0));
}

}  // namespace
}  // namespace tron
