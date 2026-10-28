#include <gtest/gtest.h>

#include <cmath>

#include "tron_gnss/gnss_tracker.h"

namespace tron {
namespace {

NmeaMessage Gga(int quality, double lat, double lon, double alt = 60.0, double utc = 43200.5) {
  NmeaMessage m;
  m.type = NmeaMessage::Type::kGga;
  m.gga.quality = quality;
  m.gga.lat_deg = lat;
  m.gga.lon_deg = lon;
  m.gga.altitude_m = alt;
  m.gga.utc_s = utc;
  m.gga.satellites = 18;
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
  auto p = tracker.Update(Gga(4, 52.0003, 9.0004, 61.0), 0);
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

TEST_F(TrackerTest, MissingCourseGivesNoMotion) {
  NmeaMessage m = Vtg(0.0, 0.0);
  m.vtg.course_deg.reset();
  tracker.Update(m, 0);
  auto p = tracker.Update(Gga(4, 52.0, 9.0), 0.1);
  ASSERT_TRUE(p);
  EXPECT_FALSE(p->motion_valid);
}

}  // namespace
}  // namespace tron
