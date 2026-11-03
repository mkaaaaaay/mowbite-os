#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "xcore_driver/gnss.h"
#include "xcore_driver/services.h"

using namespace xcore;
constexpr auto kRtkFixed = mowbite_gnss::Solution::kRtkFixed;
constexpr auto kRtkFloat = mowbite_gnss::Solution::kRtkFloat;

namespace {

Output Gps(uint16_t channel, const Bytes& value, uint64_t time_us = 5000000) {
  return Output{kGpsService, channel, value, time_us};
}

Bytes Doubles(std::initializer_list<double> values) {
  Writer w;
  for (double v : values) w.F64(v);
  return w.Take();
}

Bytes Text(const std::string& text) {
  return Bytes(text.begin(), text.end());
}

std::vector<Output> Solution(const char* fix, double accuracy = 0.014, uint64_t time_us = 5000000) {
  std::vector<Output> out = {Gps(gps::kOutPosition, Doubles({52.0001, 9.0002, 61.5}), time_us),
                             Gps(gps::kOutHorizontalAccuracy, Doubles({accuracy}), time_us),
                             Gps(gps::kOutVerticalAccuracy, Doubles({0.03}), time_us),
                             Gps(gps::kOutMotionEnu, Doubles({0.3, 0.0, 0.01}), time_us)};
  if (fix) out.push_back(Gps(gps::kOutFixType, Text(fix), time_us));
  return out;
}

GnssConverter Converter(GnssConfig config = {}) {
  return GnssConverter(config, mowbite_gnss::LocalFrame(52.0, 9.0, 60.0));
}

}  // namespace

TEST(Gnss, DecodesOneTransaction) {
  auto s = DecodeGnss(Solution("FIX"));
  ASSERT_TRUE(s);
  EXPECT_DOUBLE_EQ(s->lat_deg, 52.0001);
  EXPECT_DOUBLE_EQ(s->lon_deg, 9.0002);
  EXPECT_DOUBLE_EQ(s->height_m, 61.5);
  ASSERT_TRUE(s->horizontal_accuracy_m);
  EXPECT_DOUBLE_EQ(*s->horizontal_accuracy_m, 0.014);
  EXPECT_EQ(s->fix, BoardFix::kFixed);
  ASSERT_TRUE(s->motion_enu);
  EXPECT_DOUBLE_EQ((*s->motion_enu)[0], 0.3);
  EXPECT_EQ(s->board_time_us, 5000000u);
  EXPECT_EQ(DecodeGnss(Solution("FLOAT"))->fix, BoardFix::kFloat);
  // Without RTK the board leaves the fix type out.
  EXPECT_EQ(DecodeGnss(Solution(nullptr))->fix, BoardFix::kNone);
  EXPECT_EQ(DecodeGnss(Solution("SOMETHING"))->fix, BoardFix::kNone);
}

TEST(Gnss, NoPositionNoSolution) {
  EXPECT_FALSE(DecodeGnss({}));
  EXPECT_FALSE(DecodeGnss({Gps(gps::kOutFixType, Text("FIX"))}));
  // Outputs of other services do not count.
  EXPECT_FALSE(DecodeGnss({Output{kImuService, gps::kOutPosition, Doubles({52.0, 9.0, 60.0}), 1}}));
  // A position of the wrong length.
  EXPECT_FALSE(DecodeGnss({Gps(gps::kOutPosition, Doubles({52.0, 9.0}))}));
}

TEST(Gnss, ImpossiblePositionsAreDropped) {
  EXPECT_FALSE(DecodeGnss({Gps(gps::kOutPosition, Doubles({0.0, 0.0, 0.0}))}));
  EXPECT_FALSE(DecodeGnss({Gps(gps::kOutPosition, Doubles({NAN, 9.0, 60.0}))}));
  EXPECT_FALSE(DecodeGnss({Gps(gps::kOutPosition, Doubles({52.0, INFINITY, 60.0}))}));
  EXPECT_FALSE(DecodeGnss({Gps(gps::kOutPosition, Doubles({91.0, 9.0, 60.0}))}));
  EXPECT_FALSE(DecodeGnss({Gps(gps::kOutPosition, Doubles({52.0, 181.0, 60.0}))}));
  EXPECT_FALSE(DecodeGnss({Gps(gps::kOutPosition, Doubles({52.0, 9.0, NAN}))}));
}

TEST(Gnss, UnusableAccuracyAndMotionAreLeftOut) {
  for (double bad : {0.0, -1.0, std::nan(""), 5000.0}) {
    auto s = DecodeGnss(Solution("FIX", bad));
    ASSERT_TRUE(s);
    EXPECT_FALSE(s->horizontal_accuracy_m) << bad;
  }
  auto out = Solution("FIX");
  out[3] = Gps(gps::kOutMotionEnu, Doubles({NAN, 0.0, 0.0}));
  EXPECT_FALSE(DecodeGnss(out)->motion_enu);
}

TEST(Gnss, FixedNeedsTheHostsFreshCorrections) {
  auto c = Converter();
  auto pose = c.Convert(*DecodeGnss(Solution("FIX", 0.014, 1000)), 1.0);
  ASSERT_TRUE(pose);
  EXPECT_EQ(pose->solution, kRtkFixed);
  EXPECT_DOUBLE_EQ(pose->accuracy_m, 0.014);
  EXPECT_EQ(c.fixed_without_corrections(), 0u);

  // Never passed any: float.
  pose = c.Convert(*DecodeGnss(Solution("FIX", 0.014, 2000)), std::nullopt);
  EXPECT_EQ(pose->solution, kRtkFloat);
  EXPECT_DOUBLE_EQ(pose->accuracy_m, 0.3);
  EXPECT_EQ(c.fixed_without_corrections(), 1u);
  EXPECT_FALSE(c.last_downgrade_age());

  // Older than the limit: float; at the limit still fixed.
  pose = c.Convert(*DecodeGnss(Solution("FIX", 0.014, 3000)), 30.5);
  EXPECT_EQ(pose->solution, kRtkFloat);
  EXPECT_EQ(c.fixed_without_corrections(), 2u);
  EXPECT_DOUBLE_EQ(*c.last_downgrade_age(), 30.5);
  pose = c.Convert(*DecodeGnss(Solution("FIX", 0.014, 4000)), 30.0);
  EXPECT_EQ(pose->solution, kRtkFixed);
}

TEST(Gnss, DowngradedFixKeepsAWorseBoardAccuracy) {
  auto c = Converter();
  auto pose = c.Convert(*DecodeGnss(Solution("FIX", 0.5)), std::nullopt);
  EXPECT_DOUBLE_EQ(pose->accuracy_m, 0.5);
}

TEST(Gnss, FloatAndSingle) {
  auto c = Converter();
  auto pose = c.Convert(*DecodeGnss(Solution("FLOAT", 0.12, 1000)), 1.0);
  EXPECT_EQ(pose->solution, kRtkFloat);
  EXPECT_DOUBLE_EQ(pose->accuracy_m, 0.12);
  pose = c.Convert(*DecodeGnss(Solution(nullptr, 1.8, 2000)), 1.0);
  EXPECT_EQ(pose->solution, mowbite_gnss::Solution::kSingle);
  EXPECT_DOUBLE_EQ(pose->accuracy_m, 1.8);
  EXPECT_EQ(c.fixed_without_corrections(), 0u);
}

TEST(Gnss, TableWithoutBoardAccuracy) {
  GnssConfig config;
  auto c = Converter(config);
  EXPECT_DOUBLE_EQ(c.Convert(*DecodeGnss(Solution("FIX", NAN, 1000)), 1.0)->accuracy_m, config.accuracy_rtk_fixed_m);
  EXPECT_DOUBLE_EQ(c.Convert(*DecodeGnss(Solution("FLOAT", NAN, 2000)), 1.0)->accuracy_m, config.accuracy_rtk_float_m);
  EXPECT_DOUBLE_EQ(c.Convert(*DecodeGnss(Solution(nullptr, NAN, 3000)), 1.0)->accuracy_m, config.accuracy_single_m);
}

TEST(Gnss, BoardAccuracyHasAFloor) {
  auto c = Converter();
  EXPECT_DOUBLE_EQ(c.Convert(*DecodeGnss(Solution("FIX", 0.001)), 1.0)->accuracy_m, 0.01);
}

TEST(Gnss, PositionInTheLocalFrame) {
  auto c = Converter();
  auto pose = c.Convert(*DecodeGnss(Solution("FIX")), 1.0);
  const mowbite_gnss::Enu expected = mowbite_gnss::LocalFrame(52.0, 9.0, 60.0).ToEnu(52.0001, 9.0002, 61.5);
  EXPECT_DOUBLE_EQ(pose->position.east, expected.east);
  EXPECT_DOUBLE_EQ(pose->position.north, expected.north);
  EXPECT_NEAR(pose->position.up, 1.5, 0.01);
  EXPECT_NEAR(pose->position.north, 11.1, 0.1);
  EXPECT_NEAR(pose->position.east, 13.7, 0.1);
  EXPECT_EQ(pose->sensor_stamp_ms, 5000u);
}

TEST(Gnss, MotionGivesTheCourse) {
  auto c = Converter();
  auto out = Solution("FIX");
  out[3] = Gps(gps::kOutMotionEnu, Doubles({0.0, -0.2, 0.0}));
  auto pose = c.Convert(*DecodeGnss(out), 1.0);
  EXPECT_TRUE(pose->motion_valid);
  EXPECT_DOUBLE_EQ(pose->motion_north_mps, -0.2);
  EXPECT_NEAR(pose->motion_heading, -M_PI / 2, 1e-12);
  out.erase(out.begin() + 3);
  out.front().board_time_us = 6000000;
  EXPECT_FALSE(c.Convert(*DecodeGnss(out), 1.0)->motion_valid);
}

TEST(Gnss, SameBoardTimeIsARepeat) {
  auto c = Converter();
  EXPECT_TRUE(c.Convert(*DecodeGnss(Solution("FIX", 0.014, 1000)), 1.0));
  EXPECT_FALSE(c.Convert(*DecodeGnss(Solution("FIX", 0.014, 1000)), 1.0));
  EXPECT_EQ(c.repeated(), 1u);
  // A restarted board starts its clock again.
  EXPECT_TRUE(c.Convert(*DecodeGnss(Solution("FIX", 0.014, 500)), 1.0));
}

TEST(Gnss, GgaOfTheSolution) {
  auto s = *DecodeGnss(Solution("FIX"));
  auto f = GgaOf(s, 3600.0);
  EXPECT_EQ(f.quality, mowbite_gnss::GgaQuality::kRtkFixed);
  EXPECT_DOUBLE_EQ(f.lat_deg, 52.0001);
  EXPECT_DOUBLE_EQ(f.altitude_m, 61.5);
  EXPECT_GT(f.satellites, 4);
  s.fix = BoardFix::kFloat;
  EXPECT_EQ(GgaOf(s, 0).quality, mowbite_gnss::GgaQuality::kRtkFloat);
  s.fix = BoardFix::kNone;
  EXPECT_EQ(GgaOf(s, 0).quality, mowbite_gnss::GgaQuality::kSingle);
  EXPECT_EQ(mowbite_gnss::BuildGga(GgaOf(s, 0)).compare(0, 7, "$GPGGA,"), 0);
}

TEST(RtcmQueue, PiecesFitOneDatagram) {
  RtcmQueue q;
  std::vector<uint8_t> data(5000);
  for (size_t i = 0; i < data.size(); i++) data[i] = static_cast<uint8_t>(i * 7);
  ASSERT_TRUE(q.Push(data.data(), 3000));
  ASSERT_TRUE(q.Push(data.data() + 3000, 2000));
  auto pieces = q.Take();
  ASSERT_EQ(pieces.size(), 4u);
  Bytes joined;
  for (const auto& p : pieces) {
    EXPECT_LE(p.size(), kMaxPayload);
    joined.insert(joined.end(), p.begin(), p.end());
  }
  EXPECT_EQ(joined, Bytes(data.begin(), data.end()));
  EXPECT_EQ(kMaxPayload, 1448u);
  EXPECT_EQ(q.size(), 0u);
  EXPECT_TRUE(q.Take().empty());
}

TEST(RtcmQueue, FullQueueDropsTheNewest) {
  RtcmQueue q(1000, 400);
  std::vector<uint8_t> a(900, 1), b(200, 2);
  EXPECT_TRUE(q.Push(a.data(), a.size()));
  EXPECT_FALSE(q.Push(b.data(), b.size()));
  EXPECT_EQ(q.dropped_bytes(), 200u);
  auto pieces = q.Take();
  ASSERT_EQ(pieces.size(), 3u);
  EXPECT_EQ(pieces[2].size(), 100u);
  EXPECT_TRUE(q.Push(b.data(), b.size()));
  q.Clear();
  EXPECT_EQ(q.dropped_bytes(), 400u);
  EXPECT_EQ(q.size(), 0u);
}
