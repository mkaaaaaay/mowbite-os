#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "mowbite_stats/sensor_history.h"

using namespace mowbite_stats;

namespace {

// A whole hour, 2026-11-15 00:00 UTC.
constexpr double kNow = 1794700800;
constexpr int64_t kT = static_cast<int64_t>(kNow);

SensorHistory WithVoltage() {
  SensorHistory h;
  EXPECT_TRUE(h.Describe("v", "Battery", "V", true));
  return h;
}

json Points(const SensorHistory& h, const HistoryQuery& q, double now = kNow) {
  const json answer = h.Query(q, now);
  return answer["sensors"].empty() ? json::array() : answer["sensors"][0]["points"];
}

const HourFile* Find(const std::vector<HourFile>& files, int64_t hour) {
  for (const auto& file : files) {
    if (file.hour == hour) return &file;
  }
  return nullptr;
}

}  // namespace

TEST(SensorHistory, MinuteBucketsWithMinMeanMaxAndCount) {
  SensorHistory h = WithVoltage();
  h.Add("v", 25.0, kNow + 1);
  h.Add("v", 26.0, kNow + 30);
  h.Add("v", 27.5, kNow + 59.9);
  h.Add("v", 24.0, kNow + 60);
  EXPECT_EQ(Points(h, {}, kNow + 60), json::parse("[[1794700800, 25, 26.1667, 27.5], [1794700860, 24, 24, 24]]"));
  const auto files = h.TakeFiles(kNow + 120);
  ASSERT_EQ(files.size(), 1u);
  EXPECT_EQ(files[0].hour, kT);
  EXPECT_EQ(files[0].text, "{\"schema\":1,\"sensors\":[[\"v\",\"Battery\",\"V\"]]}\n"
                           "[1794700800,0,25.0,26.1667,27.5,3]\n[1794700860,0,24.0,24.0,24.0,1]\n");
}

TEST(SensorHistory, OnlyDescribedNumbersWithFiniteValuesAndASetClock) {
  SensorHistory h = WithVoltage();
  EXPECT_FALSE(h.Describe("state", "State", "", false));
  h.Add("state", 1.0, kNow);
  h.Add("unknown", 1.0, kNow);
  h.Add("v", std::nan(""), kNow);
  h.Add("v", std::numeric_limits<double>::infinity(), kNow);
  h.Add("v", 25.0, kEarliestWall - 1);
  h.Add("v", 25.0, std::nan(""));
  EXPECT_EQ(h.Buckets(), 0u);
  EXPECT_EQ(h.Query({}, kNow)["sensors"].size(), 1u);
  // Turned into a text: no more values, the kept ones stay.
  h.Add("v", 25.0, kNow);
  EXPECT_FALSE(h.Describe("v", "Battery", "", false));
  h.Add("v", 26.0, kNow + 60);
  EXPECT_EQ(Points(h, {}), json::parse("[[1794700800, 25, 25, 25]]"));
}

TEST(SensorHistory, SensorLimitsAndTexts) {
  SensorHistory h;
  for (size_t i = 0; i < kMaxSensors; ++i) EXPECT_TRUE(h.Describe("s" + std::to_string(i), "", "", true));
  EXPECT_FALSE(h.Describe("one_more", "", "", true));
  EXPECT_TRUE(h.Describe("s0", "New name", "A", true));
  SensorHistory t;
  EXPECT_FALSE(t.Describe("", "x", "", true));
  EXPECT_FALSE(t.Describe("long", std::string(kMaxSensorText + 1, 'x'), "", true));
  EXPECT_TRUE(t.Describe("ok", std::string(kMaxSensorText, 'x'), "", true));
  EXPECT_FALSE(t.Describe("utf", "bad \xff", "", true));
}

TEST(SensorHistory, StepsMergeWeightedByCount) {
  SensorHistory h = WithVoltage();
  for (int i = 0; i < 3; ++i) h.Add("v", 10.0, kNow + i);  // three values of 10 in the first minute
  h.Add("v", 40.0, kNow + 120);                            // one of 40 in the third
  h.Add("v", 5.0, kNow + 300);                             // the next 5 min
  HistoryQuery q;
  q.step = 300;
  json answer = h.Query(q, kNow + 400);
  EXPECT_EQ(answer["step"], 300);
  EXPECT_EQ(answer["sensors"][0]["points"], json::parse("[[1794700800, 10, 17.5, 40], [1794701100, 5, 5, 5]]"));
  // A step between is raised to the next one.
  q.step = 120;
  EXPECT_EQ(h.Query(q, kNow + 400)["step"], 300);
  q.step = 900;
  EXPECT_EQ(h.Query(q, kNow + 400)["sensors"][0]["points"], json::parse("[[1794700800, 5, 15, 40]]"));
}

TEST(SensorHistory, FromIsInclusiveAndTheRunningBucketIsIn) {
  SensorHistory h = WithVoltage();
  for (int m = 0; m < 5; ++m) h.Add("v", m, kNow + m * 60);
  h.Add("v", 9.0, kNow + 250);  // the minute going on
  HistoryQuery q;
  q.from = kT + 180;
  EXPECT_EQ(Points(h, q, kNow + 250), json::parse("[[1794700980, 3, 3, 3], [1794701040, 4, 6.5, 9]]"));
  json answer = h.Query(q, kNow + 250);
  EXPECT_EQ(answer["from"], kT + 180);
  EXPECT_EQ(answer["to"], kT + 250);
  EXPECT_FALSE(answer["truncated"].get<bool>());
  // A from within a minute takes that minute; to cuts after the buckets that begin up to it.
  q.from = kT + 200;
  q.to = kT + 180;
  EXPECT_EQ(Points(h, q, kNow + 250), json::parse("[[1794700980, 3, 3, 3]]"));
  // Without from the last 48 h.
  EXPECT_EQ(h.Query({}, kNow + 250)["from"], kT + 240 - kKeepSeconds);
}

TEST(SensorHistory, SensorIdsSelect) {
  SensorHistory h = WithVoltage();
  ASSERT_TRUE(h.Describe("a", "Amps", "A", true));
  h.Add("v", 1, kNow);
  h.Add("a", 2, kNow);
  HistoryQuery q;
  q.ids = std::vector<std::string>{"a", "nope"};
  const json answer = h.Query(q, kNow);
  ASSERT_EQ(answer["sensors"].size(), 1u);
  EXPECT_EQ(answer["sensors"][0]["sensor_id"], "a");
  EXPECT_EQ(answer["sensors"][0]["name"], "Amps");
  EXPECT_EQ(answer["sensors"][0]["unit"], "A");
  q.ids = std::vector<std::string>{};
  EXPECT_TRUE(h.Query(q, kNow)["sensors"].empty());
}

// The 48 h rule: a minute that ended kKeepSeconds before now goes, the one after it stays, and an hour file goes
// once its hour is wholly over.
TEST(SensorHistory, BucketsAndFilesGoAfter48Hours) {
  SensorHistory h = WithVoltage();
  h.Add("v", 1, kNow);
  h.Add("v", 2, kNow + 60);
  h.TakeFiles(kNow + 120);
  const double later = kNow + kKeepSeconds + 60;
  HistoryQuery all;
  all.from = 0;
  auto files = h.TakeFiles(later);
  EXPECT_EQ(Points(h, all, later), json::parse("[[1794700860, 2, 2, 2]]"));
  EXPECT_TRUE(files.empty());
  files = h.TakeFiles(kNow + kKeepSeconds + 3600);
  EXPECT_EQ(h.Buckets(), 0u);
  ASSERT_EQ(files.size(), 1u);
  EXPECT_EQ(files[0].hour, kT);
  EXPECT_EQ(files[0].text, "");
  EXPECT_EQ(h.Bytes(), 0u);
}

// The clock going back destroys nothing: values land in their minutes, nothing is dropped as old.
TEST(SensorHistory, ClockGoingBackKeepsEverything) {
  SensorHistory h = WithVoltage();
  h.Add("v", 1, kNow + 7200);
  h.Add("v", 2, kNow);         // two hours back
  h.Add("v", 3, kNow + 7200);  // and forth again
  auto files = h.TakeFiles(kNow);
  EXPECT_NE(Find(files, kT), nullptr);
  EXPECT_NE(Find(files, kT + 7200), nullptr);
  HistoryQuery q;
  q.to = kT + 7200;
  EXPECT_EQ(Points(h, q), json::parse("[[1794700800, 2, 2, 2], [1794708000, 1, 2, 3]]"));
  // A clock before kEarliestWall (not set yet) prunes nothing either.
  h.TakeFiles(1000);
  EXPECT_EQ(h.Buckets(), 2u);
}

TEST(SensorHistory, FilesLoadBack) {
  SensorHistory h = WithVoltage();
  ASSERT_TRUE(h.Describe("t", "Temp \xc2\xb0", "deg.C", true));
  for (int m = 0; m < 90; ++m) {
    h.Add("v", 25 + m * 0.001, kNow + m * 60);
    h.Add("t", -3.25, kNow + m * 60 + 1);
  }
  const auto files = h.TakeFiles(kNow + 5400);
  ASSERT_EQ(files.size(), 2u);
  SensorHistory back;
  for (auto it = files.rbegin(); it != files.rend(); ++it) back.Load(it->hour, it->text);
  EXPECT_EQ(back.Query({}, kNow + 5400), h.Query({}, kNow + 5400));
  EXPECT_EQ(back.Bytes(), h.Bytes());
  // Loaded clean: nothing to write; described again, it records on.
  EXPECT_TRUE(back.TakeFiles(kNow + 5400).empty());
  ASSERT_TRUE(back.Describe("v", "Battery", "V", true));
  back.Add("v", 30, kNow + 5400);
  EXPECT_EQ(back.TakeFiles(kNow + 5460).size(), 1u);
}

TEST(SensorHistory, FileWithABrokenEndLoadsTheRest) {
  SensorHistory h = WithVoltage();
  for (int m = 0; m < 3; ++m) h.Add("v", m, kNow + m * 60);
  const std::string text = h.TakeFiles(kNow + 180)[0].text;
  // Cut in the middle of the last line, as a write that stopped would leave it.
  const std::string broken = text.substr(0, text.size() - 8);
  SensorHistory back;
  back.Load(kT, broken);
  EXPECT_EQ(Points(back, {}, kNow + 180), json::parse("[[1794700800, 0, 0, 0], [1794700860, 1, 1, 1]]"));
  // Written anew without the broken line.
  const auto files = back.TakeFiles(kNow + 180);
  ASSERT_EQ(files.size(), 1u);
  EXPECT_EQ(files[0].text, text.substr(0, text.rfind('[')));
  // Lines that are no buckets of this hour are left out as well.
  SensorHistory odd;
  odd.Load(kT, text + "[1794700800,5,1,1,1,1]\n[1794697200,0,1,1,1,1]\n[1794700801,0,1,1,1,1]\n"
                      "[1794700920,0,2,1,1,1]\nnot json\n[1794700920,0,1,1,1,0]\n");
  EXPECT_EQ(odd.Buckets(), 3u);
}

TEST(SensorHistory, FilesThatAreNoHistoryAreRemoved) {
  SensorHistory h;
  h.Load(kT, "");
  h.Load(kT + 3600, "garbage\n[1794704400,0,1,1,1,1]\n");
  h.Load(kT + 7200, "{\"schema\":2,\"sensors\":[[\"v\",\"\",\"\"]]}\n[1794708000,0,1,1,1,1]\n");
  h.Load(kT + 1, "{\"schema\":1,\"sensors\":[[\"v\",\"\",\"\"]]}\n[1794700800,0,1,1,1,1]\n");
  EXPECT_EQ(h.Buckets(), 0u);
  const auto files = h.TakeFiles(kNow);
  ASSERT_EQ(files.size(), 3u);
  for (const auto& file : files) EXPECT_EQ(file.text, "");
}

// The MB rule: past the limit the oldest hours go, in memory and on the disk.
TEST(SensorHistory, OldestHoursGoPastTheLimit) {
  SensorHistory probe = WithVoltage();
  for (int m = 0; m < 60; ++m) probe.Add("v", 25.5, kNow + m * 60);
  const size_t hour_bytes = probe.TakeFiles(kNow)[0].text.size();
  SensorHistory h(hour_bytes * 3);
  ASSERT_TRUE(h.Describe("v", "Battery", "V", true));
  for (int m = 0; m < 4 * 60; ++m) h.Add("v", 25.5, kNow + m * 60);
  const auto files = h.TakeFiles(kNow + 4 * 3600);
  EXPECT_EQ(h.Bytes(), hour_bytes * 3);
  ASSERT_NE(Find(files, kT), nullptr);
  EXPECT_EQ(Find(files, kT)->text, "");
  EXPECT_NE(Find(files, kT + 3600), nullptr);
  HistoryQuery all;
  all.from = 0;
  EXPECT_EQ(Points(h, all, kNow + 4 * 3600)[0][0], kT + 3600);
  // Loading stops at the limit as well: the newest files first, older ones are removed.
  SensorHistory back(hour_bytes * 2);
  for (int64_t hour = 3; hour >= 1; --hour) back.Load(kT + hour * 3600, Find(files, kT + hour * 3600)->text);
  EXPECT_EQ(back.Bytes(), hour_bytes * 2);
  const auto removed = back.TakeFiles(kNow + 4 * 3600);
  ASSERT_EQ(removed.size(), 1u);
  EXPECT_EQ(removed[0].hour, kT + 3600);
}

// kMaxSensors over 49 h with long values stay below kMaxHistoryBytes: the limit only acts past the plan.
TEST(SensorHistory, FullHistoryFitsTheLimit) {
  SensorHistory h;
  const double end = kNow + 49 * 3600;
  for (size_t i = 0; i < kMaxSensors; ++i) {
    const std::string id = "sensor_with_a_long_id_" + std::to_string(i);
    ASSERT_TRUE(h.Describe(id, "A long name of sensor " + std::to_string(i), "deg.C", true));
    for (int64_t m = 0; m < int64_t{49} * 60; ++m) {
      const double t = kNow + static_cast<double>(m * 60);
      h.Add(id, -123.456789 - t * 1e-9, t);
      h.Add(id, -1234567.89, t + 1);
    }
  }
  h.TakeFiles(kNow + kKeepSeconds);
  EXPECT_EQ(h.Buckets(), kMaxSensors * 49 * 60);
  EXPECT_LT(h.Bytes(), kMaxHistoryBytes);
  EXPECT_GT(h.Bytes(), kMaxHistoryBytes / 2);
  // The answer: too much at 60 and 300 s, all of it at 900 s and below the limit of the catalog.
  const json answer = h.Query({}, end - 1);
  EXPECT_EQ(answer["step"], 900);
  EXPECT_FALSE(answer["truncated"].get<bool>());
  EXPECT_LE(answer.dump().size(), static_cast<size_t>(mowbite_api::kHistoryMaxAnswerBytes));
}

TEST(SensorHistory, AnswerTooLargeEvenAt900LosesTheOldestPoints) {
  SensorHistory h = WithVoltage();
  for (int m = 0; m < 10 * 60; ++m) h.Add("v", 25.5, kNow + m * 60);
  HistoryQuery q;
  q.from = kT;
  const double now = kNow + 10 * 3600;
  const json full = h.Query(q, now);
  ASSERT_EQ(full["step"], 60);
  ASSERT_EQ(full["sensors"][0]["points"].size(), 600u);
  // Room for about 20 points of 15 min: 900 s with the newest of them.
  const size_t room = h.Query(q, now, 0)["sensors"].dump().size() + size_t{20 * 25 + 60};
  const json cut = h.Query(q, now, room);
  EXPECT_EQ(cut["step"], 900);
  EXPECT_TRUE(cut["truncated"].get<bool>());
  EXPECT_LE(cut.dump().size(), room);
  const json& points = cut["sensors"][0]["points"];
  ASSERT_FALSE(points.empty());
  EXPECT_LT(points.size(), 40u);
  EXPECT_EQ(cut["from"], points[0][0]);
  EXPECT_EQ(points.back()[0], kT + int64_t{10 * 3600 - 900});
  // One more point would not fit.
  q.from = cut["from"].get<int64_t>() - 900;
  q.step = 900;
  EXPECT_GT(h.Query(q, now, 1 << 30).dump().size(), room);
  // A step of 60 that fits stays 60.
  q.from = kT + int64_t{10 * 3600 - 300};
  q.step = 60;
  EXPECT_EQ(h.Query(q, now, room)["step"], 60);
}
