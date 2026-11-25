#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "mowbite_api/catalog.h"
#include "mowbite_stats/stop_book.h"

using namespace mowbite_stats;

namespace {

// Wednesday 2026-11-25 12:00 UTC, in ISO week 2026-W48.
constexpr double kWall = 1795608000;

json State(const std::string& state, const std::string& sub_state = "", json more = json::object()) {
  more["type"] = "STATE";
  more["state"] = state;
  more["sub_state"] = sub_state;
  return more;
}

json Event(const std::string& type, json more = json::object()) {
  more["type"] = type;
  return more;
}

// A book on a monotonic clock from 0, with the wall clock kWall at 0; STOP incidents count up from stop-1.
struct Book {
  StopBook book{nullptr};
  std::vector<std::string> opened;

  Book() {
    book.SetIncidents({[this](const std::string& reason) {
                         opened.push_back(reason);
                         return "stop-" + std::to_string(opened.size());
                       },
                       [] { return std::string("collecting"); }});
  }
  void At(double mono, const json& event) { book.OnEvent(event, mono, kWall + mono); }
  bool Tick(double mono) { return book.Tick(mono); }
  // The entry of the last tick that added one.
  const StopEntry& Last() const { return book.last().front(); }
};

}  // namespace

TEST(StopBook, IsoWeeks) {
  EXPECT_EQ(WeekStart(kWall), 1795392000);  // Monday 2026-11-23
  EXPECT_EQ(WeekName(WeekStart(kWall)), "2026-W48");
  EXPECT_EQ(WeekStart(1795392000), 1795392000);
  EXPECT_EQ(WeekStart(1795392000 - 1), 1795392000 - 7 * 86400);
  // 2021-01-03 is in the last week of 2020, 2024-12-30 in the first of 2025.
  EXPECT_EQ(WeekName(WeekStart(1609632000)), "2020-W53");
  EXPECT_EQ(WeekStart(1609632000), 1609113600);
  EXPECT_EQ(WeekName(WeekStart(1736121599)), "2025-W01");
  EXPECT_EQ(WeekStart(1736121599), 1735516800);
  EXPECT_EQ(OldestWeek(kWall), 1795392000 - 7 * 7 * 86400);
}

TEST(StopBook, ACauseWithinTheWindowExplains) {
  Book b;
  b.At(0, State("MOWING"));
  EXPECT_EQ(b.book.activity(), "MOWING");
  b.At(10, Event("DOCKING", {{"reason", "RAIN"}}));
  b.At(10, State("DOCKING"));
  EXPECT_FALSE(b.Tick(11.9));
  EXPECT_TRUE(b.Tick(12.0));
  const StopEntry& entry = b.Last();
  EXPECT_EQ(entry.kind, "stop");
  EXPECT_EQ(entry.activity, "MOWING");
  EXPECT_EQ(entry.reason, "DOCKING/RAIN");
  EXPECT_EQ(entry.cls, mowbite_api::kCause);
  EXPECT_TRUE(entry.explained);
  EXPECT_EQ(entry.incident, "collecting");
  EXPECT_DOUBLE_EQ(entry.t, kWall + 10);
  // Docked at the end: no stop.
  b.At(50, Event("DOCKED"));
  b.At(50, State("IDLE"));
  EXPECT_FALSE(b.Tick(60));
  EXPECT_EQ(b.book.last().size(), 1u);
}

TEST(StopBook, NormalEndsAndCommandsAreNoStops) {
  Book b;
  b.At(0, State("UNDOCKING"));
  b.At(5, Event("UNDOCKED"));
  b.At(5, State("MOWING"));
  b.At(100, Event("JOB_COMPLETE"));
  b.At(101, State("DOCKING"));
  b.At(150, State("IDLE"));
  b.At(150.5, Event("DOCKED"));
  // A pause by the app, a drive that arrived, a calibration drive that ended, the schedule sending it home.
  b.At(200, State("MOWING"));
  b.At(210, State("PAUSED", "", {{"reason", "MANUAL"}, {"client", "app"}}));
  b.At(220, State("IDLE", "DRIVING"));
  b.At(230, Event("DRIVE", {{"phase", "end"}, {"arrived", true}, {"reason", "DONE"}}));
  b.At(230, State("IDLE"));
  b.At(240, State("IDLE", "CALIBRATING"));
  b.At(300, Event("CALIBRATED", {{"status", "failed"}}));
  b.At(300, State("IDLE"));
  b.At(310, State("MOWING"));
  b.At(400, Event("DOCKING", {{"reason", "SCHEDULE"}, {"detail", "end"}}));
  b.At(400, State("DOCKING"));
  b.At(450, Event("DOCKED"));
  b.At(450, State("IDLE"));
  EXPECT_FALSE(b.Tick(500));
  EXPECT_TRUE(b.book.last().empty());
  EXPECT_TRUE(b.opened.empty());
}

TEST(StopBook, ASymptomAloneOpensAnIncident) {
  Book b;
  b.At(0, State("MOWING"));
  b.At(30, State("IDLE", "", {{"reason", "FOLLOWER_MISSING"}}));
  EXPECT_TRUE(b.Tick(32));
  const StopEntry& entry = b.Last();
  EXPECT_FALSE(entry.explained);
  EXPECT_EQ(entry.reason, "STATE/FOLLOWER_MISSING");
  EXPECT_EQ(entry.cls, mowbite_api::kSymptom);
  EXPECT_EQ(entry.incident, "stop-1");
  EXPECT_EQ(b.opened, std::vector<std::string>{"STATE/FOLLOWER_MISSING"});
  // Nothing at all: UNKNOWN; a cause only after the window is too late.
  b.At(40, State("IDLE", "DRIVING"));
  b.At(50, State("IDLE"));
  b.At(52.5, Event("DRIVE", {{"phase", "end"}, {"arrived", false}, {"reason", "POSITION_ACCURACY"}}));
  EXPECT_TRUE(b.Tick(52.5));
  EXPECT_EQ(b.Last().reason, "UNKNOWN");
  EXPECT_EQ(b.Last().activity, "DRIVING");
  EXPECT_FALSE(b.Last().explained);
  // The same cause within the window explains, also when it comes after the end.
  b.At(60, State("IDLE", "DRIVING"));
  b.At(70, State("IDLE"));
  b.At(71.5, Event("DRIVE", {{"phase", "end"}, {"arrived", false}, {"reason", "POSITION_ACCURACY"}}));
  EXPECT_TRUE(b.Tick(72));
  EXPECT_EQ(b.Last().reason, "DRIVE/POSITION_ACCURACY");
  EXPECT_TRUE(b.Last().explained);
  const json week = b.book.Json(kWall + 100)["weeks"][0];
  EXPECT_EQ(week["stops"], 3);
  EXPECT_EQ(week["unexplained"], 2);
}

TEST(StopBook, AnEmergencyExplainsByTheClassOfItsBits) {
  Book b;
  b.At(0, State("MOWING"));
  // LATCH and LIFT: LIFT is the cause.
  b.At(10, Event("EMERGENCY", {{"emergency", true}, {"reason", (1 << 0) | (1 << 3)}}));
  b.At(10.1, State("IDLE", "", {{"reason", "EMERGENCY"}}));
  EXPECT_TRUE(b.Tick(13));
  EXPECT_EQ(b.Last().reason, "EMERGENCY/LIFT");
  EXPECT_TRUE(b.Last().explained);
  // LATCH and TIMEOUT_HIGH_LEVEL tell only that a part went silent.
  b.At(20, State("MOWING"));
  b.At(30, Event("EMERGENCY", {{"emergency", true}, {"reason", (1 << 0) | (1 << 6)}}));
  b.At(30.1, State("IDLE", "", {{"reason", "EMERGENCY"}}));
  EXPECT_TRUE(b.Tick(33));
  EXPECT_EQ(b.Last().reason, "EMERGENCY/TIMEOUT_HIGH_LEVEL");
  EXPECT_FALSE(b.Last().explained);
  // The end of an emergency explains nothing.
  b.At(40, State("MOWING"));
  b.At(50, Event("EMERGENCY", {{"emergency", false}, {"reason", 0}}));
  b.At(50, State("IDLE"));
  EXPECT_TRUE(b.Tick(53));
  EXPECT_EQ(b.Last().reason, "UNKNOWN");
}

TEST(StopBook, AnUnexpectedRestartWhileBusyIsAStop) {
  Book b;
  b.At(1, Event("UNEXPECTED_RESTART", {{"reason", "POWER_LOSS"}, {"state_before", "MOWING"}}));
  b.At(2, Event("UNEXPECTED_RESTART", {{"reason", "OS_CRASH"}, {"state_before", "IDLE"}}));
  b.At(3, Event("UNEXPECTED_RESTART", {{"reason", "KERNEL_CRASH"}, {"state_before", "DRIVING"}}));
  EXPECT_TRUE(b.Tick(10));
  ASSERT_EQ(b.book.last().size(), 2u);
  EXPECT_EQ(b.book.last()[1].activity, "MOWING");
  EXPECT_EQ(b.book.last()[1].reason, "UNEXPECTED_RESTART/POWER_LOSS");
  EXPECT_FALSE(b.book.last()[1].explained);
  EXPECT_EQ(b.book.last()[0].activity, "DRIVING");
  EXPECT_EQ(b.book.last()[0].reason, "UNEXPECTED_RESTART/KERNEL_CRASH");
}

TEST(StopBook, AnEventSentAgainCountsOnce) {
  Book b;
  b.At(0, State("MOWING", "", {{"id", "a"}}));
  b.At(5, State("IDLE", "", {{"id", "b"}}));
  b.At(5.5, State("MOWING", "", {{"id", "a"}}));
  b.At(6, State("IDLE", "", {{"id", "b"}}));
  b.Tick(20);
  EXPECT_EQ(b.book.last().size(), 1u);
  EXPECT_EQ(b.book.activity(), "");
}

TEST(StopBook, StandstillsAreOnlyObserved) {
  Book b;
  const auto pose = [&](double mono, double x, bool paused = false) {
    return b.book.OnPose(paused, true, x, 0, mono, kWall + mono);
  };
  EXPECT_FALSE(pose(0, 0));  // idle
  b.At(0, State("MOWING"));
  EXPECT_FALSE(pose(1, 0));
  EXPECT_FALSE(pose(60, 0.04));
  EXPECT_TRUE(pose(61, 0.04));
  EXPECT_EQ(b.Last().kind, "standstill");
  EXPECT_EQ(b.Last().reason, "STANDSTILL");
  EXPECT_EQ(b.Last().incident, "collecting");
  EXPECT_FALSE(pose(200, 0.04));  // once per standstill
  EXPECT_FALSE(pose(201, 0.10));  // moved: anew
  EXPECT_FALSE(pose(300, 0.10, true));  // a pause starts anew
  EXPECT_FALSE(pose(330, 0.10));
  EXPECT_FALSE(pose(389, 0.10));
  EXPECT_TRUE(pose(390, 0.10));
  // A calibration drive stands on purpose.
  b.At(400, State("IDLE", "CALIBRATING"));
  EXPECT_FALSE(pose(401, 0));
  EXPECT_FALSE(pose(500, 0));
  const json week = b.book.Json(kWall + 500)["weeks"][0];
  EXPECT_EQ(week["standstills"], 2);
  EXPECT_EQ(week["stops"], 0);
  EXPECT_EQ(week["unexplained"], 0);
}

TEST(StopBook, AMarkWithoutAStopCountsAsUnexplained) {
  Book b;
  EXPECT_TRUE(b.book.OnMark("mark-1", kWall));
  EXPECT_EQ(b.Last().kind, "mark");
  EXPECT_EQ(b.Last().activity, "IDLE");
  EXPECT_EQ(b.Last().incident, "mark-1");
  EXPECT_FALSE(b.book.OnMark("mark-1", kWall + 10));  // grew the same incident
  b.At(1000, State("MOWING"));
  b.At(1010, State("IDLE"));
  b.Tick(1020);
  EXPECT_FALSE(b.book.OnMark("mark-2", kWall + 1010 + kStopMarkS - 1));
  EXPECT_TRUE(b.book.OnMark("mark-3", kWall + 1010 + kStopMarkS + 1));
  const json week = b.book.Json(kWall + 2000)["weeks"][0];
  EXPECT_EQ(week["stops"], 1);
  EXPECT_EQ(week["unexplained"], 3);
  EXPECT_EQ(week["reasons"][0], json({{"reason", "MARK"}, {"class", "symptom"}, {"count", 2}}));
}

TEST(StopBook, KeepsEightWeeksTheLastEntriesAndLimitedReasons) {
  StopBook book(nullptr);
  const double week = 7 * 86400;
  // One stop per week over ten weeks, the newest in the week of kWall.
  for (int i = 9; i >= 0; i--) {
    book.OnEvent(State("MOWING"), 1000.0 * (10 - i), kWall - i * week);
    book.OnEvent(State("IDLE"), 1000.0 * (10 - i) + 1, kWall - i * week + 1);
    book.Tick(1000.0 * (10 - i) + 5);
  }
  json answer = book.Json(kWall);
  ASSERT_EQ(answer["weeks"].size(), static_cast<size_t>(mowbite_api::kStopBookWeeks));
  EXPECT_EQ(answer["weeks"][0]["start"], OldestWeek(kWall));
  EXPECT_EQ(answer["weeks"].back()["week"], "2026-W48");
  EXPECT_EQ(answer["last"].size(), 10u);
  // More than kStopBookLast entries and kStopBookMaxReasons reasons in one week.
  double mono = 20000;
  for (size_t i = 0; i <= kStopBookMaxReasons; i++) {
    book.OnEvent(State("MOWING"), mono, kWall);
    book.OnEvent(Event("X", {{"reason", "R" + std::to_string(i)}}), mono + 1, kWall);
    book.OnEvent(State("IDLE"), mono + 1, kWall);
    mono += 10;
    book.Tick(mono);
  }
  answer = book.Json(kWall);
  EXPECT_EQ(answer["last"].size(), static_cast<size_t>(mowbite_api::kStopBookLast));
  EXPECT_EQ(answer["last"][0]["reason"], "X/R" + std::to_string(kStopBookMaxReasons));
  const json& reasons = answer["weeks"].back()["reasons"];
  EXPECT_EQ(reasons.size(), kStopBookMaxReasons + 1);  // UNKNOWN of the first stops, R0 to R498, OTHER
  bool other = false;
  for (const json& row : reasons) other = other || row["reason"] == "OTHER";
  EXPECT_TRUE(other);
}

TEST(StopBook, StoredAndReadBack) {
  Book b;
  b.At(0, State("MOWING"));
  b.At(10, State("IDLE", "", {{"reason", "FOLLOWER_MISSING"}}));
  b.Tick(20);
  b.book.OnMark("m", kWall + 1000);
  const json stored = b.book.Json(kWall + 1000);
  EXPECT_EQ(StopBook(stored).Json(kWall + 1000), stored);
  // blade_h only in the answer, 0 for a week without.
  const std::map<int64_t, double> blade_h = {{WeekStart(kWall), 1.25}};
  EXPECT_EQ(b.book.Json(kWall, &blade_h)["weeks"][0]["blade_h"], 1.25);
  EXPECT_FALSE(stored["weeks"][0].contains("blade_h"));
  // A broken file is an empty book.
  json broken = stored;
  broken["last"][0]["t"] = "now";
  EXPECT_TRUE(StopBook(broken).last().empty());
  EXPECT_TRUE(StopBook(json::array()).Json(kWall)["weeks"].empty());
}
