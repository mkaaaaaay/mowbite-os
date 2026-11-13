#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "mowbite_schedule/scheduler.h"

namespace mowbite_schedule {
namespace {

constexpr char kBerlin[] = "CET-1CEST,M3.5.0,M10.5.0/3";
// Monday 15 June 2026, Berlin: the sun is up from about 04:45 to 21:30.
const Date kMonday{2026, 6, 15};

class SchedulerTest : public ::testing::Test {
 protected:
  // A schedule switched on in Berlin at Monday 00:00, plan 10:00 every day over every area; extra keys replace those.
  ScheduleFile File(const json& extra = json::object()) {
    ScheduleFile file;
    json change = {{"enabled", 1}, {"tz", kBerlin}, {"plans", {{{"days", {1, 2, 3, 4, 5, 6, 7}}, {"time", "10:00"}}}}};
    change.update(extra);
    const std::optional<Invalid> bad = ApplySchedule(change, true, 0, &file);
    EXPECT_FALSE(bad) << (bad ? bad->message : "");
    file.changed_at = At(kMonday, "00:00");
    return file;
  }

  static int64_t At(const Date& date, const std::string& time, int seconds = 0) {
    std::string reason;
    return TimeZone::Parse(kBerlin, &reason)->ToUnix(date, *ParseClockTime(time)) + seconds;
  }

  // All well for a start at that moment.
  static Facts Good(int64_t wall) {
    Facts facts;
    facts.wall = wall;
    facts.mono = static_cast<double>(wall - At(kMonday, "00:00"));
    facts.clock_ok = true;
    facts.logic_fresh = true;
    facts.state = "IDLE";
    facts.docked = true;
    facts.battery = 100;
    facts.rain_sensor = true;
    facts.latitude = 52.52;
    facts.longitude = 13.40;
    facts.areas = {{"a", true}, {"b", true}, {"c", false}};
    return facts;
  }

  void Make(const ScheduleFile& file, bool available = true, const json& state = nullptr) {
    scheduler_.emplace(file, available, state, nullptr, At(kMonday, "00:00"));
  }

  Output Tick(const Facts& facts) {
    scheduler_->Tick(facts);
    return scheduler_->TakeOutput();
  }

  Output TickAt(const Date& date, const std::string& time, int seconds = 0) {
    return Tick(Good(At(date, time, seconds)));
  }

  std::vector<std::string> Words() const {
    std::vector<std::string> words;
    for (const json& entry : scheduler_->LogEntries(kLogEntries)) {
      std::string word = entry["what"].get<std::string>();
      if (entry["detail"].is_string()) word += " " + entry["detail"].get<std::string>();
      words.push_back(word);
    }
    return words;
  }

  // The start of the 10:00 plan on Monday, taken by the logic.
  RunRequest StartAndAccept() {
    const Output out = TickAt(kMonday, "10:00");
    EXPECT_TRUE(out.start);
    if (!out.start) return RunRequest();
    scheduler_->OnStartAnswer(out.start->run_id, true, "", nullptr);
    return *out.start;
  }

  void Status(const std::string& run_id, Phase phase, const std::string& end_code = "",
              const std::string& detail = "") {
    scheduler_->OnRunStatus(RunStatus{run_id, phase, end_code, detail});
  }

  std::optional<Scheduler> scheduler_;
};

using Lines = std::vector<std::string>;

TEST_F(SchedulerTest, StartsInTheWindowOnce) {
  Make(File());
  EXPECT_FALSE(TickAt(kMonday, "09:59", 59).start);
  const Output out = TickAt(kMonday, "10:00");
  ASSERT_TRUE(out.start);
  EXPECT_TRUE(out.state_changed);
  EXPECT_EQ(out.start->run_id, "2026-06-15T10:00");
  EXPECT_TRUE(out.start->area_ids.empty());
  EXPECT_TRUE(out.start->pass_over.empty());
  EXPECT_FALSE(out.start->restart);
  EXPECT_FALSE(out.start->dark_allowed);
  EXPECT_TRUE(out.start->until_sunset);
  EXPECT_EQ(out.start->end_in_s, 0);
  scheduler_->StartFailed(out.start->run_id, "storage");
  EXPECT_FALSE(TickAt(kMonday, "10:00", 30).start);
  EXPECT_EQ(Words(), (Lines{"start_failed storage"}));
  EXPECT_EQ(scheduler_->StateJson()["handled"]["2026-06-15"], json({"10:00"}));
}

TEST_F(SchedulerTest, T0_NotReleasedOrSwitchedOffDoesNothing) {
  Make(File(), false);
  EXPECT_FALSE(TickAt(kMonday, "10:00").start);
  EXPECT_FALSE(TickAt(kMonday, "12:00").start);
  EXPECT_TRUE(Words().empty());
  EXPECT_FALSE(scheduler_->NextStart());
  Make(File({{"enabled", 0}}));
  EXPECT_FALSE(TickAt(kMonday, "10:00").start);
  EXPECT_TRUE(Words().empty());
  EXPECT_EQ(scheduler_->StateJson()["handled"], json::object());
}

TEST_F(SchedulerTest, T1_MissedWhileTheOsWasOffIsNotMadeUp) {
  Make(File());
  EXPECT_FALSE(TickAt(kMonday, "12:00").start);
  EXPECT_EQ(Words(), (Lines{"skip_missed"}));
  EXPECT_EQ(scheduler_->LogEntries(1)[0]["at"], "10:00");
  // Yesterday's too, once.
  ScheduleFile file = File();
  file.changed_at = 0;
  Make(file);
  TickAt(kMonday, "12:00");
  TickAt(kMonday, "12:00", 1);
  EXPECT_EQ(Words(), (Lines{"skip_missed", "skip_missed"}));
}

TEST_F(SchedulerTest, T1_ClockJumpForwardOverAPlan) {
  Make(File());
  TickAt(kMonday, "09:59", 30);
  EXPECT_FALSE(TickAt(kMonday, "10:05").start);
  EXPECT_EQ(Words(), (Lines{"skip_missed"}));
}

TEST_F(SchedulerTest, T1_ClockJumpBackDoesNotFireAgain) {
  Make(File());
  StartAndAccept();
  const std::string run_id = "2026-06-15T10:00";
  Status(run_id, Phase::kEnded, "UNDOCKING_FAILED");
  TickAt(kMonday, "09:58");
  EXPECT_FALSE(TickAt(kMonday, "10:00", 10).start);
  EXPECT_EQ(Words(), (Lines{"start_failed UNDOCKING_FAILED"}));
}

TEST_F(SchedulerTest, T1_RestartWhileWaitingEndsTheWait) {
  const json state = {{"schema", 1},
                      {"handled", {{"2026-06-15", {"10:00"}}}},
                      {"waiting", {{"date", "2026-06-15"}, {"time", "10:00"}, {"kind", "battery"}}}};
  Make(File(), true, state);
  EXPECT_EQ(Words(), (Lines{"skip_missed"}));
  EXPECT_TRUE(scheduler_->TakeOutput().state_changed);
  EXPECT_FALSE(TickAt(kMonday, "10:00", 30).start);
  EXPECT_FALSE(TickAt(kMonday, "10:30").start);
  EXPECT_TRUE(scheduler_->StateJson()["waiting"].is_null());
  EXPECT_EQ(Words(), (Lines{"skip_missed"}));
}

TEST_F(SchedulerTest, T2_UnsafeClockSkipsWithoutStoringTheDate) {
  Make(File());
  Facts facts = Good(At(kMonday, "10:00"));
  facts.clock_ok = false;
  facts.clock_reason = "never_synced";
  EXPECT_FALSE(Tick(facts).start);
  facts.wall += 1;
  Tick(facts);
  EXPECT_EQ(Words(), (Lines{"skip_clock never_synced"}));
  EXPECT_EQ(scheduler_->StateJson()["handled"], json::object());
  EXPECT_FALSE(scheduler_->NextStart());
  // Safe again: neither a start nor a second word for the same plan.
  EXPECT_FALSE(TickAt(kMonday, "10:00", 30).start);
  TickAt(kMonday, "10:05");
  EXPECT_EQ(Words(), (Lines{"skip_clock never_synced"}));
}

TEST_F(SchedulerTest, T3_PauseOverAllAreas) {
  Make(File({{"pause", {{"until", "2026-06-15"}}}}));
  EXPECT_FALSE(TickAt(kMonday, "10:00").start);
  EXPECT_EQ(Words(), (Lines{"skip_paused"}));
  // Over by the next day.
  EXPECT_TRUE(TickAt(Date{2026, 6, 16}, "10:00").start);
}

TEST_F(SchedulerTest, T4_PausedAndUnknownAreasPassedOver) {
  Make(File({{"plans", {{{"days", {1}}, {"time", "10:00"}, {"areas", {"a", "x", "b"}}}}},
             {"pause", {{"until", "2026-06-15"}, {"areas", {"b"}}}}}));
  const Output out = TickAt(kMonday, "10:00");
  ASSERT_TRUE(out.start);
  EXPECT_EQ(out.start->area_ids, (std::vector<std::string>{"a", "x", "b"}));
  EXPECT_EQ(out.start->pass_over, (std::vector<std::string>{"x", "b"}));
  EXPECT_EQ(Words(), (Lines{"paused_area b", "unknown_area x"}));
}

TEST_F(SchedulerTest, T4_AllAreasWithAPausedOne) {
  Make(File({{"pause", {{"until", "2026-06-15"}, {"areas", {"a"}}}}}));
  const Output out = TickAt(kMonday, "10:00");
  ASSERT_TRUE(out.start);
  EXPECT_TRUE(out.start->area_ids.empty());
  EXPECT_EQ(out.start->pass_over, (std::vector<std::string>{"a"}));
}

TEST_F(SchedulerTest, T4_NothingLeft) {
  Make(File({{"plans", {{{"days", {1}}, {"time", "10:00"}, {"areas", {"b"}}}}},
             {"pause", {{"until", "2026-06-15"}, {"areas", {"b"}}}}}));
  EXPECT_FALSE(TickAt(kMonday, "10:00").start);
  EXPECT_EQ(Words(), (Lines{"skip_paused"}));
  // An inactive area paused leaves nothing to mow either way.
  Make(File({{"plans", {{{"days", {1}}, {"time", "10:00"}, {"areas", {"c"}}}}},
             {"pause", {{"until", "2026-06-15"}, {"areas", {"c"}}}}}));
  EXPECT_FALSE(TickAt(kMonday, "10:00").start);
  EXPECT_EQ(Words(), (Lines{"skip_no_area"}));
  // A pause over all areas covers unknown ones too.
  Make(File({{"plans", {{{"days", {1}}, {"time", "10:00"}, {"areas", {"x"}}}}}, {"pause", {{"until", "2026-06-15"}}}}));
  EXPECT_FALSE(TickAt(kMonday, "10:00").start);
  EXPECT_EQ(Words(), (Lines{"skip_paused"}));
  Make(File({{"plans", {{{"days", {1}}, {"time", "10:00"}, {"areas", {"x", "c"}}}}}}));
  EXPECT_FALSE(TickAt(kMonday, "10:00").start);
  EXPECT_EQ(Words(), (Lines{"skip_no_area"}));
  Make(File());
  Facts facts = Good(At(kMonday, "10:00"));
  facts.areas.clear();
  EXPECT_FALSE(Tick(facts).start);
  EXPECT_EQ(Words(), (Lines{"skip_no_area"}));
}

TEST_F(SchedulerTest, T5_EndReachedWhileWaiting) {
  Make(File({{"plans", {{{"days", {1}}, {"time", "10:00"}, {"end", "11:00"}}}}}));
  Facts facts = Good(At(kMonday, "10:00"));
  facts.battery = 50;
  Tick(facts);
  facts.wall = At(kMonday, "10:59", 59);
  Tick(facts);
  EXPECT_EQ(Words(), (Lines{"waiting_battery 50"}));
  facts.wall = At(kMonday, "11:00");
  Tick(facts);
  EXPECT_EQ(Words(), (Lines{"waiting_battery 50", "skip_end"}));
  EXPECT_TRUE(scheduler_->StateJson()["waiting"].is_null());
}

TEST_F(SchedulerTest, T5_EndOnTheNextDayAndTheTimeLeft) {
  Make(File({{"plans", {{{"days", {1}}, {"time", "10:00"}, {"end", "09:00"}}}}}));
  const Output out = TickAt(kMonday, "10:00", 5);
  ASSERT_TRUE(out.start);
  EXPECT_EQ(out.start->end_in_s, 23 * 3600 - 5);
  Make(File({{"plans", {{{"days", {1}}, {"time", "10:00"}, {"end", "12:30"}}}}}));
  EXPECT_EQ(TickAt(kMonday, "10:00").start->end_in_s, 2 * 3600 + 1800);
}

TEST_F(SchedulerTest, T6_LogicSilentWithinTheWindowThenBack) {
  Make(File());
  Facts facts = Good(At(kMonday, "10:00"));
  facts.logic_fresh = false;
  EXPECT_FALSE(Tick(facts).start);
  EXPECT_TRUE(Words().empty());
  EXPECT_TRUE(TickAt(kMonday, "10:00", 40).start);
}

TEST_F(SchedulerTest, T6_LogicSilentTheWholeWindow) {
  Make(File());
  Facts facts = Good(At(kMonday, "10:00"));
  facts.logic_fresh = false;
  for (int s = 0; s < 70; s += 7) {
    facts.wall = At(kMonday, "10:00", s);
    EXPECT_FALSE(Tick(facts).start);
  }
  EXPECT_EQ(Words(), (Lines{"skip_offline"}));
}

TEST_F(SchedulerTest, T7_Emergency) {
  Make(File());
  Facts facts = Good(At(kMonday, "10:00"));
  facts.emergency = true;
  EXPECT_FALSE(Tick(facts).start);
  EXPECT_EQ(Words(), (Lines{"skip_emergency"}));
}

TEST_F(SchedulerTest, T8_Busy) {
  Make(File());
  Facts facts = Good(At(kMonday, "10:00"));
  facts.state = "MOWING";
  EXPECT_FALSE(Tick(facts).start);
  EXPECT_EQ(Words(), (Lines{"skip_busy MOWING"}));
}

TEST_F(SchedulerTest, T8_BusyWithOwnRun) {
  Make(File({{"plans", {{{"days", {1}}, {"time", "10:00"}}, {{"days", {1}}, {"time", "10:01"}}}}}));
  StartAndAccept();
  Status("2026-06-15T10:00", Phase::kAnnouncing);
  EXPECT_FALSE(TickAt(kMonday, "10:01").start);
  EXPECT_EQ(Words(), (Lines{"skip_busy ANNOUNCING"}));
  EXPECT_EQ(scheduler_->StateJson()["handled"]["2026-06-15"], json({"10:00", "10:01"}));
}

TEST_F(SchedulerTest, T9_NotDocked) {
  Make(File());
  Facts facts = Good(At(kMonday, "10:00"));
  facts.docked = false;
  EXPECT_FALSE(Tick(facts).start);
  EXPECT_EQ(Words(), (Lines{"skip_not_docked"}));
}

TEST_F(SchedulerTest, T10_Rain) {
  Facts facts = Good(At(kMonday, "10:00"));
  facts.rain_wet = true;
  Make(File());
  EXPECT_FALSE(Tick(facts).start);
  EXPECT_EQ(Words(), (Lines{"skip_rain"}));
  Make(File({{"skip_rain", 0}}));
  facts.rain_mode = 1;
  EXPECT_FALSE(Tick(facts).start);
  EXPECT_EQ(Words(), (Lines{"skip_rain"}));
  Make(File({{"skip_rain", 0}}));
  facts.rain_mode = 0;
  EXPECT_TRUE(Tick(facts).start);
  Make(File());
  facts.rain_sensor = false;
  EXPECT_TRUE(Tick(facts).start);
}

TEST_F(SchedulerTest, T11_DarkAndNoPosition) {
  Make(File({{"plans", {{{"days", {1}}, {"time", "23:00"}}}}}));
  EXPECT_FALSE(TickAt(kMonday, "23:00").start);
  EXPECT_EQ(Words(), (Lines{"skip_dark"}));
  Make(File({{"plans", {{{"days", {1}}, {"time", "23:00"}, {"dark", 1}}}}}));
  Output out = TickAt(kMonday, "23:00");
  ASSERT_TRUE(out.start);
  EXPECT_TRUE(out.start->dark_allowed);
  EXPECT_FALSE(out.start->until_sunset);
  Make(File({{"plans", {{{"days", {1}}, {"time", "23:00"}, {"dark", 1}, {"sunset", 1}}}}}));
  out = TickAt(kMonday, "23:00");
  ASSERT_TRUE(out.start);
  EXPECT_TRUE(out.start->until_sunset);
  Facts facts = Good(At(kMonday, "10:00"));
  facts.latitude.reset();
  Make(File());
  EXPECT_FALSE(Tick(facts).start);
  EXPECT_EQ(Words(), (Lines{"skip_no_position"}));
  Make(File({{"plans", {{{"days", {1}}, {"time", "10:00"}, {"dark", 1}}}}}));
  EXPECT_TRUE(Tick(facts).start);
}

TEST_F(SchedulerTest, T12_WaitsForTheBattery) {
  Make(File());
  Facts facts = Good(At(kMonday, "10:00"));
  facts.battery = 60;
  EXPECT_FALSE(Tick(facts).start);
  EXPECT_EQ(scheduler_->StateJson()["waiting"],
            (json{{"date", "2026-06-15"}, {"time", "10:00"}, {"kind", "battery"}}));
  EXPECT_EQ(scheduler_->NextStart(), facts.wall);
  facts.wall = At(kMonday, "10:30");
  facts.battery = 79;
  EXPECT_FALSE(Tick(facts).start);
  facts.wall = At(kMonday, "11:00");
  facts.battery = 80;
  ASSERT_TRUE(Tick(facts).start);
  EXPECT_EQ(Words(), (Lines{"waiting_battery 60"}));
  EXPECT_TRUE(scheduler_->StateJson()["waiting"].is_null());
}

TEST_F(SchedulerTest, T12_BatteryStillLowAfterTwoHours) {
  Make(File());
  Facts facts = Good(At(kMonday, "10:00"));
  facts.battery = 60;
  Tick(facts);
  facts.wall = At(kMonday, "11:59", 59);
  Tick(facts);
  facts.battery = 70;
  facts.wall = At(kMonday, "12:00");
  EXPECT_FALSE(Tick(facts).start);
  EXPECT_EQ(Words(), (Lines{"waiting_battery 60", "skip_battery 70"}));
}

TEST_F(SchedulerTest, T13_WaitsForAHold) {
  Make(File());
  Facts facts = Good(At(kMonday, "10:00"));
  facts.hold = true;
  Tick(facts);
  EXPECT_EQ(scheduler_->StateJson()["waiting"]["kind"], "hold");
  facts.wall = At(kMonday, "12:00");
  EXPECT_FALSE(Tick(facts).start);
  EXPECT_EQ(Words(), (Lines{"waiting_hold", "skip_hold"}));
  Make(File());
  facts.wall = At(kMonday, "10:00");
  Tick(facts);
  facts.hold = false;
  facts.wall = At(kMonday, "10:20");
  EXPECT_TRUE(Tick(facts).start);
}

TEST_F(SchedulerTest, T12_T13_BatteryThenHoldEachLoggedOnce) {
  Make(File());
  Facts facts = Good(At(kMonday, "10:00"));
  facts.battery = 60;
  Tick(facts);
  facts.wall += 600;
  facts.battery = 90;
  facts.hold = true;
  Tick(facts);
  facts.wall += 600;
  Tick(facts);
  EXPECT_EQ(Words(), (Lines{"waiting_battery 60", "waiting_hold"}));
  EXPECT_EQ(scheduler_->StateJson()["waiting"]["kind"], "hold");
}

TEST_F(SchedulerTest, WaitEndsWhenTheNextPlanIsDue) {
  Make(File({{"plans", {{{"days", {1}}, {"time", "10:00"}}, {{"days", {1}}, {"time", "10:30"}}}}}));
  Facts facts = Good(At(kMonday, "10:00"));
  facts.battery = 60;
  Tick(facts);
  facts.wall = At(kMonday, "10:30");
  Tick(facts);
  EXPECT_EQ(Words(), (Lines{"waiting_battery 60", "skip_battery 60", "waiting_battery 60"}));
  EXPECT_EQ(scheduler_->StateJson()["waiting"]["time"], "10:30");
}

TEST_F(SchedulerTest, WaitEndsWhenThePlanIsGone) {
  Make(File());
  Facts facts = Good(At(kMonday, "10:00"));
  facts.battery = 60;
  Tick(facts);
  ASSERT_FALSE(scheduler_->SetSchedule({{"plans", {{{"days", {1}}, {"time", "10:10"}}}}}, At(kMonday, "10:05")));
  facts.wall = At(kMonday, "10:06");
  EXPECT_TRUE(Tick(facts).state_changed);
  facts.wall += 1;
  Tick(facts);
  EXPECT_EQ(Words(), (Lines{"waiting_battery 60", "skip_gone"}));
  EXPECT_TRUE(scheduler_->StateJson()["waiting"].is_null());
}

TEST_F(SchedulerTest, WaitEndsSilentlyWhenSwitchedOff) {
  Make(File());
  Facts facts = Good(At(kMonday, "10:00"));
  facts.battery = 60;
  Tick(facts);
  ASSERT_FALSE(scheduler_->SetSchedule({{"enabled", 0}}, At(kMonday, "10:05")));
  facts.wall = At(kMonday, "10:06");
  facts.battery = 100;
  const Output out = Tick(facts);
  EXPECT_FALSE(out.start);
  EXPECT_TRUE(out.state_changed);
  EXPECT_TRUE(scheduler_->StateJson()["waiting"].is_null());
  EXPECT_EQ(Words(), (Lines{"waiting_battery 60"}));
}

TEST_F(SchedulerTest, T14_ForecastAsksTheWeather) {
  Make(File({{"skip_forecast", 1}, {"forecast_hours", 2}}));
  Output out = TickAt(kMonday, "10:00");
  ASSERT_TRUE(out.weather);
  EXPECT_FALSE(out.start);
  EXPECT_EQ(out.weather->hours, 2);
  EXPECT_DOUBLE_EQ(out.weather->latitude, 52.52);
  Weather weather;
  weather.id = out.weather->id;
  weather.ok = true;
  weather.has_rain = true;
  weather.rain_hour_mm = {0.1, 0.2, 5.0};
  scheduler_->OnWeather(weather);
  EXPECT_FALSE(TickAt(kMonday, "10:00", 1).start);
  EXPECT_EQ(Words(), (Lines{"skip_forecast"}));
}

TEST_F(SchedulerTest, T14_RainBeyondTheHoursAskedStarts) {
  Make(File({{"skip_forecast", 1}}));
  const Output out = TickAt(kMonday, "10:00");
  Weather weather;
  weather.id = out.weather->id;
  weather.ok = true;
  weather.has_rain = true;
  weather.rain_now_mm = 0.04;
  weather.rain_hour_mm = {0.19, 3.0};
  scheduler_->OnWeather(weather);
  EXPECT_TRUE(TickAt(kMonday, "10:00", 1).start);
  EXPECT_TRUE(Words().empty());
}

TEST_F(SchedulerTest, T14_RainNow) {
  Make(File({{"skip_forecast", 1}}));
  const Output out = TickAt(kMonday, "10:00");
  Weather weather;
  weather.id = out.weather->id;
  weather.ok = true;
  weather.has_rain = true;
  weather.rain_now_mm = 0.05;
  scheduler_->OnWeather(weather);
  EXPECT_FALSE(TickAt(kMonday, "10:00", 1).start);
  EXPECT_EQ(Words(), (Lines{"skip_forecast"}));
}

TEST_F(SchedulerTest, T14_Cold) {
  Make(File({{"skip_cold", 1}}));
  const Output out = TickAt(kMonday, "10:00");
  Weather weather;
  weather.id = out.weather->id;
  weather.ok = true;
  weather.has_temp = true;
  weather.temp_c = 4.24;
  scheduler_->OnWeather(weather);
  EXPECT_FALSE(TickAt(kMonday, "10:00", 1).start);
  EXPECT_EQ(Words(), (Lines{"skip_cold 4.2"}));
}

TEST_F(SchedulerTest, T14_NoAnswerStartsAll) {
  Make(File({{"skip_forecast", 1}, {"skip_cold", 1}}));
  ASSERT_TRUE(TickAt(kMonday, "10:00").weather);
  EXPECT_FALSE(TickAt(kMonday, "10:00", 14).start);
  EXPECT_TRUE(TickAt(kMonday, "10:00", 15).start);
  EXPECT_EQ(Words(), (Lines{"weather_unknown forecast", "weather_unknown cold"}));
}

TEST_F(SchedulerTest, T14_OldOrUnusableAnswerCountsAsNone) {
  Make(File({{"skip_forecast", 1}, {"skip_cold", 1}}));
  const Output out = TickAt(kMonday, "10:00");
  Weather weather;
  weather.id = out.weather->id;
  weather.ok = true;
  weather.age_s = kWeatherMaxAge + 1;
  weather.has_rain = true;
  weather.rain_now_mm = 9;
  weather.has_temp = true;
  scheduler_->OnWeather(weather);
  EXPECT_TRUE(TickAt(kMonday, "10:00", 1).start);
  EXPECT_EQ(Words(), (Lines{"weather_unknown forecast", "weather_unknown cold"}));
  // An answer to another request is left alone.
  Make(File({{"skip_cold", 1}}));
  const Output again = TickAt(kMonday, "10:00");
  weather.id = again.weather->id + 1;
  weather.age_s = 0;
  weather.temp_c = 0;
  scheduler_->OnWeather(weather);
  EXPECT_FALSE(TickAt(kMonday, "10:00", 1).start);
}

TEST_F(SchedulerTest, T14_NoPositionForTheWeather) {
  Make(File({{"skip_cold", 1}, {"plans", {{{"days", {1}}, {"time", "10:00"}, {"dark", 1}}}}}));
  Facts facts = Good(At(kMonday, "10:00"));
  facts.latitude.reset();
  const Output out = Tick(facts);
  EXPECT_FALSE(out.weather);
  EXPECT_TRUE(out.start);
  EXPECT_EQ(Words(), (Lines{"weather_unknown cold"}));
}

TEST_F(SchedulerTest, T15_RowsAgainAfterTheWeather) {
  Make(File({{"skip_forecast", 1}}));
  const Output out = TickAt(kMonday, "10:00");
  Weather weather;
  weather.id = out.weather->id;
  weather.ok = true;
  weather.has_rain = true;
  scheduler_->OnWeather(weather);
  Facts facts = Good(At(kMonday, "10:00", 2));
  facts.state = "RECORDING";
  EXPECT_FALSE(Tick(facts).start);
  EXPECT_EQ(Words(), (Lines{"skip_busy RECORDING"}));
}

TEST_F(SchedulerTest, T16_RefusedByTheLogic) {
  Make(File());
  const Output out = TickAt(kMonday, "10:00");
  scheduler_->OnStartAnswer(out.start->run_id, false, "BATTERY_HOT", 61.5);
  EXPECT_EQ(Words(), (Lines{"start_refused BATTERY_HOT"}));
  EXPECT_EQ(scheduler_->LogEntries(1)[0]["more"], 61.5);
  // After the announcement.
  Make(File());
  const RunRequest run = StartAndAccept();
  Status(run.run_id, Phase::kAnnouncing);
  Status(run.run_id, Phase::kEnded, "REFUSED", "NOT_DOCKED");
  EXPECT_EQ(Words(), (Lines{"start_refused NOT_DOCKED"}));
}

TEST_F(SchedulerTest, T17_AnnouncementCanceled) {
  Make(File());
  const RunRequest run = StartAndAccept();
  Status(run.run_id, Phase::kAnnouncing);
  EXPECT_EQ(scheduler_->NextStart(), At(kMonday, "10:00"));
  Status(run.run_id, Phase::kEnded, "CANCELED", "button");
  EXPECT_EQ(Words(), (Lines{"start_canceled button"}));
}

TEST_F(SchedulerTest, T18_StartedWithTheBattery) {
  Make(File());
  const RunRequest run = StartAndAccept();
  Facts facts = Good(At(kMonday, "10:01"));
  facts.battery = 97;
  Tick(facts);
  Status(run.run_id, Phase::kUndocking);
  Status(run.run_id, Phase::kMowing);
  Status(run.run_id, Phase::kMowing);
  EXPECT_EQ(Words(), (Lines{"started 97"}));
  // Mowing for hours is no silence.
  EXPECT_FALSE(TickAt(kMonday, "13:00").start);
  EXPECT_EQ(Words(), (Lines{"started 97"}));
}

TEST_F(SchedulerTest, T19_EndedBeforeMowing) {
  Make(File());
  RunRequest run = StartAndAccept();
  Status(run.run_id, Phase::kUndocking);
  Status(run.run_id, Phase::kEnded, "UNDOCKING_FAILED");
  EXPECT_EQ(Words(), (Lines{"start_failed UNDOCKING_FAILED"}));
  Make(File());
  run = StartAndAccept();
  TickAt(kMonday, "10:01", 59);
  EXPECT_TRUE(Words().empty());
  TickAt(kMonday, "10:02", 1);
  EXPECT_EQ(Words(), (Lines{"start_failed timeout"}));
  // A word of the logic keeps it.
  Make(File());
  run = StartAndAccept();
  TickAt(kMonday, "10:01", 50);
  Status(run.run_id, Phase::kChecking);
  TickAt(kMonday, "10:03", 40);
  EXPECT_TRUE(Words().empty());
}

TEST_F(SchedulerTest, T20_WhyARunWentHome) {
  const std::vector<std::pair<std::pair<std::string, std::string>, std::string>> cases = {
      {{"SCHEDULE", "end"}, "stopped_end"},   {{"SCHEDULE", "pause"}, "stopped_paused"},
      {{"DARK", ""}, "stopped_dark"},         {{"RAIN", ""}, "stopped_rain"},
      {{"JOB_COMPLETE", ""}, ""},             {{"BATTERY_LOW", ""}, ""},
      {{"MANUAL", "pause"}, ""}};
  for (const auto& item : cases) {
    Make(File());
    const RunRequest run = StartAndAccept();
    Status(run.run_id, Phase::kMowing);
    Status(run.run_id, Phase::kEnded, item.first.first, item.first.second);
    Lines expected{"started 100"};
    if (!item.second.empty()) expected.push_back(item.second);
    EXPECT_EQ(Words(), expected) << item.first.first;
    EXPECT_EQ(scheduler_->LogEntries(1)[0]["at"], "10:00");
  }
}

TEST_F(SchedulerTest, OtherRunsAreLeftAlone) {
  Make(File());
  const RunRequest run = StartAndAccept();
  Status("2026-06-15T09:00", Phase::kMowing);
  scheduler_->OnStartAnswer("2026-06-15T09:00", false, "X", nullptr);
  scheduler_->StartFailed("other", "storage");
  EXPECT_TRUE(Words().empty());
  Status(run.run_id, Phase::kMowing);
  EXPECT_EQ(Words(), (Lines{"started 100"}));
}

TEST_F(SchedulerTest, OrderOfTheRows) {
  Make(File({{"pause", {{"until", "2026-06-15"}}}}));
  Facts facts = Good(At(kMonday, "10:00"));
  facts.clock_ok = false;
  facts.clock_reason = "clock_jump";
  Tick(facts);
  EXPECT_EQ(Words(), (Lines{"skip_clock clock_jump"}));
  // Emergency before busy, busy before not docked, not docked before rain, rain before dark, dark before battery.
  facts = Good(At(kMonday, "10:00"));
  facts.emergency = true;
  facts.state = "EMERGENCY";
  facts.docked = false;
  facts.rain_wet = true;
  facts.battery = 1;
  facts.hold = true;
  const std::vector<std::string> expected = {"skip_emergency", "skip_busy EMERGENCY", "skip_not_docked",
                                             "skip_rain", "waiting_battery 1"};
  for (const std::string& word : expected) {
    Make(File());
    Tick(facts);
    EXPECT_EQ(Words(), Lines{word});
    if (word == "skip_emergency") facts.emergency = false;
    if (word == "skip_busy EMERGENCY") facts.state = "IDLE";
    if (word == "skip_not_docked") facts.docked = true;
    if (word == "skip_rain") facts.rain_wet = false;
  }
  // Paused before no area, no area before offline.
  facts = Good(At(kMonday, "10:00"));
  facts.logic_fresh = false;
  facts.areas = {{"a", false}};
  Make(File());
  Tick(facts);
  EXPECT_EQ(Words(), Lines{"skip_no_area"});
}

TEST_F(SchedulerTest, NoStartInTheMinuteThePlanWasSaved) {
  Make(File());
  ASSERT_FALSE(scheduler_->SetSchedule({{"plans", {{{"days", {1}}, {"time", "10:00"}, {"fresh", 1}}}}},
                                       At(kMonday, "10:00", 20)));
  EXPECT_FALSE(TickAt(kMonday, "10:00", 21).start);
  TickAt(kMonday, "12:00");
  EXPECT_TRUE(Words().empty());
  // Other keys do not move it.
  Make(File());
  ASSERT_FALSE(scheduler_->SetSchedule({{"min_battery", 50}}, At(kMonday, "10:00", 20)));
  EXPECT_TRUE(TickAt(kMonday, "10:00", 21).start);
}

TEST_F(SchedulerTest, SummerTimeBegins) {
  const Date sunday{2026, 3, 29};
  ScheduleFile file = File({{"plans", {{{"days", {7}}, {"time", "02:30"}, {"dark", 1}}}}});
  file.changed_at = 0;
  Make(file);
  EXPECT_FALSE(TickAt(sunday, "01:59", 59).start);
  EXPECT_TRUE(Tick(Good(At(sunday, "03:00"))).start);
  EXPECT_EQ(At(sunday, "02:30"), At(sunday, "03:00"));
}

TEST_F(SchedulerTest, SummerTimeEndsFiresOnce) {
  const Date sunday{2026, 10, 25};
  Make(File({{"plans", {{{"days", {7}}, {"time", "02:30"}, {"dark", 1}}}}}));
  const int64_t first = At(sunday, "02:30");
  const Output out = Tick(Good(first));
  ASSERT_TRUE(out.start);
  scheduler_->StartFailed(out.start->run_id, "storage");
  EXPECT_FALSE(Tick(Good(first + 3600)).start);
  EXPECT_FALSE(Tick(Good(first + 3600 + 30)).start);
  EXPECT_EQ(Words(), (Lines{"start_failed storage"}));
}

TEST_F(SchedulerTest, DaysOfThePlan) {
  Make(File({{"plans", {{{"days", {2, 7}}, {"time", "10:00"}}}}}));
  EXPECT_FALSE(TickAt(kMonday, "10:00").start);
  EXPECT_TRUE(TickAt(Date{2026, 6, 16}, "10:00").start);
}

TEST_F(SchedulerTest, NextStart) {
  Make(File({{"plans", {{{"days", {1}}, {"time", "10:00"}}, {{"days", {3}}, {"time", "08:00"}}}}}));
  EXPECT_FALSE(scheduler_->NextStart());
  TickAt(kMonday, "09:00");
  EXPECT_EQ(scheduler_->NextStart(), At(kMonday, "10:00"));
  TickAt(kMonday, "11:00");
  EXPECT_EQ(scheduler_->NextStart(), At(Date{2026, 6, 17}, "08:00"));
  // A pause over all areas leaves its days out.
  ASSERT_FALSE(scheduler_->SetSchedule({{"pause", {{"until", "2026-06-17"}}}}, At(kMonday, "11:00")));
  EXPECT_EQ(scheduler_->NextStart(), At(Date{2026, 6, 22}, "10:00"));
  ASSERT_FALSE(scheduler_->SetSchedule({{"pause", {{"until", "2026-06-17"}, {"areas", {"a"}}}}},
                                       At(kMonday, "11:00")));
  EXPECT_EQ(scheduler_->NextStart(), At(Date{2026, 6, 17}, "08:00"));
  Facts facts = Good(At(kMonday, "11:00", 1));
  facts.clock_ok = false;
  Tick(facts);
  EXPECT_FALSE(scheduler_->NextStart());
  ASSERT_FALSE(scheduler_->SetSchedule({{"plans", json::array()}}, At(kMonday, "11:00")));
  TickAt(kMonday, "11:01");
  EXPECT_FALSE(scheduler_->NextStart());
}

TEST_F(SchedulerTest, PauseDuringARunLeavesItsAreasOut) {
  Make(File({{"plans", {{{"days", {1}}, {"time", "10:00"}, {"areas", {"a", "b"}}}}}}));
  const RunRequest run = StartAndAccept();
  Status(run.run_id, Phase::kMowing);
  scheduler_->TakeOutput();
  ASSERT_FALSE(scheduler_->SetSchedule({{"pause", {{"until", "2026-06-15"}, {"areas", {"b", "c"}}}}},
                                       At(kMonday, "10:30")));
  Output out = scheduler_->TakeOutput();
  ASSERT_TRUE(out.drop);
  EXPECT_EQ(out.drop->run_id, run.run_id);
  EXPECT_EQ(out.drop->area_ids, (std::vector<std::string>{"b"}));
  EXPECT_TRUE(out.file_changed);
  ASSERT_FALSE(scheduler_->SetSchedule({{"pause", {{"until", "2026-06-15"}}}}, At(kMonday, "10:31")));
  out = scheduler_->TakeOutput();
  ASSERT_TRUE(out.drop);
  EXPECT_TRUE(out.drop->area_ids.empty());
  // A pause from tomorrow on, or none, leaves the run alone.
  ASSERT_FALSE(scheduler_->SetSchedule({{"pause", nullptr}}, At(kMonday, "10:32")));
  EXPECT_FALSE(scheduler_->TakeOutput().drop);
}

TEST_F(SchedulerTest, PauseWhileTheStartIsAskedComesWithTheAnswer) {
  Make(File());
  const Output out = TickAt(kMonday, "10:00");
  ASSERT_FALSE(scheduler_->SetSchedule({{"pause", {{"until", "2026-06-15"}, {"areas", {"a"}}}}},
                                       At(kMonday, "10:00", 1)));
  EXPECT_FALSE(scheduler_->TakeOutput().drop);
  scheduler_->OnStartAnswer(out.start->run_id, true, "", nullptr);
  const Output answer = scheduler_->TakeOutput();
  ASSERT_TRUE(answer.drop);
  EXPECT_EQ(answer.drop->area_ids, (std::vector<std::string>{"a"}));
}

TEST_F(SchedulerTest, StateAndLogSurviveARestart) {
  Make(File());
  const Output out = TickAt(kMonday, "10:00");
  scheduler_->StartFailed(out.start->run_id, "storage");
  const json state = scheduler_->StateJson();
  const json log = scheduler_->LogJson();
  scheduler_.emplace(File(), true, state, log, At(kMonday, "10:00", 20));
  EXPECT_FALSE(TickAt(kMonday, "10:00", 30).start);
  EXPECT_EQ(Words(), (Lines{"start_failed storage"}));
  // Marks of days before yesterday fall out; yesterday's plan was missed.
  TickAt(Date{2026, 6, 17}, "09:00");
  EXPECT_EQ(scheduler_->StateJson()["handled"], (json{{"2026-06-16", {"10:00"}}}));
}

TEST_F(SchedulerTest, LogKeepsTheLastHundred) {
  json entries = json::array();
  for (int i = 0; i < 120; ++i) entries.push_back({{"t", i}, {"what", "skip_rain"}, {"detail", nullptr}});
  entries.push_back({{"what", "no time"}});
  scheduler_.emplace(File(), true, nullptr, json{{"schema", 1}, {"entries", entries}}, 0);
  const json kept = scheduler_->LogEntries(kLogEntries);
  ASSERT_EQ(kept.size(), kLogEntries);
  EXPECT_EQ(kept[0]["t"], 20);
  EXPECT_EQ(scheduler_->LogEntries(2).size(), 2u);
  EXPECT_EQ(scheduler_->LogEntries(2)[1]["t"], 119);
}

TEST_F(SchedulerTest, EveryDecisionIsStoredOnce) {
  Make(File());
  Facts facts = Good(At(kMonday, "10:00"));
  facts.emergency = true;
  Output out = Tick(facts);
  EXPECT_TRUE(out.state_changed);
  EXPECT_TRUE(out.log_changed);
  facts.wall += 1;
  out = Tick(facts);
  EXPECT_FALSE(out.state_changed);
  EXPECT_FALSE(out.log_changed);
  // A new day prunes the marks of the day before yesterday.
  TickAt(Date{2026, 6, 17}, "09:00");
  EXPECT_TRUE(TickAt(Date{2026, 6, 17}, "09:00", 1).state_changed == false);
  Make(File(), true, json{{"schema", 1}, {"handled", {{"2026-06-10", {"10:00"}}}}, {"waiting", nullptr}});
  EXPECT_TRUE(TickAt(kMonday, "09:00").state_changed);
  EXPECT_EQ(scheduler_->StateJson()["handled"], json::object());
}

TEST_F(SchedulerTest, WindowEdges) {
  Make(File());
  // The first second after the window: missed, not started.
  EXPECT_FALSE(TickAt(kMonday, "10:01").start);
  EXPECT_EQ(Words(), (Lines{"skip_missed"}));
  // A plan saved in the very second of its time does not fire, and is not missed either.
  Make(File());
  ASSERT_FALSE(scheduler_->SetSchedule({{"plans", {{{"days", {1}}, {"time", "10:00"}}}}}, At(kMonday, "10:00")));
  EXPECT_FALSE(TickAt(kMonday, "10:00").start);
  TickAt(kMonday, "10:01");
  EXPECT_TRUE(Words().empty());
  EXPECT_EQ(scheduler_->NextStart(), At(Date{2026, 6, 22}, "10:00"));
}

TEST_F(SchedulerTest, NextStartEdges) {
  Make(File());
  TickAt(kMonday, "10:00", 59);
  ASSERT_FALSE(scheduler_->SetSchedule({{"enabled", 0}}, At(kMonday, "09:00")));
  EXPECT_FALSE(scheduler_->NextStart());
  Make(File());
  Facts facts = Good(At(kMonday, "10:00"));
  facts.clock_ok = false;
  Tick(facts);
  TickAt(kMonday, "10:00", 20);
  EXPECT_EQ(scheduler_->NextStart(), At(Date{2026, 6, 16}, "10:00"));
  // A plan whose window has just passed is no longer next.
  Make(File({{"plans", {{{"days", {1}}, {"time", "10:00"}}, {{"days", {1}}, {"time", "11:00"}}}}}));
  facts = Good(At(kMonday, "10:00"));
  facts.logic_fresh = false;
  Tick(facts);
  EXPECT_EQ(scheduler_->NextStart(), At(kMonday, "10:00"));
  facts.wall = At(kMonday, "10:01");
  Tick(facts);
  EXPECT_EQ(scheduler_->NextStart(), At(kMonday, "11:00"));
}

TEST_F(SchedulerTest, ClockSkipsOnTwoDaysAreTwo) {
  Make(File());
  Facts facts = Good(At(kMonday, "10:00"));
  facts.clock_ok = false;
  facts.clock_reason = "gnss_mismatch";
  Tick(facts);
  facts.wall = At(Date{2026, 6, 16}, "10:00");
  Tick(facts);
  EXPECT_EQ(Words(), (Lines{"skip_clock gnss_mismatch", "skip_clock gnss_mismatch"}));
}

TEST_F(SchedulerTest, ClockSkipBeforeMidnightIsNotMissedAfterIt) {
  Make(File({{"plans", {{{"days", {1}}, {"time", "23:59"}, {"dark", 1}}}}}));
  Facts facts = Good(At(kMonday, "23:59"));
  facts.clock_ok = false;
  facts.clock_reason = "never_synced";
  Tick(facts);
  TickAt(Date{2026, 6, 16}, "00:05");
  EXPECT_EQ(Words(), (Lines{"skip_clock never_synced"}));
  EXPECT_EQ(scheduler_->StateJson()["handled"]["2026-06-15"], json({"23:59"}));
}

TEST_F(SchedulerTest, WaitEndsOnAnUnsafeClockOrSilentLogic) {
  Make(File());
  Facts facts = Good(At(kMonday, "10:00"));
  facts.battery = 60;
  Tick(facts);
  facts.wall = At(kMonday, "10:10");
  facts.clock_ok = false;
  facts.clock_reason = "clock_jump";
  EXPECT_TRUE(Tick(facts).state_changed);
  EXPECT_EQ(Words(), (Lines{"waiting_battery 60", "skip_clock clock_jump"}));
  EXPECT_TRUE(scheduler_->StateJson()["waiting"].is_null());
  Make(File());
  facts = Good(At(kMonday, "10:00"));
  facts.battery = 60;
  Tick(facts);
  facts.wall = At(kMonday, "10:10");
  facts.logic_fresh = false;
  Tick(facts);
  EXPECT_EQ(Words(), (Lines{"waiting_battery 60", "skip_offline"}));
}

TEST_F(SchedulerTest, WaitEndsOnASkipOfItsOwn) {
  Make(File());
  Facts facts = Good(At(kMonday, "10:00"));
  facts.battery = 60;
  Tick(facts);
  facts.wall = At(kMonday, "10:10");
  facts.emergency = true;
  EXPECT_TRUE(Tick(facts).state_changed);
  EXPECT_TRUE(scheduler_->StateJson()["waiting"].is_null());
  facts.wall += 1;
  facts.emergency = false;
  facts.battery = 100;
  EXPECT_FALSE(Tick(facts).start);
  EXPECT_EQ(Words(), (Lines{"waiting_battery 60", "skip_emergency"}));
}

TEST_F(SchedulerTest, WaitEndsForTheNextPlanEvenIfThatOneIsSkipped) {
  Make(File({{"plans", {{{"days", {1}}, {"time", "10:00"}}, {{"days", {1}}, {"time", "10:30"}}}}}));
  Facts facts = Good(At(kMonday, "10:00"));
  facts.battery = 60;
  Tick(facts);
  facts.wall = At(kMonday, "10:30");
  facts.emergency = true;
  Tick(facts);
  facts.wall += 1;
  facts.emergency = false;
  facts.battery = 100;
  EXPECT_FALSE(Tick(facts).start);
  EXPECT_EQ(Words(), (Lines{"waiting_battery 60", "skip_battery 60", "skip_emergency"}));
  EXPECT_TRUE(scheduler_->StateJson()["waiting"].is_null());
}

TEST_F(SchedulerTest, WaitFromBatteryToHoldIsStored) {
  Make(File());
  Facts facts = Good(At(kMonday, "10:00"));
  facts.battery = 60;
  Tick(facts);
  facts.wall += 600;
  facts.battery = 90;
  facts.hold = true;
  EXPECT_TRUE(Tick(facts).state_changed);
}

TEST_F(SchedulerTest, WaitOverWhenTheWeatherIsAsked) {
  Make(File({{"skip_forecast", 1}}));
  Facts facts = Good(At(kMonday, "10:00"));
  facts.battery = 60;
  Tick(facts);
  facts.wall += 600;
  facts.battery = 90;
  const Output out = Tick(facts);
  EXPECT_TRUE(out.weather);
  EXPECT_TRUE(out.state_changed);
  EXPECT_TRUE(scheduler_->StateJson()["waiting"].is_null());
}

TEST_F(SchedulerTest, ADuePlanWaitsWhileTheWeatherIsAsked) {
  Make(File({{"skip_forecast", 1}, {"plans", {{{"days", {1}}, {"time", "10:00"}}, {{"days", {1}}, {"time", "10:30"}}}}}));
  Facts facts = Good(At(kMonday, "10:00"));
  facts.battery = 60;
  Tick(facts);
  facts.wall = At(kMonday, "10:29", 55);
  facts.battery = 90;
  const Output asked = Tick(facts);
  ASSERT_TRUE(asked.weather);
  facts.wall = At(kMonday, "10:30");
  EXPECT_FALSE(Tick(facts).weather);
  Weather weather;
  weather.id = asked.weather->id;
  weather.ok = true;
  weather.has_rain = true;
  scheduler_->OnWeather(weather);
  facts.wall += 1;
  const Output started = Tick(facts);
  ASSERT_TRUE(started.start);
  EXPECT_EQ(started.start->run_id, "2026-06-15T10:00");
  EXPECT_EQ(Words(), (Lines{"waiting_battery 60", "skip_busy NONE"}));
}

TEST_F(SchedulerTest, WeatherEdges) {
  Make(File({{"skip_cold", 1}}));
  Output out = TickAt(kMonday, "10:00");
  Weather weather;
  weather.id = out.weather->id;
  weather.ok = true;
  weather.age_s = kWeatherMaxAge;
  weather.has_temp = true;
  weather.temp_c = 5;
  scheduler_->OnWeather(weather);
  EXPECT_TRUE(TickAt(kMonday, "10:00", 1).start);
  EXPECT_TRUE(Words().empty());
}

TEST_F(SchedulerTest, NoPositionForTheWeatherBothUnknown) {
  Make(File({{"skip_forecast", 1}, {"skip_cold", 1}, {"plans", {{{"days", {1}}, {"time", "10:00"}, {"dark", 1}}}}}));
  Facts facts = Good(At(kMonday, "10:00"));
  facts.longitude.reset();
  EXPECT_TRUE(Tick(facts).start);
  EXPECT_EQ(Words(), (Lines{"weather_unknown forecast", "weather_unknown cold"}));
}

TEST_F(SchedulerTest, AfterTheWeather) {
  // The plan is gone.
  Make(File({{"skip_cold", 1}}));
  Output out = TickAt(kMonday, "10:00");
  ASSERT_FALSE(scheduler_->SetSchedule({{"plans", json::array()}}, At(kMonday, "10:00", 5)));
  EXPECT_FALSE(TickAt(kMonday, "10:00", 15).start);
  EXPECT_EQ(Words(), (Lines{"skip_gone"}));
  EXPECT_FALSE(TickAt(kMonday, "10:00", 16).start);
  EXPECT_EQ(Words(), (Lines{"skip_gone"}));
  // The clock became unsafe.
  Make(File({{"skip_cold", 1}}));
  TickAt(kMonday, "10:00");
  Facts facts = Good(At(kMonday, "10:00", 15));
  facts.clock_ok = false;
  facts.clock_reason = "clock_jump";
  EXPECT_FALSE(Tick(facts).start);
  EXPECT_EQ(Words(), (Lines{"weather_unknown cold", "skip_clock clock_jump"}));
  // The logic went silent.
  Make(File({{"skip_cold", 1}}));
  TickAt(kMonday, "10:00");
  facts = Good(At(kMonday, "10:00", 15));
  facts.logic_fresh = false;
  EXPECT_FALSE(Tick(facts).start);
  EXPECT_EQ(Words(), (Lines{"weather_unknown cold", "skip_offline"}));
}

TEST_F(SchedulerTest, SwitchedOffWhileTheWeatherIsAsked) {
  Make(File({{"skip_cold", 1}}));
  const Output out = TickAt(kMonday, "10:00");
  ASSERT_FALSE(scheduler_->SetSchedule({{"enabled", 0}}, At(kMonday, "10:00", 2)));
  TickAt(kMonday, "10:00", 3);
  ASSERT_FALSE(scheduler_->SetSchedule({{"enabled", 1}}, At(kMonday, "10:00", 4)));
  Weather weather;
  weather.id = out.weather->id;
  weather.ok = true;
  weather.has_temp = true;
  weather.temp_c = 20;
  scheduler_->OnWeather(weather);
  EXPECT_FALSE(TickAt(kMonday, "10:00", 5).start);
  EXPECT_TRUE(Words().empty());
}

TEST_F(SchedulerTest, TheRequestCarriesThePlan) {
  Make(File({{"plans", {{{"days", {1}}, {"time", "10:00"}, {"fresh", 1}, {"angle_offset_deg", 30}}}}}));
  const Output out = TickAt(kMonday, "10:00");
  ASSERT_TRUE(out.start);
  EXPECT_TRUE(out.start->restart);
  EXPECT_EQ(out.start->angle_offset_deg, 30.0);
}

TEST_F(SchedulerTest, AnswersInAnOddOrder) {
  // The silence counts from the answer.
  Make(File());
  Output out = TickAt(kMonday, "10:00");
  TickAt(kMonday, "10:01", 50);
  scheduler_->OnStartAnswer(out.start->run_id, true, "", nullptr);
  TickAt(kMonday, "10:03");
  EXPECT_TRUE(Words().empty());
  // A status means taken: a refusal after it is left alone.
  Make(File());
  out = TickAt(kMonday, "10:00");
  Status(out.start->run_id, Phase::kAnnouncing);
  scheduler_->OnStartAnswer(out.start->run_id, false, "BUSY", nullptr);
  EXPECT_TRUE(Words().empty());
  // A refusal ends the run: the next plan is not busy.
  Make(File({{"plans", {{{"days", {1}}, {"time", "10:00"}}, {{"days", {1}}, {"time", "10:01"}}}}}));
  out = TickAt(kMonday, "10:00");
  scheduler_->OnStartAnswer(out.start->run_id, false, "BUSY", nullptr);
  scheduler_->OnStartAnswer(out.start->run_id, true, "", nullptr);
  EXPECT_TRUE(TickAt(kMonday, "10:01").start);
  // So does the end of a run, and a silence.
  Make(File({{"plans", {{{"days", {1}}, {"time", "10:00"}}, {{"days", {1}}, {"time", "10:03"}}}}}));
  out = TickAt(kMonday, "10:00");
  TickAt(kMonday, "10:02", 1);
  EXPECT_TRUE(TickAt(kMonday, "10:03").start);
  Make(File({{"plans", {{{"days", {1}}, {"time", "10:00"}}, {{"days", {1}}, {"time", "10:01"}}}}}));
  out = TickAt(kMonday, "10:00");
  Status(out.start->run_id, Phase::kEnded, "DARK");
  EXPECT_EQ(Words(), (Lines{"start_failed DARK"}));
  EXPECT_TRUE(TickAt(kMonday, "10:01").start);
}

TEST_F(SchedulerTest, ExactlyTheSilenceIsNoFailure) {
  Make(File());
  StartAndAccept();
  TickAt(kMonday, "10:02");
  EXPECT_TRUE(Words().empty());
}

TEST_F(SchedulerTest, ChangesMarkTheFile) {
  Make(File());
  EXPECT_TRUE(scheduler_->SetSchedule({{"min_temp", 4}}, 0));
  EXPECT_FALSE(scheduler_->TakeOutput().file_changed);
  EXPECT_TRUE(scheduler_->SetLeftovers({{"enabled", 1}}));
  EXPECT_FALSE(scheduler_->TakeOutput().file_changed);
  EXPECT_FALSE(scheduler_->SetLeftovers({{"min_m2", 3}}));
  EXPECT_TRUE(scheduler_->TakeOutput().file_changed);
  EXPECT_EQ(scheduler_->file().leftovers.min_m2, 3.0);
  // A refused change leaves a run alone.
  StartAndAccept();
  scheduler_->TakeOutput();
  EXPECT_TRUE(scheduler_->SetSchedule({{"pause", {{"until", "2026-06-15"}}}, {"min_temp", 4}}, At(kMonday, "10:01")));
  EXPECT_FALSE(scheduler_->TakeOutput().drop);
}

TEST_F(SchedulerTest, LogStaysAtAHundred) {
  json entries = json::array();
  for (int i = 0; i < 100; ++i) entries.push_back({{"t", i}, {"what", "skip_rain"}});
  scheduler_.emplace(File(), true, nullptr, json{{"schema", 1}, {"entries", entries}}, 0);
  Facts facts = Good(At(kMonday, "10:00"));
  facts.emergency = true;
  Tick(facts);
  const json kept = scheduler_->LogEntries(kLogEntries);
  ASSERT_EQ(kept.size(), kLogEntries);
  EXPECT_EQ(kept[0]["t"], 1);
  EXPECT_EQ(kept[99]["what"], "skip_emergency");
}

TEST_F(SchedulerTest, StoredTextsOnlyAsWritten) {
  const json entry = {{"t", 5}, {"what", "skip_rain"}};
  const auto log_size = [&](const json& log) {
    scheduler_.emplace(File(), true, nullptr, log, 0);
    return scheduler_->LogEntries(kLogEntries).size();
  };
  EXPECT_EQ(log_size(json{{"schema", 1}, {"entries", {entry}}}), 1u);
  EXPECT_EQ(log_size(json{{"entries", {entry}}}), 0u);
  EXPECT_EQ(log_size(json{{"schema", 1}}), 0u);
  EXPECT_EQ(log_size(json{{"schema", 1}, {"entries", entry}}), 0u);
  EXPECT_EQ(log_size(json::array({entry})), 0u);
  EXPECT_EQ(log_size(json{{"schema", 1}, {"entries", {{{"t", 5}}, {{"what", "x"}}, 3, {{"t", "5"}, {"what", "x"}}}}}),
            0u);
  const auto handled = [&](const json& state) {
    scheduler_.emplace(File(), true, state, nullptr, 0);
    return scheduler_->StateJson()["handled"];
  };
  EXPECT_EQ(handled(json{{"schema", 1}, {"handled", {{"2026-06-15", {"10:00", "25:00", 3}}}}}),
            (json{{"2026-06-15", {"10:00"}}}));
  EXPECT_EQ(handled(json{{"handled", {{"2026-06-15", {"10:00"}}}}}), json::object());
  EXPECT_EQ(handled(json{{"schema", 1}, {"handled", json::array()}}), json::object());
  EXPECT_EQ(handled(json{{"schema", 1}, {"handled", {{"2026-02-30", {"10:00"}}, {"2026-06-16", "10:00"}}}}),
            json::object());
  const auto missed = [&](const json& waiting) {
    scheduler_.emplace(File(), true, json{{"schema", 1}, {"waiting", waiting}}, nullptr, 0);
    return Words().size();
  };
  EXPECT_EQ(missed(json{{"date", "2026-06-15"}, {"time", "10:00"}, {"kind", "hold"}}), 1u);
  EXPECT_EQ(missed(json{{"date", "2026-06-15"}}), 0u);
  EXPECT_EQ(missed(json{{"time", 600}}), 0u);
  EXPECT_EQ(missed("10:00"), 0u);
}

}  // namespace
}  // namespace mowbite_schedule
