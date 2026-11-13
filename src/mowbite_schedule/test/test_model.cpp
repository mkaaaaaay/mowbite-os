#include <gtest/gtest.h>

#include <string>

#include "mowbite_schedule/model.h"

namespace mowbite_schedule {
namespace {

json OnePlan(json plan) {
  return {{"plans", json::array({std::move(plan)})}};
}

// The field and reason of a refused change, or "taken".
std::string Refusal(const json& change, bool available = true) {
  ScheduleFile file;
  const std::optional<Invalid> bad = ApplySchedule(change, available, 100, &file);
  return bad ? bad->field + " " + bad->reason : "taken";
}

TEST(ScheduleModel, DefaultsAsTheApp) {
  const json out = ScheduleToJson(Schedule());
  EXPECT_EQ(out["enabled"], 0);
  EXPECT_EQ(out["min_battery"], 80);
  EXPECT_EQ(out["skip_rain"], 1);
  EXPECT_EQ(out["skip_forecast"], 0);
  EXPECT_EQ(out["forecast_hours"], 1);
  EXPECT_EQ(out["skip_cold"], 0);
  EXPECT_EQ(out["min_temp"], 5);
  EXPECT_EQ(out["tz"], "UTC0");
  EXPECT_EQ(out["plans"], json::array());
  EXPECT_TRUE(out["pause"].is_null());
  EXPECT_EQ(LeftoversToJson(Leftovers()), (json{{"enabled", 0}, {"min_m2", 0.5}}));
}

TEST(ScheduleModel, EveryKeyTaken) {
  ScheduleFile file;
  const json change = {{"enabled", 1},
                       {"min_battery", 60},
                       {"skip_rain", 0},
                       {"skip_forecast", 1},
                       {"forecast_hours", 3},
                       {"skip_cold", 1},
                       {"min_temp", 10},
                       {"tz", "CET-1CEST,M3.5.0,M10.5.0/3"},
                       {"plans", json::array({{{"days", {1, 3, 5}},
                                               {"time", "09:30"},
                                               {"areas", {"a", "b"}},
                                               {"end", "12:00"},
                                               {"dark", 1},
                                               {"sunset", 1},
                                               {"fresh", 1},
                                               {"angle_offset_deg", -45.5}}})},
                       {"pause", {{"until", "2026-11-20"}, {"areas", {"b"}}}}};
  ASSERT_FALSE(ApplySchedule(change, true, 100, &file));
  EXPECT_EQ(ScheduleToJson(file.schedule), change);
  EXPECT_EQ(file.version, 1);
  EXPECT_EQ(file.changed_at, 100);
}

TEST(ScheduleModel, PlanDefaultsAndPauseOverAll) {
  ScheduleFile file;
  ASSERT_FALSE(ApplySchedule({{"plans", {{{"days", {7}}, {"time", "08:00"}}}}, {"pause", {{"until", "2026-11-20"}}}},
                             true, 100, &file));
  const json out = ScheduleToJson(file.schedule);
  EXPECT_EQ(out["plans"][0], (json{{"days", {7}}, {"time", "08:00"}, {"areas", json::array()}, {"dark", 0},
                                    {"sunset", 0}, {"fresh", 0}}));
  EXPECT_EQ(out["pause"], (json{{"until", "2026-11-20"}, {"areas", "all"}}));
}

TEST(ScheduleModel, EachRuleWithFieldAndReason) {
  EXPECT_EQ(Refusal(json::array()), "schedule type");
  EXPECT_EQ(Refusal({{"color", 1}}), "color unknown");
  EXPECT_EQ(Refusal({{"enabled", true}}), "enabled type");
  EXPECT_EQ(Refusal({{"enabled", 2}}), "enabled type");
  EXPECT_EQ(Refusal({{"skip_rain", "1"}}), "skip_rain type");
  EXPECT_EQ(Refusal({{"skip_forecast", 1.0}}), "skip_forecast type");
  EXPECT_EQ(Refusal({{"skip_cold", -1}}), "skip_cold type");
  EXPECT_EQ(Refusal({{"min_battery", 101}}), "min_battery range");
  EXPECT_EQ(Refusal({{"min_battery", -1}}), "min_battery range");
  EXPECT_EQ(Refusal({{"min_battery", 50.5}}), "min_battery type");
  EXPECT_EQ(Refusal({{"forecast_hours", 4}}), "forecast_hours range");
  EXPECT_EQ(Refusal({{"min_temp", 4}}), "min_temp range");
  EXPECT_EQ(Refusal({{"tz", 1}}), "tz type");
  EXPECT_EQ(Refusal({{"tz", "EST5EDT"}}), "tz missing_rule");
  EXPECT_EQ(Refusal({{"tz", "Europe/Berlin"}}), "tz syntax");
  EXPECT_EQ(Refusal({{"plans", json::object()}}), "plans type");
  EXPECT_EQ(Refusal({{"plans", {1}}}), "plans[0] type");
  EXPECT_EQ(Refusal(OnePlan({{"time", "09:00"}})), "plans[0].days missing");
  EXPECT_EQ(Refusal(OnePlan({{"days", {1}}})), "plans[0].time missing");
  EXPECT_EQ(Refusal(OnePlan({{"days", json::array()}, {"time", "09:00"}})), "plans[0].days type");
  EXPECT_EQ(Refusal(OnePlan({{"days", {0}}, {"time", "09:00"}})), "plans[0].days range");
  EXPECT_EQ(Refusal(OnePlan({{"days", {8}}, {"time", "09:00"}})), "plans[0].days range");
  EXPECT_EQ(Refusal(OnePlan({{"days", {1, 1}}, {"time", "09:00"}})), "plans[0].days duplicate");
  EXPECT_EQ(Refusal(OnePlan({{"days", {"1"}}, {"time", "09:00"}})), "plans[0].days type");
  EXPECT_EQ(Refusal(OnePlan({{"days", {1}}, {"time", "9:00"}})), "plans[0].time format");
  EXPECT_EQ(Refusal(OnePlan({{"days", {1}}, {"time", "24:00"}})), "plans[0].time format");
  EXPECT_EQ(Refusal(OnePlan({{"days", {1}}, {"time", 900}})), "plans[0].time type");
  EXPECT_EQ(Refusal(OnePlan({{"days", {1}}, {"time", "09:00"}, {"end", "09:00"}})), "plans[0].end equal_time");
  EXPECT_EQ(Refusal(OnePlan({{"days", {1}}, {"time", "09:00"}, {"end", "25:00"}})), "plans[0].end format");
  EXPECT_EQ(Refusal(OnePlan({{"days", {1}}, {"time", "09:00"}, {"areas", "a"}})), "plans[0].areas type");
  EXPECT_EQ(Refusal(OnePlan({{"days", {1}}, {"time", "09:00"}, {"areas", {"a", "a"}}})),
            "plans[0].areas duplicate");
  EXPECT_EQ(Refusal(OnePlan({{"days", {1}}, {"time", "09:00"}, {"areas", {""}}})), "plans[0].areas type");
  EXPECT_EQ(Refusal(OnePlan({{"days", {1}}, {"time", "09:00"}, {"areas", {std::string(65, 'a')}}})),
            "plans[0].areas range");
  json many = json::array();
  for (int i = 0; i < 65; ++i) many.push_back("a" + std::to_string(i));
  EXPECT_EQ(Refusal(OnePlan({{"days", {1}}, {"time", "09:00"}, {"areas", many}})), "plans[0].areas too_many");
  EXPECT_EQ(Refusal(OnePlan({{"days", {1}}, {"time", "09:00"}, {"dark", true}})), "plans[0].dark type");
  EXPECT_EQ(Refusal(OnePlan({{"days", {1}}, {"time", "09:00"}, {"sunset", 3}})), "plans[0].sunset type");
  EXPECT_EQ(Refusal(OnePlan({{"days", {1}}, {"time", "09:00"}, {"fresh", false}})), "plans[0].fresh type");
  EXPECT_EQ(Refusal(OnePlan({{"days", {1}}, {"time", "09:00"}, {"angle_offset_deg", 181}})),
            "plans[0].angle_offset_deg range");
  EXPECT_EQ(Refusal(OnePlan({{"days", {1}}, {"time", "09:00"}, {"angle_offset_deg", "5"}})),
            "plans[0].angle_offset_deg type");
  EXPECT_EQ(Refusal(OnePlan({{"days", {1}}, {"time", "09:00"}, {"skip", 1}})), "plans[0].skip unknown");
  json plans = json::array();
  for (int i = 0; i < 33; ++i) plans.push_back({{"days", {1}}, {"time", FormatClockTime(i)}});
  EXPECT_EQ(Refusal({{"plans", plans}}), "plans too_many");
  EXPECT_EQ(Refusal({{"pause", 1}}), "pause type");
  EXPECT_EQ(Refusal({{"pause", {{"areas", "all"}}}}), "pause.until missing");
  EXPECT_EQ(Refusal({{"pause", {{"until", "2026-02-30"}}}}), "pause.until format");
  EXPECT_EQ(Refusal({{"pause", {{"until", "2026-02-20"}, {"areas", json::array()}}}}), "pause.areas empty");
  EXPECT_EQ(Refusal({{"pause", {{"until", "2026-02-20"}, {"areas", "some"}}}}), "pause.areas type");
  EXPECT_EQ(Refusal({{"pause", {{"until", "2026-02-20"}, {"why", 1}}}}), "pause.why unknown");
}

TEST(ScheduleModel, TwoPlansAtOneTimeOnACommonDayAreRefused) {
  const json first = {{"days", {1, 2}}, {"time", "09:00"}};
  EXPECT_EQ(Refusal({{"plans", {first, {{"days", {2, 3}}, {"time", "09:00"}}}}}), "plans[1].time duplicate_time");
  EXPECT_EQ(Refusal({{"plans", {first, {{"days", {3, 4}}, {"time", "09:00"}}}}}), "taken");
  EXPECT_EQ(Refusal({{"plans", {first, {{"days", {1, 2}}, {"time", "09:01"}}}}}), "taken");
}

TEST(ScheduleModel, EnabledOnlyWhereThePlatformAllowsIt) {
  EXPECT_EQ(Refusal({{"enabled", 1}}, false), "enabled platform_off");
  EXPECT_EQ(Refusal({{"enabled", 0}}, false), "taken");
  EXPECT_EQ(Refusal({{"min_battery", 50}}, false), "taken");
}

TEST(ScheduleModel, AllOrNothing) {
  ScheduleFile file;
  ASSERT_FALSE(ApplySchedule({{"min_battery", 50}}, true, 100, &file));
  const json before = FileToJson(file);
  EXPECT_TRUE(ApplySchedule({{"min_battery", 60}, {"min_temp", 4}}, true, 200, &file));
  EXPECT_EQ(FileToJson(file), before);
}

TEST(ScheduleModel, ChangedAtMovesWithPlansEnabledAndTzOnly) {
  ScheduleFile file;
  ASSERT_FALSE(ApplySchedule({{"min_battery", 50}, {"skip_cold", 1}, {"pause", {{"until", "2026-11-20"}}}}, true,
                             100, &file));
  EXPECT_EQ(file.changed_at, 0);
  EXPECT_EQ(file.version, 1);
  ASSERT_FALSE(ApplySchedule({{"enabled", 1}}, true, 200, &file));
  EXPECT_EQ(file.changed_at, 200);
  ASSERT_FALSE(ApplySchedule({{"tz", "<+01>-1"}}, true, 300, &file));
  EXPECT_EQ(file.changed_at, 300);
  ASSERT_FALSE(ApplySchedule(OnePlan({{"days", {1}}, {"time", "09:00"}}), true, 400, &file));
  EXPECT_EQ(file.changed_at, 400);
  // The same again moves nothing.
  ASSERT_FALSE(ApplySchedule(OnePlan({{"days", {1}}, {"time", "09:00"}}), true, 500, &file));
  ASSERT_FALSE(ApplySchedule({{"enabled", 1}, {"tz", "<+01>-1"}}, true, 500, &file));
  EXPECT_EQ(file.changed_at, 400);
  EXPECT_EQ(file.version, 6);
}

TEST(ScheduleModel, LeftoversWaitForThePlanner) {
  ScheduleFile file;
  std::optional<Invalid> bad = ApplyLeftovers({{"enabled", 1}}, &file);
  ASSERT_TRUE(bad);
  EXPECT_EQ(bad->field, "enabled");
  EXPECT_EQ(bad->reason, "unsupported");
  EXPECT_EQ(ApplyLeftovers({{"min_m2", -1}}, &file)->reason, "range");
  EXPECT_EQ(ApplyLeftovers({{"min_m2", "1"}}, &file)->reason, "type");
  EXPECT_EQ(ApplyLeftovers({{"more", 1}}, &file)->reason, "unknown");
  EXPECT_EQ(file.version, 0);
  ASSERT_FALSE(ApplyLeftovers({{"enabled", 0}, {"min_m2", 2.5}}, &file));
  EXPECT_EQ(file.leftovers.min_m2, 2.5);
  EXPECT_EQ(file.version, 1);
}

TEST(ScheduleModel, FileBothWays) {
  ScheduleFile file;
  ASSERT_FALSE(ApplySchedule({{"enabled", 1},
                              {"tz", "CET-1CEST,M3.5.0,M10.5.0/3"},
                              {"plans", {{{"days", {1}}, {"time", "09:00"}, {"end", "08:00"}}}}},
                             true, 1234, &file));
  std::string error;
  const std::optional<ScheduleFile> back = FileFromJson(FileToJson(file), &error);
  ASSERT_TRUE(back) << error;
  EXPECT_EQ(FileToJson(*back), FileToJson(file));
  EXPECT_EQ(back->changed_at, 1234);
  EXPECT_EQ(back->schedule.zone.OffsetAt(0), 3600);
  json other = FileToJson(file);
  other["schema"] = 2;
  EXPECT_FALSE(FileFromJson(other, &error));
  other = FileToJson(file);
  other["schedule"]["min_temp"] = 4;
  EXPECT_FALSE(FileFromJson(other, &error));
  EXPECT_NE(error.find("min_temp"), std::string::npos);
  other = FileToJson(file);
  other.erase("version");
  EXPECT_FALSE(FileFromJson(other, &error));
}

TEST(ScheduleModel, PauseAndPlanLookups) {
  ScheduleFile file;
  ASSERT_FALSE(ApplySchedule({{"plans", {{{"days", {1}}, {"time", "09:00"}}, {{"days", {2}}, {"time", "09:00"}}}},
                              {"pause", {{"until", "2026-11-17"}, {"areas", {"b"}}}}},
                             true, 0, &file));
  const Schedule& schedule = file.schedule;
  EXPECT_FALSE(PausedAll(schedule, Date{2026, 11, 16}));
  EXPECT_TRUE(PausedArea(schedule, Date{2026, 11, 17}, "b"));
  EXPECT_FALSE(PausedArea(schedule, Date{2026, 11, 18}, "b"));
  EXPECT_FALSE(PausedArea(schedule, Date{2026, 11, 16}, "a"));
  EXPECT_EQ(PlanAt(schedule, Date{2026, 11, 16}, 9 * 60), &schedule.plans[0]);
  EXPECT_EQ(PlanAt(schedule, Date{2026, 11, 17}, 9 * 60), &schedule.plans[1]);
  EXPECT_EQ(PlanAt(schedule, Date{2026, 11, 18}, 9 * 60), nullptr);
  EXPECT_EQ(PlanAt(schedule, Date{2026, 11, 16}, 9 * 60 + 1), nullptr);
  ASSERT_FALSE(ApplySchedule({{"pause", {{"until", "2026-11-17"}, {"areas", "all"}}}}, true, 0, &file));
  EXPECT_TRUE(PausedAll(file.schedule, Date{2026, 11, 17}));
  EXPECT_TRUE(PausedArea(file.schedule, Date{2026, 11, 17}, "x"));
  EXPECT_FALSE(PausedAll(file.schedule, Date{2026, 11, 18}));
}

}  // namespace
}  // namespace mowbite_schedule
