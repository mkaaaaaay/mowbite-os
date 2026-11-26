#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>

#include "mowbite_logic/logic_core.h"
#include "mowbite_time/sun.h"
#include "logic_test_core.h"

using namespace mowbite_logic;

namespace {

const std::string kStart = "mowing.start";

// The station of the simulation and a lawn that reaches to 0.5 m in front of it, as in test_logic_undock.cpp.
const DockPose kDock{-1.0, 0.0, M_PI};
const Ring kLawn{{-0.5, -2.0}, {4.0, -2.0}, {4.0, 2.0}, {-0.5, 2.0}};
const double kStartX = kDock.x + 0.02;

// Berlin, and 2026-11-12 12:00 UTC (sun up) and 22:00 UTC (dark).
constexpr double kLat = 52.52;
constexpr double kLon = 13.405;
constexpr double kNoon = 1794484800.0;
constexpr double kNight = kNoon + 10 * 3600.0;

HardwareSnapshot Healthy(bool contact) {
  HardwareSnapshot hw;
  hw.telemetry_ok = true;
  hw.battery_ok = true;
  hw.battery_voltage = 24.5f;
  hw.battery_percentage = 80;
  hw.dock_contact = contact;
  return hw;
}

PoseSnapshot PoseAt(double x, double y, float accuracy = 0.02f, bool heading = true) {
  PoseSnapshot pose;
  pose.x = x;
  pose.y = y;
  pose.position_accuracy = accuracy;
  pose.orientation_accuracy = heading ? 0.01f : 999.0f;
  pose.orientation_valid = heading;
  return pose;
}

LogicConfig Tron() {
  LogicConfig config;
  config.dock_contact_x = -0.10;
  return config;
}

// On the contacts with two areas, the datum and a safe clock at wall time unix_s for monotonic 0.
TestCore Docked(double unix_s = kNoon) {
  TestCore core{Tron()};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy(true));
  core.OnCapabilities(true);
  core.OnPose(PoseAt(kDock.x, kDock.y, 0.02f, false));
  core.OnMowMap({{"a", "A", true}, {"b", "B", true}}, Reason());
  core.OnMapDock(kDock);
  core.OnDrivableAreas({kLawn}, {});
  core.OnDatum(kLat, kLon);
  core.OnClock(true, "", unix_s);
  core.TakeEvents();
  core.TakeNotices();
  core.TakeActionsChanged();
  return core;
}

ScheduleRun Plan() {
  ScheduleRun run;
  run.run_id = "2026-11-12T13:00";
  run.area_ids = {"a", "b"};
  return run;
}

const Event* Find(const std::vector<Event>& events, const std::string& type) {
  auto it = std::find_if(events.begin(), events.end(), [&](const Event& e) { return e.type == type; });
  return it == events.end() ? nullptr : &*it;
}

int CountOf(const std::vector<Event>& events, const std::string& type) {
  return static_cast<int>(std::count_if(events.begin(), events.end(), [&](const Event& e) { return e.type == type; }));
}

// The phases logic/run went through, the end as "ENDED code detail".
std::vector<std::string> Phases(TestCore& core) {
  static const char* kNames[] = {"NONE", "ANNOUNCING", "CHECKING", "UNDOCKING", "MOWING", "ENDED"};
  std::vector<std::string> out;
  for (const auto& run : core.TakeRuns()) {
    std::string text = kNames[static_cast<int>(run.phase)];
    if (run.phase == RunPhase::kEnded) text += " " + run.end_code + " " + run.detail;
    out.push_back(text);
  }
  return out;
}

std::string Ended(TestCore& core) {
  const auto phases = Phases(core);
  return phases.empty() ? "" : phases.back();
}

// The announcement ran out and the reference check passed: charging off, out of the station, mowing.
void ToMowing(TestCore& core, double now_s = kAnnounceS) {
  core.OnTick(now_s);
  ASSERT_TRUE(core.TakeDockCheck());
  ASSERT_TRUE(core.OnDockCheck(true, kStartX, kDock.y, 0.02f, "").empty());
  EXPECT_TRUE(core.TakeChargeOff());
  core.OnChargeOff(true, "charging off");
  ASSERT_TRUE(core.TakeUndockPath());
  core.OnHardwareStatus(Healthy(false));
  core.OnPose(PoseAt(kStartX + 0.75, kDock.y));
  core.OnUndockEnded(LegEnd::kDone, Reason("DONE", ""));
  ASSERT_TRUE(core.Mowing());
}

// A run of the plan that mows.
TestCore MowingRun(ScheduleRun run = Plan(), double unix_s = kNoon) {
  TestCore core = Docked(unix_s);
  EXPECT_TRUE(core.StartRun(run, Reason(), 0).empty());
  ToMowing(core);
  core.TakeEvents();
  core.TakeNotices();
  return core;
}

// The plan of the area the job asks for has no paths: on to the next.
void NothingToMow(TestCore& core) {
  ASSERT_EQ(core.NextJobStep(20).kind, JobStep::Kind::kPlan);
  core.OnJobPlanned({}, "fp");
}

}  // namespace

TEST(ScheduleStart, AcceptedItAnnouncesTenSecondsAndStaysIdle) {
  TestCore core = Docked();
  EXPECT_TRUE(core.StartRun(Plan(), Reason(), 100).empty());
  const auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "SCHEDULE_START");
  EXPECT_EQ(events[0].details["seconds"], 10);
  EXPECT_EQ(events[0].details["until"], static_cast<int64_t>(kNoon + 110));
  EXPECT_EQ(events[0].details["source"], "schedule");
  EXPECT_EQ(Phases(core), std::vector<std::string>{"ANNOUNCING"});
  EXPECT_EQ(core.RobotState().state, "IDLE");
  EXPECT_EQ(core.RobotState().sub_state, "");
  EXPECT_FALSE(core.HoldAllowed());
  core.OnTick(109.9);
  EXPECT_FALSE(core.TakeDockCheck());
  core.OnTick(110);
  EXPECT_TRUE(core.TakeDockCheck());
  EXPECT_EQ(Phases(core), std::vector<std::string>{"CHECKING"});
  EXPECT_FALSE(core.HoldAllowed());
}

TEST(ScheduleStart, ToMowingThroughEveryPhaseWithSourceSchedule) {
  TestCore core = Docked();
  ASSERT_TRUE(core.StartRun(Plan(), Reason(), 0).empty());
  ToMowing(core);
  EXPECT_EQ(Phases(core), (std::vector<std::string>{"ANNOUNCING", "CHECKING", "UNDOCKING", "MOWING"}));
  const auto events = core.TakeEvents();
  ASSERT_TRUE(Find(events, "UNDOCKED"));
  for (const auto& event : events) EXPECT_EQ(event.details.value("source", ""), "schedule") << event.type;
  EXPECT_EQ(core.RobotState().task_count, 2);
}

TEST(ScheduleStart, AListTheLogicDoesNotTakeIsInvalidWithoutAnEvent) {
  const auto invalid = [](ScheduleRun run) {
    TestCore core = Docked();
    const Reason why = core.StartRun(run, Reason(), 0);
    if (why.code == "INVALID") {
      EXPECT_TRUE(core.TakeEvents().empty());
      EXPECT_TRUE(core.TakeRuns().empty());
      EXPECT_TRUE(core.HoldAllowed());
    }
    return why.code;
  };
  ScheduleRun unknown = Plan();
  unknown.area_ids.push_back("gone");
  EXPECT_EQ(invalid(unknown), "INVALID");
  unknown.pass_over = {"gone"};
  EXPECT_EQ(invalid(unknown), "");
  ScheduleRun outside = Plan();
  outside.pass_over = {"c"};
  EXPECT_EQ(invalid(outside), "INVALID");
  ScheduleRun twice = Plan();
  twice.area_ids = {"a", "a"};
  EXPECT_EQ(invalid(twice), "INVALID");
  ScheduleRun nameless = Plan();
  nameless.run_id.clear();
  EXPECT_EQ(invalid(nameless), "INVALID");
  ScheduleRun negative = Plan();
  negative.end_in_s = -1;
  EXPECT_EQ(invalid(negative), "INVALID");
  // With every area pass_over names map areas.
  ScheduleRun all;
  all.run_id = "r";
  all.pass_over = {"a"};
  EXPECT_EQ(invalid(all), "");
}

TEST(ScheduleStart, RefusalsInTheirOrderWithSourceSchedule) {
  const auto refused = [](TestCore& core, const Reason& node_problem = Reason()) {
    const Reason why = core.StartRun(Plan(), node_problem, 0);
    const auto events = core.TakeEvents();
    EXPECT_EQ(events.size(), 1u) << why.code;
    if (!events.empty()) {
      EXPECT_EQ(events[0].type, "MOWING_REFUSED");
      EXPECT_EQ(events[0].details["reason"], why.code);
      EXPECT_EQ(events[0].details["source"], "schedule");
    }
    EXPECT_TRUE(core.TakeRuns().empty());
    EXPECT_TRUE(core.HoldAllowed()) << why.code;
    return why;
  };
  // Off the contacts with an unsafe clock in the dark: not docked first.
  TestCore off = Docked(kNight);
  off.OnHardwareStatus(Healthy(false));
  off.OnClock(false, "never_synced", kNight);
  EXPECT_EQ(refused(off).code, "NOT_DOCKED");
  // A hold before that.
  off.SetHold(1);
  EXPECT_EQ(refused(off).code, "HOLD");
  // On the contacts: the clock, with its reason.
  TestCore clock = Docked(kNight);
  clock.OnClock(false, "never_synced", kNight);
  HardwareSnapshot emergency = Healthy(true);
  emergency.emergency = true;
  emergency.emergency_reason = kEmergencyLatch | kEmergencyLift;
  clock.OnHardwareStatus(emergency);
  clock.TakeEvents();
  const Reason unsafe = refused(clock);
  EXPECT_EQ(unsafe.code, "CLOCK_UNSAFE");
  EXPECT_EQ(unsafe.fields["clock_reason"], "never_synced");
  // Then the problems of a start by hand before the sun.
  clock.OnClock(true, "", kNight);
  EXPECT_EQ(refused(clock).code, "EMERGENCY");
  TestCore dark = Docked(kNight);
  const Reason night = refused(dark, Reason("PLANNER_MISSING", "The planner is not running"));
  EXPECT_EQ(night.code, "DARK");
  EXPECT_LT(night.fields["sun_elevation"].get<double>(), mowbite_time::kSunsetElevation);
  EXPECT_DOUBLE_EQ(night.fields["limit"].get<double>(), mowbite_time::kSunsetElevation);
  // The node's problem last.
  TestCore day = Docked();
  EXPECT_EQ(refused(day, Reason("PLANNER_MISSING", "The planner is not running")).code, "PLANNER_MISSING");
}

// A run of the schedule waives nothing: what holds the mower refuses it like rain, also after the announcement.
TEST(ScheduleStart, AnAnimalOutOfViewThatHoldsTheMowerRefusesTheRun) {
  const ObstacleSample animal{true, {{7, "ANIMAL", true, false, -0.3, 2, 0.0, 0.0, 0.1, 0.05}}};
  TestCore core = Docked();
  core.SetObstacleMode(2, 2);
  core.OnObstacleStatus(animal, 0);
  const Reason why = core.StartRun(Plan(), Reason(), 0);
  EXPECT_EQ(why.code, "OBSTACLE");
  EXPECT_EQ(why.fields["kind"], "ANIMAL");
  const auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "MOWING_REFUSED");
  EXPECT_EQ(events[0].details["source"], "schedule");
  EXPECT_TRUE(core.TakeRuns().empty());
  // Came during the announcement: refused at its end.
  TestCore announced = Docked();
  announced.SetObstacleMode(2, 2);
  ASSERT_TRUE(announced.StartRun(Plan(), Reason(), 0).empty());
  announced.OnObstacleStatus(animal, 5);
  announced.OnTick(kAnnounceS);
  EXPECT_FALSE(announced.TakeDockCheck());
  EXPECT_EQ(Ended(announced), "ENDED REFUSED OBSTACLE");
  // Came during the reference check: refused there, before undocking.
  TestCore checked = Docked();
  checked.SetObstacleMode(2, 2);
  ASSERT_TRUE(checked.StartRun(Plan(), Reason(), 0).empty());
  checked.OnTick(kAnnounceS);
  ASSERT_TRUE(checked.TakeDockCheck());
  checked.OnObstacleStatus(animal, kAnnounceS + 0.1);
  EXPECT_EQ(checked.OnDockCheck(true, kStartX, kDock.y, 0.02f, "").code, "OBSTACLE");
  EXPECT_FALSE(checked.Undocking());
  EXPECT_EQ(Ended(checked), "ENDED REFUSED OBSTACLE");
}

TEST(ScheduleStart, InTheDarkOnlyWithDarkAllowedAndADatumForTheSun) {
  ScheduleRun dark = Plan();
  dark.dark_allowed = true;
  TestCore night = Docked(kNight);
  EXPECT_TRUE(night.StartRun(dark, Reason(), 0).empty());
  TestCore nodatum{Tron()};
  nodatum.OnMapStatus(MapDatum::kMatch, "");
  nodatum.OnHardwareStatus(Healthy(true));
  nodatum.OnCapabilities(true);
  nodatum.OnPose(PoseAt(kDock.x, kDock.y, 0.02f, false));
  nodatum.OnMowMap({{"a", "A", true}, {"b", "B", true}}, Reason());
  nodatum.OnMapDock(kDock);
  nodatum.OnDrivableAreas({kLawn}, {});
  nodatum.OnClock(true, "", kNoon);
  EXPECT_EQ(nodatum.StartRun(Plan(), Reason(), 0).code, "NO_POSITION");
  EXPECT_TRUE(nodatum.StartRun(dark, Reason(), 0).empty());
}

TEST(ScheduleStart, EveryAreaPassedOverIsNoMowingAreaAndTheJobStays) {
  TestCore core = Docked();
  ScheduleRun run = Plan();
  run.pass_over = {"a", "b"};
  EXPECT_EQ(core.StartRun(run, Reason(), 0).code, "NO_MOWING_AREA");
  EXPECT_EQ(core.JobId(), "");
}

TEST(ScheduleStart, ASecondStartWhileOneIsAnnouncedIsBusy) {
  TestCore core = Docked();
  ASSERT_TRUE(core.StartRun(Plan(), Reason(), 0).empty());
  core.TakeEvents();
  ScheduleRun other = Plan();
  other.run_id = "other";
  const Reason why = core.StartRun(other, Reason(), 1);
  EXPECT_EQ(why.code, "BUSY");
  EXPECT_EQ(why.fields["state"], "announcing");
  EXPECT_EQ(Phases(core), std::vector<std::string>{"ANNOUNCING"});
}

TEST(ScheduleAnnouncement, EachSourceCancelsIt) {
  const auto canceled = [](const std::function<void(TestCore&)>& act) {
    TestCore core = Docked();
    EXPECT_TRUE(core.StartRun(Plan(), Reason(), 0).empty());
    core.TakeEvents();
    core.TakeRuns();
    act(core);
    const auto events = core.TakeEvents();
    const Event* event = Find(events, "SCHEDULE_START_CANCELED");
    EXPECT_TRUE(event);
    const std::string source = event ? event->details["source"].get<std::string>() : "";
    if (event) {
      EXPECT_EQ(event->details["severity"], "info");
      EXPECT_EQ(event->details["resume"], "auto");
    }
    EXPECT_EQ(Ended(core), "ENDED CANCELED " + source);
    // Nothing of the run starts once the time is over; a drive the command asked for checks on its own.
    core.TakeDockCheck();
    core.OnTick(kAnnounceS + 1);
    EXPECT_FALSE(core.TakeDockCheck());
    EXPECT_TRUE(core.TakeRuns().empty());
    return source;
  };
  EXPECT_EQ(canceled([](TestCore& core) {
              HardwareSnapshot hw = Healthy(true);
              hw.emergency = true;
              hw.emergency_reason = kEmergencyStop;
              core.OnHardwareStatus(hw);
            }),
            "emergency");
  EXPECT_EQ(canceled([](TestCore& core) { core.OnHardwareTimeout(); }), "emergency");
  EXPECT_EQ(canceled([](TestCore& core) { core.OnHardwareStatus(Healthy(false)); }), "undocked");
  EXPECT_EQ(canceled([](TestCore& core) {
              HardwareSnapshot hw = Healthy(true);
              hw.hold = true;
              core.OnHardwareStatus(hw);
            }),
            "hold");
  EXPECT_EQ(canceled([](TestCore& core) { core.SetHold(static_cast<int64_t>(kNoon) + 600); }), "hold");
  EXPECT_EQ(canceled([](TestCore& core) { core.Command("mowing.reset_job", json::object()); }), "rpc");
  EXPECT_EQ(canceled([](TestCore& core) { core.OnAction("start_manual_mowing"); }), "rpc");
  EXPECT_EQ(canceled([](TestCore& core) { core.AbortCalibration(Reason("STOPPED", "by hand")); }), "rpc");
  EXPECT_EQ(canceled([](TestCore& core) { core.RefuseCalibration(Reason("NO_ROOM", "no room")); }), "rpc");
  EXPECT_EQ(canceled([](TestCore& core) { core.CheckDockBeforeDrive(1.0, 0.0); }), "rpc");
  EXPECT_EQ(canceled([](TestCore& core) { core.RefuseDrive(Reason("PLANNER_MISSING", "no planner")); }), "rpc");
}

TEST(ScheduleAnnouncement, AStartByHandCancelsItAndRunsAsARunByHand) {
  TestCore core = Docked();
  ASSERT_TRUE(core.StartRun(Plan(), Reason(), 0).empty());
  core.TakeEvents();
  core.TakeRuns();
  EXPECT_EQ(core.OnAction(kStart), ActionResult::kDone);
  EXPECT_EQ(Ended(core), "ENDED CANCELED rpc");
  EXPECT_EQ(Find(core.TakeEvents(), "SCHEDULE_START_CANCELED")->details["source"], "rpc");
  EXPECT_TRUE(core.TakeDockCheck());
  ASSERT_TRUE(core.OnDockCheck(true, kStartX, kDock.y, 0.02f, "").empty());
  for (const auto& event : core.TakeEvents()) EXPECT_FALSE(event.details.contains("source")) << event.type;
  EXPECT_TRUE(core.TakeRuns().empty());
}

TEST(ScheduleAnnouncement, AStartByHandFromBeforeTheCancelCarriesNoSource) {
  TestCore core = Docked();
  ASSERT_TRUE(core.StartRun(Plan(), Reason(), 0).empty());
  core.TakeEvents();
  const CommandOutcome outcome = core.Command("mowing.start", json::object());
  EXPECT_TRUE(outcome.accepted);
  const auto events = core.TakeEvents();
  ASSERT_TRUE(Find(events, "SCHEDULE_START_CANCELED"));
  EXPECT_EQ(Find(events, "SCHEDULE_START_CANCELED")->details["source"], "rpc");
}

TEST(ScheduleAnnouncement, TheCheckAfterItRefuses) {
  TestCore core = Docked();
  ASSERT_TRUE(core.StartRun(Plan(), Reason(), 0).empty());
  core.TakeEvents();
  core.TakeRuns();
  HardwareSnapshot rain = Healthy(true);
  rain.rain = true;
  core.OnHardwareStatus(rain);
  core.OnTick(5);
  EXPECT_TRUE(core.TakeEvents().empty());
  core.OnTick(kAnnounceS);
  const auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "MOWING_REFUSED");
  EXPECT_EQ(events[0].details["reason"], "RAIN");
  EXPECT_EQ(events[0].details["source"], "schedule");
  EXPECT_EQ(Ended(core), "ENDED REFUSED RAIN");
  EXPECT_FALSE(core.TakeDockCheck());
  EXPECT_TRUE(core.HoldAllowed());
}

TEST(ScheduleAnnouncement, TheCheckAfterItSeesTheClock) {
  TestCore core = Docked();
  ASSERT_TRUE(core.StartRun(Plan(), Reason(), 0).empty());
  core.TakeRuns();
  core.OnClock(false, "clock_jump", kNoon);
  core.OnTick(kAnnounceS);
  EXPECT_EQ(Ended(core), "ENDED REFUSED CLOCK_UNSAFE");
}

TEST(ScheduleRunEnd, TheReferenceCheckRefuses) {
  TestCore core = Docked();
  ASSERT_TRUE(core.StartRun(Plan(), Reason(), 0).empty());
  core.OnTick(kAnnounceS);
  ASSERT_TRUE(core.TakeDockCheck());
  core.TakeRuns();
  EXPECT_EQ(core.OnDockCheck(true, kDock.x + 0.5, kDock.y, 0.02f, "").code, "REFERENCE_MISMATCH");
  EXPECT_EQ(Ended(core), "ENDED REFUSED REFERENCE_MISMATCH");
  EXPECT_EQ(Find(core.TakeEvents(), "MOWING_REFUSED")->details["source"], "schedule");
  EXPECT_TRUE(core.HoldAllowed());
}

TEST(ScheduleRunEnd, UndockingFailsOnceWithoutASecondTry) {
  TestCore core = Docked();
  ASSERT_TRUE(core.StartRun(Plan(), Reason(), 0).empty());
  core.OnTick(kAnnounceS);
  ASSERT_TRUE(core.OnDockCheck(true, kStartX, kDock.y, 0.02f, "").empty());
  core.OnChargeOff(true, "off");
  ASSERT_TRUE(core.TakeUndockPath());
  core.TakeRuns();
  core.OnUndockEnded(LegEnd::kFailed, Reason("STALLED", "stuck"));
  EXPECT_EQ(Ended(core), "ENDED UNDOCKING_FAILED path_failed");
  EXPECT_EQ(core.RobotState().state, "IDLE");
  core.OnTick(100);
  EXPECT_FALSE(core.TakeDockCheck());
  EXPECT_FALSE(core.TakeUndockPath());
}

TEST(ScheduleRunEnd, AnEmergencyWhileUndockingEndsItAsEmergency) {
  TestCore core = Docked();
  ASSERT_TRUE(core.StartRun(Plan(), Reason(), 0).empty());
  core.OnTick(kAnnounceS);
  ASSERT_TRUE(core.OnDockCheck(true, kStartX, kDock.y, 0.02f, "").empty());
  core.TakeRuns();
  HardwareSnapshot hw = Healthy(true);
  hw.emergency = true;
  hw.emergency_reason = kEmergencyLift;
  core.OnHardwareStatus(hw);
  EXPECT_EQ(Ended(core), "ENDED EMERGENCY ");
  const auto emergency_events = core.TakeEvents();
  const Event* emergency = Find(emergency_events, "EMERGENCY");
  ASSERT_TRUE(emergency);
  EXPECT_FALSE(emergency->details.contains("source"));
}

TEST(ScheduleRunEnd, JobCompleteOnceTheMowerIsIdle) {
  TestCore core = MowingRun();
  core.TakeRuns();
  NothingToMow(core);
  NothingToMow(core);
  EXPECT_EQ(core.NextJobStep(20).kind, JobStep::Kind::kNone);
  EXPECT_TRUE(core.Docking());
  EXPECT_TRUE(core.TakeRuns().empty());
  EXPECT_TRUE(core.Command("docking.abort", json::object()).accepted);
  EXPECT_EQ(Ended(core), "ENDED JOB_COMPLETE ");
  const auto events = core.TakeEvents();
  ASSERT_TRUE(Find(events, "JOB_COMPLETE"));
  EXPECT_EQ(Find(events, "JOB_COMPLETE")->details["source"], "schedule");
}

TEST(ScheduleRunEnd, AbortByHandIsManual) {
  TestCore core = MowingRun();
  core.TakeRuns();
  EXPECT_TRUE(core.Command("mowing.abort", json::object()).accepted);
  EXPECT_TRUE(core.Docking());
  EXPECT_TRUE(core.Command("docking.abort", json::object()).accepted);
  EXPECT_EQ(Ended(core), "ENDED MANUAL ");
  // The next events belong to no run.
  core.TakeEvents();
  core.OnHardwareStatus(Healthy(true));
  core.OnHardwareStatus(Healthy(false));
  for (const auto& event : core.TakeEvents()) EXPECT_FALSE(event.details.contains("source")) << event.type;
}

TEST(ScheduleRunEnd, TheBatteryEndsItWithoutAWayHome) {
  TestCore core = MowingRun();
  core.OnMapDock(std::nullopt);
  core.TakeRuns();
  HardwareSnapshot low = Healthy(false);
  low.battery_voltage = 19.0f;
  core.OnHardwareStatus(low);
  for (double t = 20; t <= 30; t += 1) core.OnTick(t);
  EXPECT_EQ(Ended(core), "ENDED BATTERY_LOW ");
}

TEST(ScheduleDeadline, TheEndOfThePlanSendsItHome) {
  ScheduleRun run = Plan();
  run.end_in_s = 600;
  TestCore core = MowingRun(run);
  for (double t = 20; t < 600; t += 10) core.OnTick(t);
  EXPECT_TRUE(core.Mowing());
  core.OnTick(600);
  EXPECT_TRUE(core.Docking());
  const auto docking_events = core.TakeEvents();
  const Event* docking = Find(docking_events, "DOCKING");
  ASSERT_TRUE(docking);
  EXPECT_EQ(docking->details["reason"], "SCHEDULE");
  EXPECT_EQ(docking->details["detail"], "end");
  EXPECT_EQ(docking->details["severity"], "info");
  EXPECT_EQ(docking->details["resume"], "auto");
  EXPECT_EQ(docking->details["source"], "schedule");
  core.Command("docking.abort", json::object());
  EXPECT_EQ(Ended(core), "ENDED SCHEDULE end");
}

TEST(ScheduleDeadline, WithoutAWayHomeItPausesForTheUser) {
  ScheduleRun run = Plan();
  run.end_in_s = 600;
  TestCore core = MowingRun(run);
  core.OnMapDock(std::nullopt);
  core.OnTick(600);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  const auto paused_events = core.TakeEvents();
  const Event* paused = Find(paused_events, "STATE");
  ASSERT_TRUE(paused);
  EXPECT_EQ(paused->details["reason"], "SCHEDULE");
  EXPECT_EQ(paused->details["severity"], "info");
  EXPECT_EQ(paused->details["resume"], "user");
}

TEST(ScheduleDeadline, NeverFromAPauseByHandAndContinueDropsTheEndThatCame) {
  ScheduleRun run = Plan();
  run.end_in_s = 600;
  TestCore core = MowingRun(run);
  EXPECT_TRUE(core.Command("mowing.pause", json::object()).accepted);
  for (double t = 590; t <= 700; t += 10) core.OnTick(t);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_FALSE(Find(core.TakeEvents(), "DOCKING"));
  EXPECT_TRUE(core.Command("mowing.continue", json::object()).accepted);
  for (double t = 710; t <= 2000; t += 10) core.OnTick(t);
  EXPECT_TRUE(core.Mowing());
  EXPECT_FALSE(Find(core.TakeEvents(), "DOCKING"));
}

TEST(ScheduleDeadline, ContinueKeepsAnEndToCome) {
  ScheduleRun run = Plan();
  run.end_in_s = 600;
  TestCore core = MowingRun(run);
  EXPECT_TRUE(core.Command("mowing.pause", json::object()).accepted);
  core.OnTick(100);
  EXPECT_TRUE(core.Command("mowing.continue", json::object()).accepted);
  core.OnTick(599);
  EXPECT_TRUE(core.Mowing());
  core.OnTick(600);
  EXPECT_TRUE(core.Docking());
}

namespace {

double SunsetAfter(double unix_s) {
  for (const double day : {unix_s, unix_s + 86400.0}) {
    const auto set = mowbite_time::SunTimes(kLat, kLon, day).set;
    if (set && *set > unix_s) return *set;
  }
  return 0;
}

// Ticks of a run that started at kNoon past its sunset; whether it went home for DARK.
bool HomeAtSunset(TestCore& core) {
  const double due = SunsetAfter(kNoon) - kNoon;
  for (double t = 20; t <= due + 1; t += 60) core.OnTick(t);
  core.OnTick(due + 1);
  const auto docking_events = core.TakeEvents();
  const Event* docking = Find(docking_events, "DOCKING");
  return docking && docking->details["reason"] == "DARK";
}

}  // namespace

TEST(ScheduleDeadline, SunsetAlwaysButForDarkWithoutUntilSunset) {
  // The switch dock_at_sunset is off.
  TestCore plain = MowingRun();
  EXPECT_TRUE(HomeAtSunset(plain));
  ScheduleRun dark = Plan();
  dark.dark_allowed = true;
  TestCore with_sunset = MowingRun(dark);
  EXPECT_TRUE(HomeAtSunset(with_sunset));
  dark.until_sunset = false;
  TestCore no_sunset = MowingRun(dark);
  EXPECT_FALSE(HomeAtSunset(no_sunset));
  EXPECT_TRUE(no_sunset.Mowing());
  // The switch does not drop the deadline of a run of the schedule.
  TestCore switched = MowingRun();
  switched.SetSunsetSettings(false);
  EXPECT_TRUE(HomeAtSunset(switched));
}

TEST(SchedulePassOver, AStartLeavesThemOutWithAreaSkipped) {
  ScheduleRun run = Plan();
  run.pass_over = {"a"};
  TestCore core = MowingRun(run);
  const JobStep step = core.NextJobStep(20);
  ASSERT_EQ(step.kind, JobStep::Kind::kPlan);
  EXPECT_EQ(step.area_id, "b");
  const auto events = core.TakeEvents();
  const Event* skipped = Find(events, "AREA_SKIPPED");
  ASSERT_TRUE(skipped);
  EXPECT_EQ(skipped->details["area_id"], "a");
  EXPECT_EQ(skipped->details["reason"], "SCHEDULE");
  EXPECT_EQ(skipped->details["detail"], "pause");
  EXPECT_EQ(skipped->details["severity"], "info");
  EXPECT_EQ(core.RobotState().task_count, 2);
  EXPECT_EQ(core.RobotState().task_index, 1);
  // Never in the checkpoint.
  const auto checkpoint = core.TakeCheckpoint();
  ASSERT_TRUE(checkpoint);
  EXPECT_EQ(checkpoint->find("pass"), std::string::npos);
  // The list after it is complete.
  core.OnJobPlanned({}, "fp");
  core.NextJobStep(20);
  EXPECT_TRUE(Find(core.TakeEvents(), "JOB_COMPLETE"));
}

TEST(SchedulePassOver, AllAreasOfTheMapLeaveOutTheOnesNamed) {
  ScheduleRun run;
  run.run_id = "all";
  run.pass_over = {"b"};
  TestCore core = MowingRun(run);
  EXPECT_EQ(core.NextJobStep(20).area_id, "a");
  core.OnJobPlanned({}, "fp");
  core.NextJobStep(20);
  const auto events = core.TakeEvents();
  EXPECT_EQ(CountOf(events, "AREA_SKIPPED"), 1);
  EXPECT_TRUE(Find(events, "JOB_COMPLETE"));
}

TEST(ScheduleDrop, OnlyTheRunningRunOfTheScheduleWhileItMows) {
  TestCore core = Docked();
  EXPECT_EQ(core.DropTasks("x", {}).code, "NOT_ALLOWED");
  ASSERT_TRUE(core.StartRun(Plan(), Reason(), 0).empty());
  EXPECT_EQ(core.DropTasks(Plan().run_id, {}).code, "WRONG_STATE");
  EXPECT_EQ(core.DropTasks("other", {}).code, "NOT_ALLOWED");
  // A run by hand is not the schedule's.
  TestCore hand = Docked();
  ASSERT_EQ(hand.OnAction(kStart), ActionResult::kDone);
  EXPECT_EQ(hand.DropTasks(Plan().run_id, {}).code, "NOT_ALLOWED");
}

TEST(ScheduleDrop, TheCurrentAreaIsSkippedAndTheNextOneMowed) {
  TestCore core = MowingRun();
  ASSERT_EQ(core.NextJobStep(20).area_id, "a");
  core.TakeEvents();
  EXPECT_TRUE(core.DropTasks(Plan().run_id, {"a"}).empty());
  const auto skipped_events = core.TakeEvents();
  const Event* skipped = Find(skipped_events, "AREA_SKIPPED");
  ASSERT_TRUE(skipped);
  EXPECT_EQ(skipped->details["reason"], "SCHEDULE");
  EXPECT_EQ(skipped->details["detail"], "pause");
  EXPECT_EQ(core.NextJobStep(20).area_id, "b");
  EXPECT_TRUE(core.Mowing());
}

// Masterplan 3.4: an area the planner cannot plan as it is is skipped with the code as its reason, the job goes on.
TEST(PlannerRefusal, AnAreaThePlannerCannotPlanIsSkipped) {
  for (const char* code : {"BODY_OUT", "NO_PLAN", "START_UNREACHABLE"}) {
    TestCore core = MowingRun();
    ASSERT_EQ(core.NextJobStep(20).area_id, "a") << code;
    core.TakeEvents();
    core.OnJobPlanFailed(Reason(code, "not as it is"));
    EXPECT_TRUE(core.Mowing()) << code;
    const auto events = core.TakeEvents();
    const Event* skipped = Find(events, "AREA_SKIPPED");
    ASSERT_TRUE(skipped) << code;
    EXPECT_EQ(skipped->details["area_id"], "a");
    EXPECT_EQ(skipped->details["reason"], code);
    EXPECT_EQ(skipped->details["severity"], "warning");
    EXPECT_EQ(skipped->details["message"], "not as it is");
    EXPECT_FALSE(Find(events, "STATE")) << code;
    const JobStep next = core.NextJobStep(20);
    EXPECT_EQ(next.kind, JobStep::Kind::kPlan) << code;
    EXPECT_EQ(next.area_id, "b") << code;
    // The last one as well: the job is complete.
    core.OnJobPlanFailed(Reason(code, "not as it is"));
    core.NextJobStep(20);
    EXPECT_TRUE(Find(core.TakeEvents(), "JOB_COMPLETE")) << code;
  }
}

// Any other failure of the planner still pauses.
TEST(PlannerRefusal, AnyOtherFailurePauses) {
  TestCore core = MowingRun();
  ASSERT_EQ(core.NextJobStep(20).area_id, "a");
  core.TakeEvents();
  core.OnJobPlanFailed(Reason("PLANNING_FAILED", "internal"));
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_FALSE(Find(core.TakeEvents(), "AREA_SKIPPED"));
}

TEST(ScheduleDrop, AnAreaLaterInTheListIsPassedOverThere) {
  TestCore core = MowingRun();
  ASSERT_EQ(core.NextJobStep(20).area_id, "a");
  core.TakeEvents();
  EXPECT_TRUE(core.DropTasks(Plan().run_id, {"b"}).empty());
  EXPECT_FALSE(Find(core.TakeEvents(), "AREA_SKIPPED"));
  core.OnJobPlanned({}, "fp");
  core.NextJobStep(20);
  const auto events = core.TakeEvents();
  EXPECT_TRUE(Find(events, "AREA_SKIPPED"));
  EXPECT_TRUE(Find(events, "JOB_COMPLETE"));
}

TEST(ScheduleDrop, TheAreaMowedNowIsNamedWhileMowing) {
  TestCore core = MowingRun();
  ASSERT_EQ(core.NextJobStep(20).area_id, "a");
  EXPECT_EQ(core.MowingAreaId(), "a");
  EXPECT_TRUE(core.DropTasks(Plan().run_id, {}).empty());
  EXPECT_EQ(core.MowingAreaId(), "");
}

TEST(ScheduleDrop, NothingLeftGoesHomeAndTheJobStaysOpen) {
  TestCore core = MowingRun();
  ASSERT_EQ(core.NextJobStep(20).area_id, "a");
  JobPath lane;
  for (int i = 0; i < 3; ++i) lane.poses.push_back(JobPose{0.5 * i, 1.0, 0, kSegmentLane});
  core.OnJobPlanned({lane, lane}, "fp");
  core.OnJobProgress(0);
  core.TakeRuns();
  core.TakeEvents();
  EXPECT_TRUE(core.DropTasks(Plan().run_id, {}).empty());
  EXPECT_TRUE(core.Docking());
  const auto events = core.TakeEvents();
  const Event* docking = Find(events, "DOCKING");
  ASSERT_TRUE(docking);
  EXPECT_EQ(docking->details["reason"], "SCHEDULE");
  EXPECT_EQ(docking->details["detail"], "pause");
  EXPECT_FALSE(Find(events, "JOB_COMPLETE"));
  EXPECT_FALSE(Find(events, "AREA_SKIPPED"));
  EXPECT_NE(core.JobId(), "");
  core.Command("docking.abort", json::object());
  EXPECT_EQ(Ended(core), "ENDED SCHEDULE pause");
}

TEST(ScheduleDrop, FromAPauseThatWaitsForTheUserItEndsWhereItIs) {
  TestCore core = MowingRun();
  ASSERT_EQ(core.NextJobStep(20).area_id, "a");
  EXPECT_TRUE(core.Command("mowing.pause", json::object()).accepted);
  core.TakeRuns();
  core.TakeEvents();
  EXPECT_TRUE(core.DropTasks(Plan().run_id, {"a", "b"}).empty());
  EXPECT_EQ(core.RobotState().state, "IDLE");
  EXPECT_FALSE(core.Docking());
  EXPECT_FALSE(Find(core.TakeEvents(), "DOCKING"));
  EXPECT_EQ(Ended(core), "ENDED SCHEDULE pause");
}

TEST(ScheduleHold, NotWithinHalfAnHourOfTheNextStartNorOnceItIsDue) {
  EXPECT_FALSE(StartBlocksHold(0, 1000));
  EXPECT_TRUE(StartBlocksHold(1000 + kHoldBeforeStartS, 1000));
  EXPECT_FALSE(StartBlocksHold(1001 + kHoldBeforeStartS, 1000));
  EXPECT_TRUE(StartBlocksHold(1000, 1000));
  EXPECT_TRUE(StartBlocksHold(900, 1000));
}

TEST(ScheduleAngle, AJobTakesTheAngleOfThePlanThatBeginsIt) {
  ScheduleRun run = Plan();
  run.has_angle = true;
  run.angle_offset_deg = 30;
  TestCore core = Docked();
  ASSERT_TRUE(core.StartRun(run, Reason(), 0).empty());
  // Nothing before the job begins.
  EXPECT_FALSE(core.PlanAngle());
  ToMowing(core);
  ASSERT_TRUE(core.PlanAngle());
  EXPECT_DOUBLE_EQ(*core.PlanAngle(), 30);
  const auto checkpoint = core.TakeCheckpoint();
  ASSERT_TRUE(checkpoint);
  EXPECT_DOUBLE_EQ(*ParseCheckpoint(*checkpoint)->angle_offset_deg, 30);
  EXPECT_FALSE(MowingRun().PlanAngle());
}

namespace {

// An open job of areas a and b, begun with 30 deg.
JobProgress OpenJob() {
  JobProgress saved;
  saved.job_id = "job";
  saved.tasks = {"a", "b"};
  saved.task = 1;
  saved.area = 1;
  saved.area_id = "b";
  saved.angle_offset_deg = 30;
  return saved;
}

}  // namespace

TEST(ScheduleAngle, AnOpenJobKeepsItsAngleUntilARestart) {
  ScheduleRun run = Plan();
  run.has_angle = true;
  run.angle_offset_deg = 50;
  TestCore core = Docked();
  core.RestoreJob(OpenJob());
  ASSERT_TRUE(core.StartRun(run, Reason(), 0).empty());
  ToMowing(core);
  EXPECT_EQ(core.JobId(), "job");
  EXPECT_DOUBLE_EQ(*core.PlanAngle(), 30);

  run.restart = true;
  TestCore fresh = Docked();
  fresh.RestoreJob(OpenJob());
  ASSERT_TRUE(fresh.StartRun(run, Reason(), 0).empty());
  ToMowing(fresh);
  EXPECT_NE(fresh.JobId(), "job");
  EXPECT_DOUBLE_EQ(*fresh.PlanAngle(), 50);
}

TEST(ScheduleAngle, AStartByHandGoesOnWithTheAngleOfTheJob) {
  TestCore core = Docked();
  core.RestoreJob(OpenJob());
  ASSERT_TRUE(
      core.Command("mowing.start", json{{"tasks", json::array({{{"area_id", "a"}}, {{"area_id", "b"}}})}}).accepted);
  EXPECT_DOUBLE_EQ(*core.PlanAngle(), 30);
}

// The record of the start failed while the run mowed: the run ends REFUSED with the reason of the disk.
TEST(ScheduleRunEnd, AStartWhoseRecordFailedEndsTheRunRefused) {
  TestCore core = MowingRun();
  Phases(core);
  core.AbandonStart("os-session.json");
  EXPECT_TRUE(core.Idle());
  EXPECT_EQ(Ended(core), "ENDED REFUSED STORAGE_NOT_WRITABLE");
}

// The restart hold counts as an emergency: the tick at the end of the announcement cancels it before its check.
TEST(ScheduleAnnouncement, ARestartHoldCancelsItAtTheNextTick) {
  TestCore core = Docked();
  ASSERT_TRUE(core.StartRun(Plan(), Reason(), 0).empty());
  core.TakeEvents();
  core.TakeRuns();
  core.SetRestartHold(true);
  core.OnTick(kAnnounceS);
  const auto events = core.TakeEvents();
  const Event* canceled = Find(events, "SCHEDULE_START_CANCELED");
  ASSERT_TRUE(canceled);
  EXPECT_EQ(canceled->details["source"], "emergency");
  EXPECT_FALSE(Find(events, "MOWING_REFUSED"));
  EXPECT_EQ(Ended(core), "ENDED CANCELED emergency");
  EXPECT_FALSE(core.TakeDockCheck());
}

// The end of a run keeps the actions change of the step it ends in.
TEST(ScheduleRunEnd, TheEndKeepsTheActionsChangeOfItsStep) {
  TestCore core = Docked();
  ASSERT_TRUE(core.StartRun(Plan(), Reason(), 0).empty());
  core.TakeActionsChanged();
  core.OnTick(kAnnounceS);
  EXPECT_EQ(core.OnDockCheck(false, 0, 0, 999.0f, "no fix").code, "NO_POSITION");
  EXPECT_TRUE(core.TakeActionsChanged());
  EXPECT_EQ(Ended(core), "ENDED REFUSED NO_POSITION");
}

// What changed while the run left the station refuses mowing after UNDOCKED, and the run ends REFUSED with it.
TEST(ScheduleRunEnd, ARefusalAfterUndockingEndsItRefused) {
  TestCore core = Docked();
  ASSERT_TRUE(core.StartRun(Plan(), Reason(), 0).empty());
  core.OnTick(kAnnounceS);
  ASSERT_TRUE(core.OnDockCheck(true, kStartX, kDock.y, 0.02f, "").empty());
  core.OnChargeOff(true, "off");
  ASSERT_TRUE(core.TakeUndockPath());
  HardwareSnapshot rain = Healthy(false);
  rain.rain = true;
  core.OnHardwareStatus(rain);
  core.OnPose(PoseAt(kStartX + 0.75, kDock.y));
  core.TakeRuns();
  core.OnUndockEnded(LegEnd::kDone, Reason("DONE", ""));
  EXPECT_TRUE(core.Idle());
  EXPECT_EQ(Ended(core), "ENDED REFUSED RAIN");
}
