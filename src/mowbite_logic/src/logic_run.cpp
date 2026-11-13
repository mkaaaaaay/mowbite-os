#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "mowbite_logic/logic_core.h"
#include "mowbite_time/sun.h"

namespace mowbite_logic {

namespace {

bool Contains(const std::vector<std::string>& list, const std::string& id) {
  return std::find(list.begin(), list.end(), id) != list.end();
}

// The first id that comes twice, empty if none does.
std::string Twice(const std::vector<std::string>& ids) {
  std::set<std::string> seen;
  for (const auto& id : ids) {
    if (!seen.insert(id).second) return id;
  }
  return "";
}

Reason SchedulePause() {
  return Reason("SCHEDULE", "the areas are paused in the schedule").With("detail", "pause");
}

}  // namespace

bool StartBlocksHold(int64_t next_start, int64_t now) {
  return next_start != 0 && next_start - now <= kHoldBeforeStartS;
}

std::string LogicCore::InvalidRun(const ScheduleRun& run) const {
  if (run.run_id.empty()) return "run_id is empty";
  if (const std::string id = Twice(run.area_ids); !id.empty()) return "area_ids: " + id + " is in the list twice";
  if (const std::string id = Twice(run.pass_over); !id.empty()) return "pass_over: " + id + " is in the list twice";
  if (run.end_in_s < 0) return "end_in_s is below 0";
  if (run.has_angle && !std::isfinite(run.angle_offset_deg)) return "angle_offset_deg is no number";
  if (run.area_ids.empty()) return "";
  for (const auto& id : run.pass_over) {
    if (!Contains(run.area_ids, id)) return "pass_over: " + id + " is not in area_ids";
  }
  // Unknown ids only in pass_over, never all areas in place of them; without a map the start is refused for that.
  for (const auto& id : run.area_ids) {
    if (mow_map_problem_.empty() && !Contains(run.pass_over, id) && !job_.HasArea(id)) {
      return "area_ids: " + id + " is no mowing area of the map";
    }
  }
  return "";
}

Reason LogicCore::RequestMowProblem(const StartRequest& request) const {
  if (request.pass_over.empty()) return MowProblem(request.tasks);
  std::vector<std::string> left;
  for (const auto& id : job_.TaskIds(request.tasks)) {
    if (!Contains(request.pass_over, id)) left.push_back(id);
  }
  if (!left.empty()) return MowProblem(left);
  const Reason why = MowProblem(request.tasks);
  if (!why.empty()) return why;
  return Reason("NO_MOWING_AREA", "Every area of the plan is paused or not on the map");
}

Reason LogicCore::ScheduleProblem(const StartRequest& request, const ScheduleRun& run, double now_s) const {
  Reason why = Busy();
  if (why.empty()) why = HoldProblem();
  if (!why.empty()) return why;
  if (!DockContact()) {
    return Reason("NOT_DOCKED", "The mower is not on the docking station: a plan of the schedule starts only there");
  }
  if (!clock_safe_) {
    const std::string text = "The clock is not safe (" + clock_reason_ + "): a plan of the schedule does not start";
    return Reason("CLOCK_UNSAFE", text).With("clock_reason", clock_reason_);
  }
  why = RequestMowProblem(request);
  if (!why.empty() || run.dark_allowed) return why;
  if (!datum_) return Reason("NO_POSITION", "No datum for the sun: this plan does not mow in the dark");
  const double elevation = mowbite_time::SunElevation(datum_->first, datum_->second, now_s + clock_offset_);
  if (elevation < mowbite_time::kSunsetElevation) {
    return Reason("DARK", "The sun is down: this plan does not mow in the dark")
        .With("sun_elevation", elevation)
        .With("limit", mowbite_time::kSunsetElevation);
  }
  return {};
}

Reason LogicCore::StartRun(const ScheduleRun& run, const Reason& node_problem, double now_s) {
  if (const std::string invalid = InvalidRun(run); !invalid.empty()) {
    Notify("Start of the schedule " + run.run_id + " refused: " + invalid);
    return Reason("INVALID", invalid);
  }
  const StartRequest request{run.area_ids, run.restart, run.pass_over};
  Reason why;
  if (run_) why = Reason("BUSY", "The mower is busy: a start of the schedule is announced").With("state", "announcing");
  if (why.empty()) why = ScheduleProblem(request, run, now_s);
  if (why.empty()) why = node_problem;
  if (!why.empty()) {
    json details = {{"source", "schedule"}};
    PutReason(&details, "MOWING_REFUSED", why);
    AddEvent("MOWING_REFUSED", std::move(details));
    return why;
  }
  run_ = Run{run, RunPhase::kNone, now_s + kAnnounceS, std::nullopt, false, "", ""};
  if (run.end_in_s > 0) run_->end_due = now_s + static_cast<double>(run.end_in_s);
  SetRunPhase(RunPhase::kAnnouncing);
  AddEvent("SCHEDULE_START", {{"seconds", static_cast<int>(kAnnounceS)},
                              {"until", static_cast<int64_t>(std::llround(now_s + clock_offset_ + kAnnounceS))},
                              {"severity", "info"},
                              {"resume", "auto"}});
  Notify("Schedule: " + run.run_id + " starts in " + std::to_string(static_cast<int>(kAnnounceS)) + " s");
  SetDue<ActionsChanged>(true);
  return {};
}

void LogicCore::EndAnnouncement(double now_s) {
  StartRequest request{run_->request.area_ids, run_->request.restart, run_->request.pass_over};
  const Reason why = ScheduleProblem(request, run_->request, now_s);
  if (!why.empty()) {
    RefuseStart(why);
    RunEnds("REFUSED", why.code);
    FinishRun();
    return;
  }
  start_request_ = std::move(request);
  SetRunPhase(RunPhase::kChecking);
  BeginDockCheck(DockCheck::kMowing);
}

void LogicCore::WatchAnnouncement() {
  if (!run_ || run_->phase != RunPhase::kAnnouncing) return;
  if (emergency_) {
    CancelAnnouncement("emergency");
  } else if (Held()) {
    CancelAnnouncement("hold");
  } else if (!DockContact()) {
    CancelAnnouncement("undocked");
  }
}

void LogicCore::CommandCancels() {
  if (run_ && run_->phase == RunPhase::kAnnouncing) CancelAnnouncement("rpc");
}

void LogicCore::CancelAnnouncement(const std::string& source) {
  AddEvent("SCHEDULE_START_CANCELED", {{"source", source}, {"severity", "info"}, {"resume", "auto"}});
  Notify("Schedule: the start of " + run_->request.run_id + " was canceled (" + source + ")");
  RunEnds("CANCELED", source);
  FinishRun();
}

void LogicCore::SetRunPhase(RunPhase phase) {
  if (!run_ || run_->phase == phase) return;
  run_->phase = phase;
  effects_.push_back(RunChanged{run_->request.run_id, phase, "", ""});
}

void LogicCore::RunEnds(const std::string& code, const std::string& detail) {
  if (!run_ || !run_->end_code.empty()) return;
  run_->end_code = code;
  run_->detail = detail;
}

void LogicCore::FinishRun() {
  if (!run_) return;
  if (run_->end_code.empty()) run_->end_code = emergency_ ? "EMERGENCY" : "STOPPED";
  effects_.push_back(RunChanged{run_->request.run_id, RunPhase::kEnded, run_->end_code, run_->detail});
  run_.reset();
  SetDue<ActionsChanged>(true);
}

std::optional<double> LogicCore::PlanAngle() const {
  if (!run_ || !run_->request.has_angle) return std::nullopt;
  return run_->request.angle_offset_deg;
}

bool LogicCore::SunsetWanted() const {
  if (!run_) return sunset_on_;
  return !run_->request.dark_allowed || run_->request.until_sunset;
}

Reason LogicCore::DeadlineDue(double now_s) {
  if (sunset_due_ && now_s >= *sunset_due_) return DarkWhy(now_s);
  if (!run_ || !run_->end_due || now_s < *run_->end_due) return {};
  run_->end_passed = true;
  return Reason("SCHEDULE", "the time of the plan is over").With("detail", "end");
}

Reason LogicCore::DropTasks(const std::string& run_id, const std::vector<std::string>& area_ids) {
  if (!run_ || run_->request.run_id != run_id) {
    return Reason("NOT_ALLOWED", "No run " + run_id + " of the schedule: a run by hand is not the schedule's");
  }
  if (state_ != State::kMowing || run_->phase != RunPhase::kMowing) return WrongState();
  job_.PassOver(area_ids.empty() ? job_.TaskIds(job_.progress().tasks) : area_ids);
  if (!job_.AreaLeft()) {
    Reason why = SchedulePause();
    // Like a reason of HomeWatch: never from a pause that waits for the user (G11); without a way home idle there.
    const bool waits_for_user = (pause_ & (kPauseManual | kPauseEmergency | kPauseFault)) != 0;
    const bool home = !waits_for_user && HomeReachable();
    why.message = (home ? "Going home: " : "Mowing ends: ") + why.message;
    Notify("Schedule: " + why.message);
    EndJob(false, home, why);
    return {};
  }
  const JobArea* area = job_.CurrentArea();
  if (!area || !job_.PassedOver(area->id)) return {};
  skip_area_ = true;
  skip_why_ = SchedulePause();
  if (LegDrives()) {
    CancelDrive(Reason("STOPPED", "The area is paused in the schedule"));
    SetDue<StopRequest>(true);
  } else {
    ApplySkips();
  }
  return {};
}

void LogicCore::AreasPassedOver(const std::vector<JobArea>& areas) {
  for (const auto& area : areas) {
    json details = {{"area_id", area.id}, {"area_name", area.name}};
    PutReason(&details, "AREA_SKIPPED", SchedulePause());
    AddEvent("AREA_SKIPPED", std::move(details));
  }
}

}  // namespace mowbite_logic
