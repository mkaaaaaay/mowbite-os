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
    if (mow_map_problem_.empty() && !Contains(run.pass_over, id) && !mowing_.job().HasArea(id)) {
      return "area_ids: " + id + " is no mowing area of the map";
    }
  }
  return "";
}

Reason LogicCore::RequestMowProblem(const StartRequest& request) const {
  if (request.pass_over.empty()) return MowProblem(request.tasks);
  std::vector<std::string> left;
  for (const auto& id : mowing_.job().TaskIds(request.tasks)) {
    if (!Contains(request.pass_over, id)) left.push_back(id);
  }
  if (!left.empty()) return MowProblem(left);
  Reason why = MowProblem(request.tasks);
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
  why = obstacle_part_.StartProblem(LeavesForwards());
  if (!why.empty()) return why;
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

void LogicCore::EndAnnouncement(double now_s) {
  auto& active = schedule_.run();
  if (!active) return;
  StartRequest request = SchedulePart::RunRequest(active->request);
  const Reason why = ScheduleProblem(request, active->request, now_s);
  if (!why.empty()) {
    RefuseStart(why);
    RunEnds("REFUSED", why.code);
    FinishRun();
    return;
  }
  mowing_.start_request() = std::move(request);
  SetRunPhase(RunPhase::kChecking);
  BeginDockCheck(DockCheck::kMowing);
}

std::optional<double> LogicCore::PlanAngle() const {
  return mowing_.job().progress().angle_offset_deg;
}

bool LogicCore::SunsetWanted() const {
  auto& active = schedule_.run();
  if (!active) return home_.sunset_on();
  return !active->request.dark_allowed || active->request.until_sunset;
}

Reason LogicCore::DeadlineDue(double now_s) {
  auto& active = schedule_.run();
  const auto& due = home_.sunset_due();
  if (due && now_s >= *due) return DarkWhy(now_s);
  if (!active || !active->end_due || now_s < *active->end_due) return {};
  active->end_passed = true;
  return Reason("SCHEDULE", "the time of the plan is over").With("detail", "end");
}

Reason LogicCore::DropTasks(const std::string& run_id, const std::vector<std::string>& area_ids) {
  auto& active = schedule_.run();
  if (!active || active->request.run_id != run_id) {
    return Reason("NOT_ALLOWED", "No run " + run_id + " of the schedule: a run by hand is not the schedule's");
  }
  if (state_ != State::kMowing || active->phase != RunPhase::kMowing) return WrongState();
  mowing_.job().PassOver(area_ids.empty() ? mowing_.job().TaskIds(mowing_.job().progress().tasks) : area_ids);
  if (!mowing_.job().AreaLeft()) {
    Reason why = SchedulePause();
    // Like a reason of HomeWatch: never from a pause that keeps its place (G11); without a way home idle there.
    const bool keeps_place = (mowing_.pause() & kPauseKeepsPlace) != 0;
    const bool home = !keeps_place && HomeReachable();
    why.message = (home ? "Going home: " : "Mowing ends: ") + why.message;
    Notify("Schedule: " + why.message);
    EndJob(false, home, why);
    return {};
  }
  const JobArea* area = mowing_.job().CurrentArea();
  if (!area || !mowing_.job().PassedOver(area->id)) return {};
  mowing_.skip_area() = true;
  mowing_.skip_why() = SchedulePause();
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
