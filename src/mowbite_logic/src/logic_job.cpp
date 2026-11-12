#include "mowbite_logic/logic_core.h"

#include <cmath>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace mowbite_logic {

std::string LogicCore::ReadStartRequest(const json& params, StartRequest* request, std::string* field) const {
  const auto fail = [field](const char* name, const std::string& text) {
    *field = name;
    return text;
  };
  // Never all areas in place of a list the OS cannot take.
  if (params.contains("areas")) return fail("areas", "areas: not taken, tasks names the areas of the job");
  StartRequest asked;
  asked.restart = params.value("restart", false);
  if (params.contains("tasks")) {
    if (params["tasks"].empty()) return fail("tasks", "tasks: the list is empty");
    std::set<std::string> seen;
    for (const json& task : params["tasks"]) {
      const std::string id = task["area_id"].get<std::string>();
      if (!seen.insert(id).second) return fail("tasks", "tasks: the area " + id + " is in the list twice");
      // Without a map yet the start is refused for that.
      if (mow_map_problem_.empty() && !job_.HasArea(id)) {
        return fail("tasks", "tasks: " + id + " is no mowing area of the map");
      }
      asked.tasks.push_back(id);
    }
  }
  if (request) *request = std::move(asked);
  return "";
}

Reason LogicCore::RequestStart(const json& params, const Reason& node_problem, CommandOutcome* out) {
  StartRequest request;
  std::string field;
  ReadStartRequest(params, &request, &field);
  Reason why = StartProblem(request.tasks);
  if (why.empty()) why = node_problem;
  if (!why.empty()) {
    RefuseStart(why);
    return why;
  }
  start_request_ = std::move(request);
  // On the docking station the reference check comes first; its refusal is an event of its own.
  out->result["check"] = DockContact();
  if (DockContact()) {
    BeginDockCheck(DockCheck::kMowing);
  } else {
    StartMowing();
  }
  return {};
}

void LogicCore::StartMowing() {
  // Another list of areas, or a restart, begins a new job; the planner learns of the dropped one by JOB_RESET.
  const StartRequest request = std::exchange(start_request_, StartRequest());
  if (request.restart || job_.TaskIds(request.tasks) != job_.TaskIds(job_.progress().tasks)) {
    if (!JobId().empty()) AddEvent("JOB_RESET", json::object());
    job_.Reset();
    job_.SetTasks(request.tasks);
  }
  // Planned anew on every start, the map may have changed; the fingerprint keeps the place if the plan is the same.
  job_.Restore(job_.progress());
  leg_ = Leg::kNone;
  approached_ = false;
  planner_silent_ = 0;
  announced_area_.clear();
  SetState(State::kMowing);
  CheckpointNow();
}

void LogicCore::ApplySkips() {
  // Its events carry the client of the skip, also after the leg stopped.
  const std::string client = client_;
  if (client_.empty()) client_ = skip_client_;
  skipped_.reset();
  drive_on_ = false;
  if (skip_area_) {
    if (const JobArea* area = job_.CurrentArea()) {
      json details = {{"area_id", area->id}, {"area_name", area->name}};
      PutReason(&details, "AREA_SKIPPED", Reason("MANUAL", "Skipped by the app"));
      AddEvent("AREA_SKIPPED", std::move(details));
    }
    job_.SkipArea();
    // A plan on its way is for the skipped area.
    if (leg_ == Leg::kPlanning) leg_ = Leg::kNone;
  } else if (skip_path_ && job_.HasPlan()) {
    const JobProgress left = job_.progress();
    job_.SkipPath();
    // Still in its plan: the path left is a checked way on where the planner finds none (OnLegEnded).
    if (job_.HasPlan()) skipped_ = left;
  }
  if (skip_area_ || skip_path_) {
    approached_ = false;
    spinup_since_.reset();
    CheckpointNow();
  }
  skip_area_ = skip_path_ = false;
  skip_client_.clear();
  client_ = client;
}

void LogicCore::Pause(int why, const Reason& reason) {
  if (state_ != State::kMowing) return;
  const bool was_paused = pause_ != 0;
  pause_ |= why;
  if (why == kPauseGps) gps_good_since_.reset();
  if (LegDrives()) {
    CancelDrive(reason);
    SetDue<StopRequest>(true);
  }
  spinup_since_.reset();
  if (was_paused) return;
  pause_reason_ = reason;
  Notify("Mowing paused: " + reason.message);
  json details = {{"state", "PAUSED"}};
  PutReason(&details, "STATE", reason);
  AddEvent("STATE", std::move(details));
  SetDue<ActionsChanged>(true);
  CheckpointNow();
}

void LogicCore::Unpause(int why) {
  if (pause_ == 0) return;
  pause_ &= ~why;
  SetDue<ActionsChanged>(true);
  if (pause_ != 0) return;
  pause_reason_ = Reason();
  // From where it stopped: the start of the path may be behind it now.
  approached_ = false;
  AddEvent("STATE", {{"state", "MOWING"}});
}

void LogicCore::EndJob(bool finished, bool home, const Reason& why) {
  if (state_ != State::kMowing) return;
  if (finished) {
    AddEvent("JOB_COMPLETE", json::object());
    job_.Complete();
  }
  if (home) {
    GoHome(why);
  } else {
    SetState(State::kIdle);
  }
  CheckpointNow();
}

JobStep LogicCore::NextJobStep(double now_s) {
  JobStep step;
  if (state_ != State::kMowing || pause_ != 0 || emergency_ || leg_ != Leg::kNone) return step;
  if (!job_.HasPlan()) {
    std::string notice;
    if (!job_.SeekArea(&notice)) {
      if (!notice.empty()) Notify(notice);
      EndJob(true, true);
      return step;
    }
    if (!notice.empty()) Notify(notice);
    const JobArea* area = job_.CurrentArea();
    if (area->id != announced_area_) {
      announced_area_ = area->id;
      AddEvent("AREA", {{"area_id", area->id}, {"area_name", area->name}});
    }
    leg_ = Leg::kPlanning;
    step.kind = JobStep::Kind::kPlan;
    step.area_id = area->id;
    return step;
  }
  const JobPose* goal = job_.CurrentPose();
  if (!goal) return step;
  if (!pose_) {
    Pause(kPauseGps, Reason("NO_POSITION", "No position from the localization"));
    return step;
  }
  if (drive_on_) {
    drive_on_ = false;
    // Moved meanwhile: the way is asked for again from where the mower stands now.
    if (const JobPath* path = SkippedPathHere()) return SkippedPiece(*path);
  }
  if (!approached_ && std::hypot(goal->x - pose_->x, goal->y - pose_->y) > config_.approach_distance) {
    leg_ = Leg::kApproach;
    step.kind = JobStep::Kind::kApproach;
    step.goal = *goal;
    return step;
  }
  // The blade gets up to speed before a path that starts mowing, like in OpenMower.
  if (config_.mow_with_blade && IsMowKind(goal->kind) && config_.mower_spinup_rpm > 0 &&
      std::abs(hw_.blade.rpm) < config_.mower_spinup_rpm) {
    if (!spinup_since_) spinup_since_ = now_s;
    if (now_s - *spinup_since_ > config_.mower_spinup_timeout) {
      AddEvent("MOW_MOTOR_SPINUP_FAILED", json::object());
      Pause(kPauseFault, Reason("BLADE_SPINUP_FAILED", "The blade did not reach its speed"));
    }
    return step;
  }
  spinup_since_.reset();
  leg_ = Leg::kFollow;
  step.kind = JobStep::Kind::kFollow;
  step.path = *job_.CurrentPath();
  step.start_index = static_cast<size_t>(job_.progress().index);
  return step;
}

void LogicCore::OnJobPlanned(std::vector<JobPath> paths, const std::string& fingerprint) {
  if (state_ != State::kMowing || leg_ != Leg::kPlanning) return;
  leg_ = Leg::kNone;
  planner_silent_ = 0;
  job_.SetPlan(std::move(paths), fingerprint);
  approached_ = false;
  CheckpointNow();
}

void LogicCore::OnJobPlanFailed(const Reason& why) {
  if (state_ != State::kMowing || leg_ != Leg::kPlanning) return;
  leg_ = Leg::kNone;
  const Reason pause("PLANNING_FAILED", "Planning failed: " + why.message);
  if (PlannerSilent(why)) {
    // A plan that timed out is not asked for again: the planner may still be at it.
    JobWithoutPlanner(why, pause, why.code != "PLANNER_TIMEOUT");
    return;
  }
  planner_silent_ = 0;
  Pause(kPauseFault, pause);
}

void LogicCore::JobWithoutPlanner(const Reason& silent, const Reason& pause, bool retry) {
  planner_silent_++;
  if (retry && planner_silent_ <= kPlannerRetries) {
    Notify("Mowing: no answer from the planner (" + silent.message + "), asking again: " +
           std::to_string(planner_silent_) + " of " + std::to_string(kPlannerRetries));
    return;
  }
  // Like a reason of HomeWatch: never from a pause that waits for the user, who may stand at the mower (G11).
  const bool waits_for_user = (pause_ & (kPauseManual | kPauseEmergency | kPauseFault)) != 0;
  if (!waits_for_user && HomeReachable()) {
    EndJob(false, true, Reason(silent.code, "Going home: the planner does not answer"));
    return;
  }
  Pause(kPauseFault, pause);
}

void LogicCore::OnJobProgress(size_t index) {
  if (state_ != State::kMowing || leg_ != Leg::kFollow) return;
  job_.OnIndex(index);
}

void LogicCore::OnLegEnded(LegEnd end, size_t index, const Reason& reason) {
  if (state_ != State::kMowing || !LegDrives()) return;
  const bool follow = leg_ == Leg::kFollow;
  const bool approach = leg_ == Leg::kApproach;
  leg_ = Leg::kNone;
  if (follow) job_.OnIndex(index);
  // A piece of the path a skip left: the job stays at the start of the next path.
  if (!follow && !approach && skipped_) skipped_->index = static_cast<int>(index);
  if (skip_area_ || skip_path_) {
    ApplySkips();
    return;
  }
  if (approach && PlannerSilent(reason)) {
    approached_ = false;
    JobWithoutPlanner(reason, Reason("APPROACH_FAILED", reason.message), true);
    return;
  }
  // An approach the logic stopped itself tells nothing about the planner.
  if (approach && end != LegEnd::kCanceled) planner_silent_ = 0;
  switch (end) {
    case LegEnd::kDone:
      if (follow) {
        job_.PathDone();
        approached_ = false;
        CheckpointNow();
      } else if (approach) {
        approached_ = true;
      }
      break;
    case LegEnd::kCanceled: approached_ = false; break;
    case LegEnd::kGps:
      // The follower's text carries no numbers: the event gets the accuracy and the limit like a refused start.
      if (reason.code == "POSITION_ACCURACY" && pose_ && !(pose_->position_accuracy <= config_.max_position_accuracy)) {
        Pause(kPauseGps, AccuracyReason(pose_->position_accuracy, config_.max_position_accuracy));
      } else {
        Pause(kPauseGps, reason);
      }
      break;
    case LegEnd::kEmergency: Pause(kPauseEmergency, reason); break;
    case LegEnd::kNoWay:
      // After a skip the path left is a checked way on: a piece of it, then the approach is asked for again.
      if (approach && SkippedPathHere()) {
        approached_ = false;
        drive_on_ = true;
        break;
      }
      [[fallthrough]];
    case LegEnd::kFailed:
      approached_ = false;
      Pause(kPauseFault, reason);
      break;
  }
  if (follow) CheckpointNow();
}

bool LogicCore::LegDrives() const {
  return leg_ == Leg::kApproach || leg_ == Leg::kFollow || leg_ == Leg::kSkippedPath;
}

const JobPath* LogicCore::SkippedPathHere() const {
  if (!skipped_ || !pose_ || !job_.HasPlan()) return nullptr;
  const JobProgress& now = job_.progress();
  if (now.area != skipped_->area || now.fingerprint != skipped_->fingerprint || now.path != skipped_->path + 1 ||
      now.index != 0) {
    return nullptr;
  }
  const JobPath* path = job_.Path(skipped_->path);
  if (!path || skipped_->index < 0 || static_cast<size_t>(skipped_->index) + 1 >= path->poses.size()) return nullptr;
  const JobPose& at = path->poses[static_cast<size_t>(skipped_->index)];
  return std::hypot(at.x - pose_->x, at.y - pose_->y) <= config_.approach_distance ? path : nullptr;
}

JobStep LogicCore::SkippedPiece(const JobPath& path) {
  const std::vector<JobPose>& poses = path.poses;
  const size_t from = static_cast<size_t>(skipped_->index);
  size_t to = from + 1;
  double length = std::hypot(poses[to].x - poses[from].x, poses[to].y - poses[from].y);
  while (to + 1 < poses.size()) {
    length += std::hypot(poses[to + 1].x - poses[to].x, poses[to + 1].y - poses[to].y);
    if (length > kSkippedPathPiece) break;
    ++to;
  }
  Notify("Mowing: no way to the next path from here, driving on along the skipped one with the blade off, to pose " +
         std::to_string(to));
  // Off also where the blade was spinning up for the next path.
  spinup_since_.reset();
  leg_ = Leg::kSkippedPath;
  JobStep step;
  step.kind = JobStep::Kind::kSkippedPath;
  step.path.poses.assign(poses.begin(), poses.begin() + static_cast<std::ptrdiff_t>(to) + 1);
  for (JobPose& pose : step.path.poses) pose.kind = kSegmentTransit;
  step.start_index = from;
  return step;
}

}  // namespace mowbite_logic
