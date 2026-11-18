#include <cmath>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "mowbite_logic/mowing_part.h"

namespace mowbite_logic {

std::string MowingPart::ReadStartRequest(CoreFacts core, const json& params, StartRequest* request,
                                         std::string* field) const {
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
      if (core.mow_map_problem().empty() && !job_.HasArea(id)) {
        return fail("tasks", "tasks: " + id + " is no mowing area of the map");
      }
      asked.tasks.push_back(id);
    }
  }
  *request = std::move(asked);
  return "";
}

Reason MowingPart::RequestStart(CoreAccess core, StartRequest request, const Reason& node_problem,
                                CommandOutcome* out) {
  Reason why = core.StartProblem(request.tasks);
  if (why.empty()) why = node_problem;
  if (!why.empty()) {
    core.RefuseStart(why);
    return why;
  }
  start_request_ = std::move(request);
  // On the docking station the reference check comes first; its refusal is an event of its own.
  out->result["check"] = core.DockContact();
  if (core.DockContact()) {
    core.BeginDockCheck(DockCheck::kMowing);
  } else {
    core.StartMowing();
  }
  return {};
}

void MowingPart::PrepareStart(CoreAccess core) {
  // Another list of areas, or a restart, begins a new job; the planner learns of the dropped one by JOB_RESET.
  const StartRequest request = std::exchange(start_request_, StartRequest());
  if (request.restart || job_.TaskIds(request.tasks) != job_.TaskIds(job_.progress().tasks)) {
    if (!core.JobId().empty()) core.AddEvent("JOB_RESET", json::object());
    job_.Reset();
  }
  // A job not begun takes the list as asked: one that names every area does not grow with the map.
  if (!job_.Unfinished()) {
    job_.SetTasks(request.tasks);
    job_.SetAngle(request.angle_offset_deg);
  }
  job_.SetPassOver(request.pass_over);
  // Planned anew on every start, the map may have changed; the fingerprint keeps the place if the plan is the same.
  job_.Restore(job_.progress());
  leg_ = Leg::kNone;
  approached_ = false;
  planner_silent_ = 0;
  announced_area_.clear();
}

void MowingPart::ApplySkips(CoreAccess core) {
  // Its events carry the client of the skip, also after the leg stopped.
  const std::string client = core.client();
  if (core.client().empty()) core.client() = skip_client_;
  skipped_.reset();
  drive_on_ = false;
  if (skip_area_) {
    if (const JobArea* area = job_.CurrentArea()) {
      json details = {{"area_id", area->id}, {"area_name", area->name}};
      PutReason(&details, "AREA_SKIPPED", skip_why_);
      core.AddEvent("AREA_SKIPPED", std::move(details));
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
    core.CheckpointNow();
  }
  skip_area_ = skip_path_ = false;
  skip_why_ = Reason();
  skip_client_.clear();
  core.client() = client;
}

void MowingPart::Pause(CoreAccess core, int why, const Reason& reason) {
  if (core.state() != State::kMowing) return;
  const bool was_paused = pause_ != 0;
  for (size_t bit = 0; bit < pause_reasons_.size(); ++bit) {
    if ((why & (1 << bit)) && !(pause_ & (1 << bit))) pause_reasons_[bit] = reason;
  }
  pause_ |= why;
  if (why == kPauseGps) gps_good_since_.reset();
  if (LegDrives()) {
    core.CancelDrive(reason);
    core.effects().SetDue<StopRequest>(true);
  }
  spinup_since_.reset();
  if (was_paused) return;
  core.Notify("Mowing paused: " + reason.message);
  json details = {{"state", "PAUSED"}};
  PutReason(&details, "STATE", reason);
  core.AddEvent("STATE", std::move(details));
  core.effects().SetDue<ActionsChanged>(true);
  core.CheckpointNow();
}

const Reason& MowingPart::PauseReason() const {
  static const Reason kNone;
  // Bits of kPauseEmergency, kPauseFault, kPauseManual, kPauseObstacle, kPauseGps.
  for (const size_t bit : {1u, 3u, 0u, 4u, 2u}) {
    if (pause_ & (1 << bit)) return pause_reasons_[bit];
  }
  return kNone;
}

void MowingPart::Unpause(CoreAccess core, int why) {
  if (pause_ == 0) return;
  pause_ &= ~why;
  core.effects().SetDue<ActionsChanged>(true);
  if (pause_ != 0) return;
  // From where it stopped: the start of the path may be behind it now.
  approached_ = false;
  core.AddEvent("STATE", {{"state", "MOWING"}});
}

JobStep MowingPart::NextJobStep(CoreAccess core, double now_s) {
  const auto& pose_now = core.pose();
  JobStep step;
  if (core.state() != State::kMowing || pause_ != 0 || core.emergency() || leg_ != Leg::kNone) return step;
  if (!job_.HasPlan()) {
    std::string notice;
    std::vector<JobArea> passed;
    const bool found = job_.SeekArea(&notice, &passed);
    core.AreasPassedOver(passed);
    if (!found) {
      if (!notice.empty()) core.Notify(notice);
      core.EndJob(true, true);
      return step;
    }
    if (!notice.empty()) core.Notify(notice);
    const JobArea* area = job_.CurrentArea();
    if (area->id != announced_area_) {
      announced_area_ = area->id;
      core.AddEvent("AREA", {{"area_id", area->id}, {"area_name", area->name}});
    }
    leg_ = Leg::kPlanning;
    step.kind = JobStep::Kind::kPlan;
    step.area_id = area->id;
    return step;
  }
  const JobPose* goal = job_.CurrentPose();
  if (!goal) return step;
  if (!pose_now) {
    Pause(core, kPauseGps, Reason("NO_POSITION", "No position from the localization"));
    return step;
  }
  if (drive_on_) {
    drive_on_ = false;
    // Moved meanwhile: the way is asked for again from where the mower stands now.
    if (const JobPath* path = SkippedPathHere(core)) return SkippedPiece(core, *path);
  }
  if (!approached_ &&
      std::hypot(goal->x - pose_now->x, goal->y - pose_now->y) > core.config().approach_distance) {
    leg_ = Leg::kApproach;
    step.kind = JobStep::Kind::kApproach;
    step.goal = *goal;
    return step;
  }
  // The blade gets up to speed before a path that starts mowing, like in OpenMower.
  if (core.config().mow_with_blade && IsMowKind(goal->kind) && core.config().mower_spinup_rpm > 0 &&
      std::abs(core.hw().blade.rpm) < core.config().mower_spinup_rpm) {
    if (!spinup_since_) spinup_since_ = now_s;
    if (now_s - *spinup_since_ > core.config().mower_spinup_timeout) {
      core.AddEvent("MOW_MOTOR_SPINUP_FAILED", json::object());
      Pause(core, kPauseFault, Reason("BLADE_SPINUP_FAILED", "The blade did not reach its speed"));
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

void MowingPart::OnJobPlanned(CoreAccess core, std::vector<JobPath> paths, const std::string& fingerprint) {
  if (core.state() != State::kMowing || leg_ != Leg::kPlanning) return;
  leg_ = Leg::kNone;
  planner_silent_ = 0;
  job_.SetPlan(std::move(paths), fingerprint);
  approached_ = false;
  core.CheckpointNow();
}

void MowingPart::OnJobPlanFailed(CoreAccess core, const Reason& why) {
  if (core.state() != State::kMowing || leg_ != Leg::kPlanning) return;
  leg_ = Leg::kNone;
  const Reason pause("PLANNING_FAILED", "Planning failed: " + why.message);
  if (PlannerSilent(why)) {
    // A plan that timed out is not asked for again: the planner may still be at it.
    JobWithoutPlanner(core, why, pause, why.code != "PLANNER_TIMEOUT");
    return;
  }
  planner_silent_ = 0;
  Pause(core, kPauseFault, pause);
}

void MowingPart::JobWithoutPlanner(CoreAccess core, const Reason& silent, const Reason& pause, bool retry) {
  planner_silent_++;
  if (retry && planner_silent_ <= kPlannerRetries) {
    core.Notify("Mowing: no answer from the planner (" + silent.message + "), asking again: " +
           std::to_string(planner_silent_) + " of " + std::to_string(kPlannerRetries));
    return;
  }
  // Like a reason of HomeWatch: never from a pause that keeps its place (G11, G45).
  const bool keeps_place = (pause_ & kPauseKeepsPlace) != 0;
  if (!keeps_place && core.HomeReachable()) {
    core.EndJob(false, true, Reason(silent.code, "Going home: the planner does not answer"));
    return;
  }
  Pause(core, kPauseFault, pause);
}

void MowingPart::OnJobProgress(CoreFacts core, size_t index) {
  if (core.state() != State::kMowing || leg_ != Leg::kFollow) return;
  job_.OnIndex(index);
}

void MowingPart::OnLegEnded(CoreAccess core, LegEnd end, size_t index, const Reason& reason) {
  const auto& mower = core.pose();
  if (core.state() != State::kMowing || !LegDrives()) return;
  const bool follow = leg_ == Leg::kFollow;
  const bool approach = leg_ == Leg::kApproach;
  leg_ = Leg::kNone;
  if (follow) job_.OnIndex(index);
  // A piece of the path a skip left: the job stays at the start of the next path.
  if (!follow && !approach && skipped_) skipped_->index = static_cast<int>(index);
  if (skip_area_ || skip_path_) {
    ApplySkips(core);
    return;
  }
  if (approach && PlannerSilent(reason)) {
    approached_ = false;
    JobWithoutPlanner(core, reason, Reason("APPROACH_FAILED", reason.message), true);
    return;
  }
  // An approach the logic stopped itself tells nothing about the planner.
  if (approach && end != LegEnd::kCanceled) planner_silent_ = 0;
  switch (end) {
    case LegEnd::kDone:
      if (follow) {
        job_.PathDone();
        approached_ = false;
        core.CheckpointNow();
      } else if (approach) {
        approached_ = true;
      }
      break;
    case LegEnd::kCanceled: approached_ = false; break;
    case LegEnd::kGps:
      // The follower's text carries no numbers: the event gets the accuracy and the limit like a refused start.
      if (reason.code == "POSITION_ACCURACY" && mower &&
          !(mower->position_accuracy <= core.config().max_position_accuracy)) {
        Pause(core, kPauseGps,
              core.AccuracyReason(mower->position_accuracy, core.config().max_position_accuracy));
      } else {
        Pause(core, kPauseGps, reason);
      }
      break;
    case LegEnd::kEmergency: Pause(core, kPauseEmergency, reason); break;
    case LegEnd::kNoWay:
      // After a skip the path left is a checked way on: a piece of it, then the approach is asked for again.
      if (approach && SkippedPathHere(core)) {
        approached_ = false;
        drive_on_ = true;
        break;
      }
      [[fallthrough]];
    case LegEnd::kFailed:
      approached_ = false;
      Pause(core, kPauseFault, reason);
      break;
  }
  if (follow) core.CheckpointNow();
}

bool MowingPart::LegDrives() const {
  return leg_ == Leg::kApproach || leg_ == Leg::kFollow || leg_ == Leg::kSkippedPath;
}

const JobPath* MowingPart::SkippedPathHere(CoreFacts core) const {
  const auto& mower = core.pose();
  if (!skipped_ || !mower || !job_.HasPlan()) return nullptr;
  const JobProgress& now = job_.progress();
  if (now.area != skipped_->area || now.fingerprint != skipped_->fingerprint || now.path != skipped_->path + 1 ||
      now.index != 0) {
    return nullptr;
  }
  const JobPath* path = job_.Path(skipped_->path);
  if (!path || skipped_->index < 0 || static_cast<size_t>(skipped_->index) + 1 >= path->poses.size()) return nullptr;
  const JobPose& at = path->poses[static_cast<size_t>(skipped_->index)];
  return std::hypot(at.x - mower->x, at.y - mower->y) <= core.config().approach_distance ? path : nullptr;
}

JobStep MowingPart::SkippedPiece(CoreAccess core, const JobPath& path) {
  const std::vector<JobPose>& poses = path.poses;
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access): the caller got path from SkippedPathHere, which checks skipped_
  const size_t from = static_cast<size_t>(skipped_->index);
  size_t to = from + 1;
  double length = std::hypot(poses[to].x - poses[from].x, poses[to].y - poses[from].y);
  while (to + 1 < poses.size()) {
    length += std::hypot(poses[to + 1].x - poses[to].x, poses[to + 1].y - poses[to].y);
    if (length > kSkippedPathPiece) break;
    ++to;
  }
  core.Notify(
      "Mowing: no way to the next path from here, driving on along the skipped one with the blade off, to pose " +
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

void MowingPart::Enter(CoreAccess core) {
  if (job_.progress().job_id.empty()) job_.SetJobId(core.NewId());
  session_id_ = core.NewId();
}

void MowingPart::Leave(CoreAccess core) {
  session_id_.clear();
  if (LegDrives()) core.CancelDrive(Reason("STOPPED", "Mowing ended"));
  leg_ = Leg::kNone;
  pause_ = 0;
  skip_path_ = skip_area_ = drive_on_ = false;
  skipped_.reset();
  spinup_since_.reset();
  gps_good_since_.reset();
}

void MowingPart::TickGpsWait(CoreAccess core, double now_s) {
  if (pause_ & kPauseGps) {
    if (!core.PoseGood()) {
      gps_good_since_.reset();
    } else if (!gps_good_since_) {
      gps_good_since_ = now_s;
    } else if (now_s - *gps_good_since_ >= core.config().gps_wait_time) {
      gps_good_since_.reset();
      Unpause(core, kPauseGps);
    }
  }
}

void MowingPart::TickCheckpoint(CoreAccess core, double now_s) {
  if (leg_ == Leg::kFollow && now_s - last_checkpoint_s_ >= core.config().checkpoint_period) {
    last_checkpoint_s_ = now_s;
    core.CheckpointNow();
  }
}

}  // namespace mowbite_logic
