#include <algorithm>
#include <cmath>
#include <cstdio>
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
  if (why.empty() && !core.obstacle_part().WaiveForStart(core.LeavesForwards())) {
    why = core.obstacle_part().StartProblem(core.LeavesForwards())
              .WithMessage("An obstacle in view, or one the sensors report no more, is in the way");
  }
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
  cut_.reset();
  cutting_added_.clear();
  recut_ = detour_ = false;
  cut_retries_ = 0;
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
    // A plan or cut on its way is for the skipped area.
    if (leg_ == Leg::kPlanning || leg_ == Leg::kCutting) leg_ = Leg::kNone;
    cut_.reset();
    detour_ = false;
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
  if (core.state() != State::kMowing || core.emergency() || leg_ != Leg::kNone) return step;
  if (core.obstacle_part().TakeCirclesGone()) recut_ = true;
  // Objects alone that stood avoid_wait s in a hold: the mower drives around them where the planner cuts the plan.
  if (pause_ == kPauseObstacle && pose_now && job_.CurrentPath()) {
    std::vector<AvoidCircle> due = core.obstacle_part().AvoidDue(now_s);
    if (!due.empty()) return CutStep(core, std::move(due));
  }
  if (pause_ != 0) return step;
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
    spinup_since_.reset();
    leg_ = Leg::kPlanning;
    step.kind = JobStep::Kind::kPlan;
    step.area_id = area->id;
    return step;
  }
  if (!job_.CurrentPose()) return step;
  if (!pose_now) {
    Pause(core, kPauseGps, Reason("NO_POSITION", "No position from the localization"));
    return step;
  }
  if (recut_) {
    recut_ = false;
    if (!core.obstacle_part().circles().empty()) return CutStep(core, {});
    // No circle left: the plan as it is from where the job stands.
    if (cut_) {
      cut_.reset();
      detour_ = false;
      approached_ = false;
    }
  }
  if (cut_) {
    FollowCut();
    // The rest of the area was left out: on with the next one.
    if (!job_.HasPlan()) return NextJobStep(core, now_s);
  }
  const JobPose* goal = job_.CurrentPose();
  if (drive_on_) {
    drive_on_ = false;
    // Moved meanwhile: the way is asked for again from where the mower stands now.
    if (const JobPath* path = SkippedPathHere(core)) return SkippedPiece(core, *path);
  }
  if (detour_) {
    if (!BladeReady(core, now_s)) return step;
    leg_ = Leg::kDetour;
    step.kind = JobStep::Kind::kDetour;
    step.goal = *goal;
    step.area_id = job_.CurrentArea()->id;
    return step;
  }
  if (!approached_ &&
      std::hypot(goal->x - pose_now->x, goal->y - pose_now->y) > core.config().approach_distance) {
    // No blade on the way to the start of the path: a spin up begun before counts no more.
    spinup_since_.reset();
    leg_ = Leg::kApproach;
    step.kind = JobStep::Kind::kApproach;
    step.goal = *goal;
    return step;
  }
  // The blade gets up to speed before a path that starts mowing, like in OpenMower.
  if (IsMowKind(goal->kind) && !BladeReady(core, now_s)) return step;
  leg_ = Leg::kFollow;
  step.kind = JobStep::Kind::kFollow;
  step.path = *job_.CurrentPath();
  // A piece of a cut ends where it does.
  if (cut_ && !cut_->empty() && cut_->front().path == job_.progress().path &&
      cut_->front().last + 1 < step.path.poses.size()) {
    step.path.poses.resize(cut_->front().last + 1);
  }
  step.start_index = static_cast<size_t>(job_.progress().index);
  return step;
}

bool MowingPart::BladeReady(CoreAccess core, double now_s) {
  if (!core.config().mow_with_blade || core.config().mower_spinup_rpm <= 0 ||
      std::abs(core.hw().blade.rpm) >= core.config().mower_spinup_rpm) {
    spinup_since_.reset();
    return true;
  }
  if (!spinup_since_) spinup_since_ = now_s;
  if (now_s - *spinup_since_ > core.config().mower_spinup_timeout) {
    core.AddEvent("MOW_MOTOR_SPINUP_FAILED", json::object());
    Pause(core, kPauseFault, Reason("BLADE_SPINUP_FAILED", "The blade did not reach its speed"));
  }
  return false;
}

JobStep MowingPart::CutStep(CoreAccess core, std::vector<AvoidCircle> added) {
  spinup_since_.reset();
  JobStep step;
  step.kind = JobStep::Kind::kCut;
  step.area_id = job_.CurrentArea()->id;
  for (int p = job_.progress().path; const JobPath* path = job_.Path(p); ++p) step.paths.push_back(*path);
  step.start_index = static_cast<size_t>(job_.progress().index);
  step.circles = core.obstacle_part().circles();
  step.circles.insert(step.circles.end(), added.begin(), added.end());
  cutting_added_ = std::move(added);
  cutting_at_ = job_.progress();
  leg_ = Leg::kCutting;
  return step;
}

void MowingPart::OnCutAnswer(CoreAccess core, const CutAnswer& given) {
  if (core.state() != State::kMowing || leg_ != Leg::kCutting) return;
  leg_ = Leg::kNone;
  CutAnswer answer = given;
  for (JobPiece& piece : answer.pieces) piece.path += cutting_at_.path;
  if (answer.ok && !PiecesFit(answer.pieces)) {
    answer.ok = false;
    answer.code = "INTERNAL";
  }
  const std::vector<AvoidCircle> added = std::exchange(cutting_added_, {});
  const bool around = !added.empty();
  const JobProgress& at = job_.progress();
  // Another plan, area or pause meanwhile: a cut is asked for again where it is still wanted.
  if (!job_.HasPlan() || at.area != cutting_at_.area || at.fingerprint != cutting_at_.fingerprint) {
    if (!around) recut_ = true;
    return;
  }
  if (around && pause_ != kPauseObstacle) return;
  // What held the mower changed while the planner cut (a person came, the status went stale, the object moved): no way
  // around for it, the hold goes on and asks again once objects alone stand.
  if (around && !core.obstacle_part().StillDue(added)) {
    core.Notify("Mowing: the obstacle changed while the way around was planned, waiting for it");
    return;
  }
  if (!answer.ok && answer.code == "SEARCH_LIMIT" && cut_retries_ < kCutRetries) {
    // The planner kept the room after the first try: the second one gets further.
    ++cut_retries_;
    if (!around) recut_ = true;
    return;
  }
  cut_retries_ = 0;
  if (around && (!answer.ok || answer.pieces.empty())) {
    // No way around them: the mower waits like with obstacle_mode stop.
    core.obstacle_part().AvoidRefused();
    core.Notify("Mowing: no way around the obstacle (" + (answer.ok ? std::string("nothing left to mow") : answer.code) +
                "), waiting for it to go");
    return;
  }
  if (!answer.ok) {
    // The circles left cannot be kept out of: they go, and objects in the way hold the mower again.
    cut_.reset();
    detour_ = false;
    core.obstacle_part().ClearCircles();
    core.obstacle_part().TakeCirclesGone();
    core.Notify("Mowing: the plan cannot be cut by the obstacles any more (" + answer.code +
                "), stopping at them again");
    return;
  }
  cut_ = answer.pieces;
  detour_ = true;
  approached_ = false;
  if (!around) return;
  const double left_out = LeftOut(*cut_);
  const JobArea* area = job_.CurrentArea();
  core.obstacle_part().Avoid(added);
  for (size_t i = 0; i < added.size(); ++i) {
    // The m left out go with the first circle of them.
    core.effects().push_back(ObstacleNote{added[i].track_id, "avoided", added[i].r, i == 0 ? left_out : 0.0,
                                          area ? area->id : "", area ? area->name : ""});
  }
  char text[96];
  std::snprintf(text, sizeof(text), "Mowing: driving around the obstacle, %.1f m of the area left out", left_out);
  core.Notify(text);
}

bool MowingPart::PiecesFit(const std::vector<JobPiece>& pieces) const {
  const JobProgress& at = cutting_at_;
  for (size_t i = 0; i < pieces.size(); ++i) {
    const JobPiece& piece = pieces[i];
    const JobPath* path = job_.Path(piece.path);
    // At least 3 poses within a path of the plan, none before where the cut began, in order without overlap.
    if (!path || piece.first + 2 > piece.last || piece.last >= path->poses.size()) return false;
    if (piece.path < at.path || (piece.path == at.path && static_cast<int>(piece.first) < at.index)) return false;
    if (i > 0 && (piece.path < pieces[i - 1].path ||
                  (piece.path == pieces[i - 1].path && piece.first <= pieces[i - 1].last))) {
      return false;
    }
  }
  return true;
}

void MowingPart::FollowCut() {
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access): called with a cut
  std::vector<JobPiece>& pieces = *cut_;
  const auto behind = [this](const JobPiece& piece) {
    const JobProgress& at = job_.progress();
    return piece.path < at.path || (piece.path == at.path && static_cast<int>(piece.last) <= at.index);
  };
  while (!pieces.empty() && behind(pieces.front())) pieces.erase(pieces.begin());
  if (pieces.empty()) {
    while (job_.HasPlan()) job_.PathDone();
    cut_.reset();
    detour_ = false;
    return;
  }
  const JobPiece& next = pieces.front();
  const JobProgress& at = job_.progress();
  if (next.path == at.path && static_cast<int>(next.first) <= at.index) return;
  while (job_.HasPlan() && job_.progress().path < next.path) job_.PathDone();
  job_.OnIndex(next.first);
  detour_ = true;
  approached_ = false;
}

double MowingPart::LeftOut(const std::vector<JobPiece>& pieces) const {
  const auto length = [this](int path, size_t first, size_t last) {
    double m = 0.0;
    const JobPath* p = job_.Path(path);
    for (size_t i = first + 1; p && i <= last && i < p->poses.size(); ++i) {
      m += std::hypot(p->poses[i].x - p->poses[i - 1].x, p->poses[i].y - p->poses[i - 1].y);
    }
    return m;
  };
  const JobProgress& at = job_.progress();
  double rest = 0.0;
  for (int p = at.path; const JobPath* path = job_.Path(p); ++p) {
    rest += length(p, p == at.path ? static_cast<size_t>(at.index) : 0, path->poses.size() - 1);
  }
  for (const JobPiece& piece : pieces) {
    const size_t from = piece.path == at.path ? std::max(piece.first, static_cast<size_t>(at.index)) : piece.first;
    rest -= length(piece.path, from, piece.last);
  }
  return std::max(0.0, rest);
}

void MowingPart::OnJobPlanned(CoreAccess core, std::vector<JobPath> paths, const std::string& fingerprint) {
  if (core.state() != State::kMowing || leg_ != Leg::kPlanning) return;
  leg_ = Leg::kNone;
  planner_silent_ = 0;
  job_.SetPlan(std::move(paths), fingerprint);
  approached_ = false;
  // A new plan is cut by the circles kept before the first leg.
  cut_.reset();
  detour_ = false;
  recut_ = !core.obstacle_part().circles().empty();
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
  const bool detour = leg_ == Leg::kDetour;
  // A detour is an approach within the area with the blade on.
  const bool approach = leg_ == Leg::kApproach || detour;
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
        const JobPath* path = job_.CurrentPath();
        if (cut_ && !cut_->empty() && path && cut_->front().path == job_.progress().path &&
            cut_->front().last + 1 < path->poses.size()) {
          // The piece ends before its path: poses left out come next.
          cut_->erase(cut_->begin());
        } else {
          job_.PathDone();
        }
        approached_ = false;
        core.CheckpointNow();
      } else if (approach) {
        if (detour) detour_ = false;
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
      // No way to the next piece from here after all: it is left out too, the detour goes to the one after it.
      if (detour && cut_ && !cut_->empty()) {
        cut_->erase(cut_->begin());
        break;
      }
      // After a skip the path left is a checked way on: a piece of it, then the approach is asked for again.
      if (!detour && approach && SkippedPathHere(core)) {
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
  return leg_ == Leg::kApproach || leg_ == Leg::kFollow || leg_ == Leg::kSkippedPath || leg_ == Leg::kDetour;
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
  // Circles count within a session of mowing only: out of it objects hold the mower again.
  cut_.reset();
  cutting_added_.clear();
  recut_ = detour_ = false;
  cut_retries_ = 0;
  core.obstacle_part().ClearCircles();
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
