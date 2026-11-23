#include "mowbite_logic/logic_core.h"

#include <string>
#include <utility>

namespace mowbite_logic {

bool LogicCore::Mowing() const {
  return state_ == State::kMowing;
}

void LogicCore::StartMowing() {
  mowing_.PrepareStart(CoreAccess(*this));
  SetState(State::kMowing);
  SetRunPhase(RunPhase::kMowing);
  home_.TryAgain();
  CheckpointNow();
}

void LogicCore::EndJob(bool finished, bool home, const Reason& why) {
  if (state_ != State::kMowing) return;
  if (finished) {
    RunEnds("JOB_COMPLETE");
  } else if (!why.empty()) {
    RunEnds(why.code, why.fields.value("detail", ""));
  }
  if (finished) {
    AddEvent("JOB_COMPLETE", json::object());
    mowing_.job().Complete();
  }
  if (home) {
    GoHome(why);
  } else {
    SetState(State::kIdle);
  }
  CheckpointNow();
}

void LogicCore::OnMowMap(std::vector<JobArea> areas, const Reason& problem) {
  MowingJob& job = mowing_.job();
  const JobArea* before = job.CurrentArea();
  const uint64_t shape_before = before ? before->shape : 0;
  job.SetAreas(std::move(areas));
  mow_map_problem_ = problem;
  if (state_ != State::kMowing) return;
  if (!problem.empty()) Pause(kPauseFault, problem);
  // The plan is for an area that is no longer where it was, no longer mowed or of another shape: it goes, continue
  // plans anew.
  const JobArea* area = job.CurrentArea();
  if (job.HasPlan() &&
      (!area || area->id != job.progress().area_id || !area->mowable || area->shape != shape_before)) {
    Pause(kPauseFault, Reason("MAP_CHANGED", "The mowing areas changed while mowing: continue plans anew"));
    mowing_.DropPlan();
  }
}

void LogicCore::RestoreJob(const JobProgress& progress) {
  mowing_.job().Restore(progress);
  SetDue<ActionsChanged>(true);
}

}  // namespace mowbite_logic
