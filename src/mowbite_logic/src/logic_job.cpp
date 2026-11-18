#include "mowbite_logic/logic_core.h"

#include <string>

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

}  // namespace mowbite_logic
