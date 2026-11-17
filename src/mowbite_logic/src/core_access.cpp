#include <string>
#include <utility>

#include "mowbite_logic/logic_core.h"

namespace mowbite_logic {

State CoreFacts::state() const {
  return read_->state_;
}

bool CoreFacts::emergency() const {
  return read_->emergency_;
}

const HardwareSnapshot& CoreFacts::hw() const {
  return read_->hw_;
}

bool CoreFacts::DockContact() const {
  return read_->DockContact();
}

Reason CoreFacts::Busy() const {
  return read_->Busy();
}

Reason CoreFacts::HoldProblem() const {
  return read_->HoldProblem();
}

void CoreAccess::Notify(std::string line) {
  core_->Notify(std::move(line));
}

void CoreAccess::SetState(State state) {
  core_->SetState(state);
}

void CoreAccess::CommandCancels() {
  core_->CommandCancels();
}

}  // namespace mowbite_logic
