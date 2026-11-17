#include <optional>
#include <string>
#include <utility>

#include "mowbite_logic/logic_core.h"

namespace mowbite_logic {

const LogicConfig& CoreFacts::config() const {
  return read_->config_;
}

State CoreFacts::state() const {
  return read_->state_;
}

bool CoreFacts::emergency() const {
  return read_->emergency_;
}

const HardwareSnapshot& CoreFacts::hw() const {
  return read_->hw_;
}

const std::optional<PoseSnapshot>& CoreFacts::pose() const {
  return read_->pose_;
}

const Reason& CoreFacts::drivable_problem() const {
  return read_->drivable_problem_;
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

Reason CoreFacts::Unsafe() const {
  return read_->Unsafe();
}

Reason CoreFacts::AccuracyReason(float accuracy, double limit) const {
  return read_->AccuracyReason(accuracy, limit);
}

EffectQueue& CoreAccess::effects() {
  return core_->effects_;
}

std::string& CoreAccess::client() {
  return core_->client_;
}

HomeWatch& CoreAccess::home() {
  return core_->home_;
}

std::string CoreAccess::AddEvent(const std::string& type, json details) {
  return core_->AddEvent(type, std::move(details));
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
