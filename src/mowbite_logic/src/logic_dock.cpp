#include "mowbite_logic/logic_core.h"

#include <algorithm>
#include <utility>

namespace mowbite_logic {

Reason LogicCore::DockProblem() const {
  if (!DockContact()) return {};
  if (!dock_) return Reason("NO_DOCK", "The mower is on a docking station, but the map has none: record it first");
  return UndockEndProblem(dock_->x, dock_->y);
}

void LogicCore::OnDrivableAreas(std::vector<Ring> drivable, std::vector<Ring> obstacles, const Reason& problem) {
  drivable_ = std::move(drivable);
  obstacles_ = std::move(obstacles);
  drivable_problem_ = problem;
}

double LogicCore::DockAccuracyLimit() const {
  return std::min(config_.dock_max_position_accuracy, config_.max_position_accuracy);
}

bool LogicCore::DockContact() const {
  return hw_.telemetry_ok && hw_.dock_contact;
}

void LogicCore::OnMapDock(const std::optional<DockPose>& dock) {
  dock_ = dock;
}

bool LogicCore::ContactsBack() const {
  return config_.dock_contact_x < 0;
}

bool LogicCore::LeavesForwards() const {
  return DockContact() && ContactsBack();
}

bool LogicCore::CanDock() const {
  return config_.dock_contact_x != 0;
}

bool LogicCore::Docking() const {
  return state_ == State::kDocking;
}

void LogicCore::SetDockingSettings(double approach_distance, int retry_count) {
  config_.dock.approach_distance = approach_distance;
  config_.dock.retry_count = retry_count;
}

void LogicCore::GoHome(const Reason& home_why) {
  // Never after an emergency or without the safety measurement: the mower does not drive off by itself then.
  if (!CanDock() || !dock_ || !Unsafe().empty()) {
    SetState(State::kIdle);
    return;
  }
  // On the contacts already: DOCKED once the watch confirms the charge there, as for any stay on the station.
  if (DockContact()) {
    docking_part_.charge_watch().Unconfirm(false);
    SetState(State::kIdle);
    return;
  }
  Reason why = drivable_problem_;
  if (why.empty()) why = ApproachProblem();
  if (!why.empty()) {
    // An approach that cannot succeed fails like OpenMower's: DOCKING_FAILED approach_failed, not DOCK_UNREACHABLE.
    json details = json::object();
    PutReason(&details, "DOCKING_FAILED", Reason("approach_failed", why.message).With("attempts", 1));
    AddEvent("DOCKING_FAILED", std::move(details));
    Notify("Docking failed: " + why.message);
    SetState(State::kIdle);
    return;
  }
  if (!home_why.empty()) {
    json details = json::object();
    PutReason(&details, "DOCKING", home_why);
    AddEvent("DOCKING", std::move(details));
    Notify(home_why.message);
  }
  BeginDocking();
}

bool LogicCore::HomeReachable() const {
  return CanDock() && dock_ && Unsafe().empty() && !DockContact() && drivable_problem_.empty() &&
         ApproachProblem().empty();
}

}  // namespace mowbite_logic
