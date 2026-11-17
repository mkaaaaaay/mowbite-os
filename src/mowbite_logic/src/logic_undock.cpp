#include "mowbite_logic/logic_core.h"

#include <cmath>
#include <optional>
#include <string>
#include <utility>

namespace mowbite_logic {

double LogicCore::DockedHeading() const {
  const double heading = dock_ ? dock_->heading : 0.0;
  const double facing = ContactsBack() ? heading + M_PI : heading;
  return std::atan2(std::sin(facing), std::cos(facing));
}

const char* LogicCore::LeaveWords() const {
  return ContactsBack() ? "straight ahead" : "straight back";
}

Reason LogicCore::UndockEndProblem(double x, double y) const {
  // The way off the station runs against the way in, whichever way the mower faces.
  const double way_in = dock_ ? dock_->heading : 0.0;
  const double distance = config_.undock.distance;
  if (Reachable(drivable_, obstacles_, x - distance * std::cos(way_in), y - distance * std::sin(way_in))) return {};
  return Reason("DOCK_UNREACHABLE", "Leaving the docking station " + Metres(distance) + " m " + LeaveWords() +
                                        " ends outside of the mowing and navigation areas: let an area reach there, "
                                        "or place the station closer to one")
      .With("distance", distance);
}

void LogicCore::SetUndockDistance(double distance) {
  config_.undock.distance = distance;
}

bool LogicCore::Undocking() const {
  return state_ == State::kUndocking;
}

void LogicCore::OnChargeOff(bool ok, const std::string& message) {
  if (state_ == State::kDocking && docking_out_after_charge_off_) {
    DockingChargeOff(ok, message);
    return;
  }
  undocking_.OnChargeOff(CoreAccess(*this), ok, message);
}

}  // namespace mowbite_logic
