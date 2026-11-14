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

void LogicCore::OnGnssFix(double x, double y) {
  if (state_ != State::kUndocking || undock_stopping_) return;
  // The first fix of the undocking is where the course is measured from; the mower stands or drives straight, against
  // its heading where it leaves backwards.
  if (!undock_fix_start_) {
    undock_fix_start_ = Point2{x, y};
    return;
  }
  if (undock_step_ != UndockStep::kDrive) return;
  const double course = ContactsBack() ? undock_heading_ : undock_heading_ + M_PI;
  const auto error = CourseError(undock_fix_start_->x, undock_fix_start_->y, course, x, y,
                                 config_.undock.course_distance);
  if (!error || std::abs(*error) <= config_.undock.course_limit) return;
  const std::string text = "Undocking stopped: the mower drove " + Degrees(std::abs(*error)) +
                           " deg off the line of the docking station, at most " + Degrees(config_.undock.course_limit) +
                           " deg allowed: the station may be turned, or its heading was recorded wrong";
  StopUndocking(Reason("path_failed", text), Reason("STOPPED", text));
}

void LogicCore::BeginDockCheck(DockCheck purpose) {
  dock_check_ = purpose;
  Replace(ReferenceCheck{DockedHeading()});
  SetDue<ActionsChanged>(true);
  Notify("On the docking station: checking the position against the recorded one first");
}

void LogicCore::CheckDockBeforeDrive(double x, double y) {
  CommandCancels();
  drive_x_ = x;
  drive_y_ = y;
  BeginDockCheck(DockCheck::kDrive);
}

bool LogicCore::DockCheckPending() const {
  return dock_check_ != DockCheck::kNone;
}

Reason LogicCore::OnDockCheck(bool ok, double x, double y, float accuracy, const std::string& message) {
  const DockCheck purpose = dock_check_;
  if (purpose == DockCheck::kNone) return Reason("STOPPED", "No reference check was on its way");
  dock_check_ = DockCheck::kNone;
  Drop<ReferenceCheck>();
  SetDue<ActionsChanged>(true);
  // What changed meanwhile comes first: an emergency, the contact, then the rest of a start or a drive.
  Reason why = Unsafe();
  if (why.empty() && !DockContact()) {
    why = Reason("NO_DOCK_CONTACT", "The contact with the docking station ended during the reference check");
  }
  if (why.empty() && purpose == DockCheck::kMowing) why = Busy();
  if (why.empty() && purpose == DockCheck::kMowing) why = HoldProblem();
  if (why.empty()) why = purpose == DockCheck::kMowing ? RequestMowProblem(start_request_) : DriveProblem();
  if (why.empty() && !ok) why = Reason("NO_POSITION", message.empty() ? "No answer from the localization" : message);
  if (why.empty() && !(accuracy <= DockAccuracyLimit())) {
    why = AccuracyReason(accuracy, DockAccuracyLimit());
    why.message += " on the docking station";
  }
  if (why.empty()) {
    const double distance = std::hypot(x - dock_->x, y - dock_->y);
    const double limit = config_.dock_reference_limit;
    if (distance > limit) {
      why = Reason("REFERENCE_MISMATCH", "On the docking station the position is " + Metres(distance) +
                                             " m from the recorded one, at most " + Metres(limit) +
                                             " m allowed: the corrections may have another reference, or the "
                                             "station was moved")
                .With("distance", distance)
                .With("limit", limit);
    } else {
      Notify("Reference check on the docking station passed: " + Metres(distance) +
                         " m from the recorded position");
      // The way out from where the mower stands.
      why = UndockEndProblem(x, y);
    }
  }
  if (why.empty()) {
    StartUndocking(purpose, x, y);
    return why;
  }
  if (purpose == DockCheck::kMowing) {
    RefuseStart(why);
    RunEnds("REFUSED", why.code);
    FinishRun();
  } else {
    RefuseDrive(why);
  }
  return why;
}

void LogicCore::StartUndocking(DockCheck purpose, double x, double y) {
  undock_for_ = purpose;
  undock_step_ = UndockStep::kChargeOff;
  undock_x_ = x;
  undock_y_ = y;
  undock_heading_ = DockedHeading();
  undock_fix_start_.reset();
  SetDue<ChargeOff>(charge_stop_);
  Drop<UndockPath>();
  undock_stopping_ = false;
  undock_failure_.reset();
  undock_stop_why_ = Reason();
  if (charge_stop_) {
    Notify("Undocking: charging off, then " + Metres(config_.undock.distance) + " m " + LeaveWords());
  } else {
    // The hardware charges by itself and cannot stop it: straight to the path.
    Notify("Undocking: " + Metres(config_.undock.distance) + " m " + LeaveWords());
    undock_step_ = UndockStep::kDrive;
    Replace(PlanLeave(undock_x_, undock_y_, undock_heading_, !ContactsBack(), config_.undock, config_.dock));
  }
  SetState(State::kUndocking);
  if (purpose == DockCheck::kMowing) SetRunPhase(RunPhase::kUndocking);
}

bool LogicCore::Undocking() const {
  return state_ == State::kUndocking;
}

bool LogicCore::UndockingForDrive() const {
  return state_ == State::kUndocking && undock_for_ == DockCheck::kDrive;
}

void LogicCore::OnChargeOff(bool ok, const std::string& message) {
  if (state_ == State::kDocking && docking_out_after_charge_off_) {
    DockingChargeOff(ok, message);
    return;
  }
  if (state_ != State::kUndocking || undock_step_ != UndockStep::kChargeOff) return;
  if (!ok) {
    const std::string text = "Undocking failed: charging could not be switched off (" + message + ")";
    EndUndocking(Reason("path_failed", text), Reason("STOPPED", text));
    return;
  }
  undock_step_ = UndockStep::kDrive;
  Replace(PlanLeave(undock_x_, undock_y_, undock_heading_, !ContactsBack(), config_.undock, config_.dock));
}

void LogicCore::StopUndocking(const std::optional<Reason>& failure, const Reason& why) {
  if (state_ != State::kUndocking || undock_stopping_) return;
  // Before the path went to the follower nothing moves: it ends at once.
  if (undock_step_ == UndockStep::kChargeOff || Pending<UndockPath>()) {
    EndUndocking(failure, why);
    return;
  }
  undock_stopping_ = true;
  undock_failure_ = failure;
  undock_stop_why_ = why;
  Drop<UndockPath>();
  CancelDrive(why);
  SetDue<StopRequest>(true);
  SetDue<ActionsChanged>(true);
}

void LogicCore::AbortUndocking(const Reason& why) {
  StopUndocking(std::nullopt, why);
}

void LogicCore::OnUndockEnded(LegEnd end, const Reason& reason) {
  if (state_ != State::kUndocking || undock_step_ != UndockStep::kDrive) return;
  if (end == LegEnd::kDone && !undock_stopping_) {
    Undocked();
    return;
  }
  // Stopped from here: the cause decided already. Otherwise the follower's: an emergency tells by itself.
  if (undock_stopping_) {
    EndUndocking(undock_failure_, undock_stop_why_);
  } else if (end == LegEnd::kEmergency) {
    EndUndocking(std::nullopt, reason);
  } else if (end == LegEnd::kGps) {
    const Reason failure("no_gps", "Undocking stopped: " + reason.message);
    EndUndocking(failure, failure);
  } else {
    const Reason failure("path_failed", "Undocking failed: " + reason.message);
    EndUndocking(failure, failure);
  }
}

void LogicCore::EndUndocking(const std::optional<Reason>& failure, const Reason& why) {
  if (failure) {
    json details = json::object();
    PutReason(&details, "UNDOCKING_FAILED", *failure);
    AddEvent("UNDOCKING_FAILED", std::move(details));
    // No second try of the schedule after it ((7a), G15).
    RunEnds("UNDOCKING_FAILED", failure->code);
  }
  if (!failure) Notify("Undocking stopped: " + why.message);
  UndockOutcome outcome;
  outcome.for_drive = undock_for_ == DockCheck::kDrive;
  outcome.why = failure.value_or(why);
  Replace(outcome);
  // The mower may still stand on the contacts.
  SetDue<ChargeOn>(charge_stop_);
  Drop<ChargeOff>();
  Drop<UndockPath>();
  undock_stopping_ = false;
  undock_for_ = DockCheck::kNone;
  SetState(State::kIdle);
}

void LogicCore::Undocked() {
  AddEvent("UNDOCKED", json::object());
  Notify("Undocked");
  UndockOutcome outcome;
  outcome.undocked = true;
  outcome.for_drive = undock_for_ == DockCheck::kDrive;
  undock_for_ = DockCheck::kNone;
  // On like a start away from the station; what changed meanwhile comes first.
  outcome.why = outcome.for_drive ? MoveProblem() : RequestMowProblem(start_request_);
  if (!outcome.why.empty()) {
    if (outcome.for_drive) {
      RefuseDrive(outcome.why);
    } else {
      RefuseStart(outcome.why);
      RunEnds("REFUSED", outcome.why.code);
    }
    SetState(State::kIdle);
  } else if (outcome.for_drive) {
    BeginDrive(drive_x_, drive_y_);
  } else {
    StartMowing();
  }
  Replace(outcome);
}

}  // namespace mowbite_logic
