// The reference check on the docking station and undocking.
#include <cmath>
#include <optional>
#include <string>
#include <utility>

#include "mowbite_logic/undocking_part.h"

namespace mowbite_logic {

void UndockingPart::OnGnssFix(CoreAccess core, double x, double y) {
  if (core.state() != State::kUndocking || undock_stopping_) return;
  // The first fix of the undocking is where the course is measured from; the mower stands or drives straight, against
  // its heading where it leaves backwards.
  if (!undock_fix_start_) {
    undock_fix_start_ = Point2{x, y};
    return;
  }
  if (undock_step_ != UndockStep::kDrive) return;
  const double course = core.ContactsBack() ? undock_heading_ : undock_heading_ + M_PI;
  const auto error = CourseError(undock_fix_start_->x, undock_fix_start_->y, course, x, y,
                                 core.config().undock.course_distance);
  if (!error || std::abs(*error) <= core.config().undock.course_limit) return;
  const std::string text = "Undocking stopped: the mower drove " + Degrees(std::abs(*error)) +
                           " deg off the line of the docking station, at most " +
                           Degrees(core.config().undock.course_limit) +
                           " deg allowed: the station may be turned, or its heading was recorded wrong";
  StopUndocking(core, Reason("path_failed", text), Reason("STOPPED", text));
}

void UndockingPart::BeginDockCheck(CoreAccess core, DockCheck purpose) {
  dock_check_ = purpose;
  core.effects().Replace(ReferenceCheck{core.DockedHeading()});
  core.effects().SetDue<ActionsChanged>(true);
  core.Notify("On the docking station: checking the position against the recorded one first");
}

void UndockingPart::CheckDockBeforeDrive(CoreAccess core, double x, double y) {
  core.CommandCancels();
  drive_x_ = x;
  drive_y_ = y;
  BeginDockCheck(core, DockCheck::kDrive);
}

bool UndockingPart::DockCheckPending() const {
  return dock_check_ != DockCheck::kNone;
}

Reason UndockingPart::OnDockCheck(CoreAccess core, bool ok, double x, double y, float accuracy,
                                  const std::string& message) {
  const DockCheck purpose = dock_check_;
  if (purpose == DockCheck::kNone) return Reason("STOPPED", "No reference check was on its way");
  dock_check_ = DockCheck::kNone;
  core.effects().Drop<ReferenceCheck>();
  core.effects().SetDue<ActionsChanged>(true);
  // What changed meanwhile comes first: an emergency, the contact, then the rest of a start or a drive.
  Reason why = core.Unsafe();
  if (why.empty() && !core.DockContact()) {
    why = Reason("NO_DOCK_CONTACT", "The contact with the docking station ended during the reference check");
  }
  if (why.empty() && purpose == DockCheck::kMowing) why = core.Busy();
  if (why.empty() && purpose == DockCheck::kMowing) why = core.HoldProblem();
  // Undocking does not wait for obstacles: what holds the mower now refuses the start.
  if (why.empty() && purpose == DockCheck::kMowing) {
    why = core.obstacle_part().StartProblem(core.LeavesForwards());
  }
  if (why.empty()) {
    why = purpose == DockCheck::kMowing ? core.RequestMowProblem(core.start_request()) : core.DriveProblem();
  }
  if (why.empty() && !ok) why = Reason("NO_POSITION", message.empty() ? "No answer from the localization" : message);
  if (why.empty() && !(accuracy <= core.DockAccuracyLimit())) {
    why = core.AccuracyReason(accuracy, core.DockAccuracyLimit());
    why.message += " on the docking station";
  }
  if (why.empty()) {
    const double distance = std::hypot(x - core.dock()->x, y - core.dock()->y);
    const double limit = core.config().dock_reference_limit;
    if (distance > limit) {
      why = Reason("REFERENCE_MISMATCH", "On the docking station the position is " + Metres(distance) +
                                             " m from the recorded one, at most " + Metres(limit) +
                                             " m allowed: the corrections may have another reference, or the "
                                             "station was moved")
                .With("distance", distance)
                .With("limit", limit);
    } else {
      core.Notify("Reference check on the docking station passed: " + Metres(distance) +
                         " m from the recorded position");
      // The way out from where the mower stands.
      why = core.UndockEndProblem(x, y);
    }
  }
  if (why.empty()) {
    StartUndocking(core, purpose, x, y);
    return why;
  }
  if (purpose == DockCheck::kMowing) {
    core.RefuseStart(why);
    core.RunEnds("REFUSED", why.code);
    core.FinishRun();
  } else {
    core.RefuseDrive(why);
  }
  return why;
}

void UndockingPart::StartUndocking(CoreAccess core, DockCheck purpose, double x, double y) {
  undock_for_ = purpose;
  undock_step_ = UndockStep::kChargeOff;
  undock_x_ = x;
  undock_y_ = y;
  undock_heading_ = core.DockedHeading();
  undock_fix_start_.reset();
  core.effects().SetDue<ChargeOff>(core.charge_stop());
  core.effects().Drop<UndockPath>();
  undock_stopping_ = false;
  undock_failure_.reset();
  undock_stop_why_ = Reason();
  if (core.charge_stop()) {
    core.Notify("Undocking: charging off, then " + Metres(core.config().undock.distance) + " m " + core.LeaveWords());
  } else {
    // The hardware charges by itself and cannot stop it: straight to the path.
    core.Notify("Undocking: " + Metres(core.config().undock.distance) + " m " + core.LeaveWords());
    undock_step_ = UndockStep::kDrive;
    core.effects().Replace(PlanLeave(undock_x_, undock_y_, undock_heading_, !core.ContactsBack(),
                                     core.config().undock, core.config().dock));
  }
  core.SetState(State::kUndocking);
  if (purpose == DockCheck::kMowing) core.SetRunPhase(RunPhase::kUndocking);
}

bool UndockingPart::UndockingForDrive(CoreFacts core) const {
  return core.state() == State::kUndocking && undock_for_ == DockCheck::kDrive;
}

void UndockingPart::OnChargeOff(CoreAccess core, bool ok, const std::string& message) {
  if (core.state() != State::kUndocking || undock_step_ != UndockStep::kChargeOff) return;
  if (!ok) {
    const std::string text = "Undocking failed: charging could not be switched off (" + message + ")";
    EndUndocking(core, Reason("path_failed", text), Reason("STOPPED", text));
    return;
  }
  undock_step_ = UndockStep::kDrive;
  core.effects().Replace(PlanLeave(undock_x_, undock_y_, undock_heading_, !core.ContactsBack(), core.config().undock,
                                   core.config().dock));
}

void UndockingPart::StopUndocking(CoreAccess core, const std::optional<Reason>& failure, const Reason& why) {
  if (core.state() != State::kUndocking || undock_stopping_) return;
  // Before the path went to the follower nothing moves: it ends at once.
  if (undock_step_ == UndockStep::kChargeOff || core.effects().Pending<UndockPath>()) {
    EndUndocking(core, failure, why);
    return;
  }
  undock_stopping_ = true;
  undock_failure_ = failure;
  undock_stop_why_ = why;
  core.effects().Drop<UndockPath>();
  core.CancelDrive(why);
  core.effects().SetDue<StopRequest>(true);
  core.effects().SetDue<ActionsChanged>(true);
}

void UndockingPart::AbortUndocking(CoreAccess core, const Reason& why) {
  StopUndocking(core, std::nullopt, why);
}

void UndockingPart::OnUndockEnded(CoreAccess core, LegEnd end, const Reason& reason) {
  if (core.state() != State::kUndocking || undock_step_ != UndockStep::kDrive) return;
  if (end == LegEnd::kDone && !undock_stopping_) {
    Undocked(core);
    return;
  }
  // Stopped from here: the cause decided already. Otherwise the follower's: an emergency tells by itself.
  if (undock_stopping_) {
    EndUndocking(core, undock_failure_, undock_stop_why_);
  } else if (end == LegEnd::kEmergency) {
    EndUndocking(core, std::nullopt, reason);
  } else if (end == LegEnd::kGps) {
    const Reason failure("no_gps", "Undocking stopped: " + reason.message);
    EndUndocking(core, failure, failure);
  } else {
    const Reason failure("path_failed", "Undocking failed: " + reason.message);
    EndUndocking(core, failure, failure);
  }
}

void UndockingPart::EndUndocking(CoreAccess core, const std::optional<Reason>& failure, const Reason& why) {
  if (failure) {
    json details = json::object();
    PutReason(&details, "UNDOCKING_FAILED", *failure);
    core.AddEvent("UNDOCKING_FAILED", std::move(details));
    // No second try of the schedule after it ((7a), G15).
    core.RunEnds("UNDOCKING_FAILED", failure->code);
  }
  if (!failure) core.Notify("Undocking stopped: " + why.message);
  UndockOutcome outcome;
  outcome.for_drive = undock_for_ == DockCheck::kDrive;
  outcome.why = failure.value_or(why);
  core.effects().Replace(outcome);
  // The mower may still stand on the contacts.
  core.effects().SetDue<ChargeOn>(core.charge_stop());
  core.effects().Drop<ChargeOff>();
  core.effects().Drop<UndockPath>();
  undock_stopping_ = false;
  undock_for_ = DockCheck::kNone;
  core.SetState(State::kIdle);
}

void UndockingPart::Undocked(CoreAccess core) {
  core.AddEvent("UNDOCKED", json::object());
  core.Notify("Undocked");
  UndockOutcome outcome;
  outcome.undocked = true;
  outcome.for_drive = undock_for_ == DockCheck::kDrive;
  undock_for_ = DockCheck::kNone;
  // On like a start away from the station; what changed meanwhile comes first.
  outcome.why = outcome.for_drive ? core.MoveProblem() : core.RequestMowProblem(core.start_request());
  if (!outcome.why.empty()) {
    if (outcome.for_drive) {
      core.RefuseDrive(outcome.why);
    } else {
      core.RefuseStart(outcome.why);
      core.RunEnds("REFUSED", outcome.why.code);
    }
    core.SetState(State::kIdle);
  } else if (outcome.for_drive) {
    core.BeginDrive(drive_x_, drive_y_);
  } else {
    core.StartMowing();
  }
  core.effects().Replace(outcome);
}

}  // namespace mowbite_logic
