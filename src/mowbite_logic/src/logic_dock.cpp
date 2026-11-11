#include "mowbite_logic/logic_core.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace mowbite_logic {

Reason LogicCore::DockProblem() const {
  if (!DockContact()) return {};
  if (!dock_) return Reason("NO_DOCK", "The mower is on a docking station, but the map has none: record it first");
  return UndockEndProblem(dock_->x, dock_->y);
}

Reason LogicCore::UndockEndProblem(double x, double y) const {
  // Contacts in front: undocking backwards is not built, the reference check ends in ON_DOCK.
  if (config_.dock_contact_x >= 0) return {};
  const double heading = DockedHeading();
  const double distance = config_.undock.distance;
  if (Reachable(drivable_, obstacles_, x + distance * std::cos(heading), y + distance * std::sin(heading))) return {};
  return Reason("DOCK_UNREACHABLE", "Leaving the docking station " + Metres(distance) +
                                        " m straight ahead ends outside of the mowing and navigation areas: let an "
                                        "area reach there, or place the station closer to one")
      .With("distance", distance);
}

void LogicCore::OnDrivableAreas(std::vector<Ring> drivable, std::vector<Ring> obstacles, const Reason& problem) {
  drivable_ = std::move(drivable);
  obstacles_ = std::move(obstacles);
  drivable_problem_ = problem;
}

void LogicCore::SetUndockDistance(double distance) {
  config_.undock.distance = distance;
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

double LogicCore::DockedHeading() const {
  const double heading = dock_ ? dock_->heading : 0.0;
  const double facing = config_.dock_contact_x < 0 ? heading + M_PI : heading;
  return std::atan2(std::sin(facing), std::cos(facing));
}

void LogicCore::BeginDockCheck(DockCheck purpose) {
  dock_check_ = purpose;
  Replace(ReferenceCheck{DockedHeading()});
  SetDue<ActionsChanged>(true);
  Notify("On the docking station: checking the position against the recorded one first");
}

void LogicCore::CheckDockBeforeDrive(double x, double y) {
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
  if (why.empty()) why = purpose == DockCheck::kMowing ? StartProblem() : DriveProblem();
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
      if (config_.dock_contact_x >= 0) {
        why = Reason("ON_DOCK", "The position on the docking station fits, but undocking backwards is not built yet: "
                                "drive the mower off by remote control first")
                  .With("distance", distance)
                  .With("limit", limit);
      } else {
        // The way out from where the mower stands.
        why = UndockEndProblem(x, y);
      }
    }
  }
  if (why.empty()) {
    StartUndocking(purpose, x, y);
    return why;
  }
  if (purpose == DockCheck::kMowing) {
    RefuseStart(why);
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
    Notify("Undocking: charging off, then " + Metres(config_.undock.distance) + " m straight ahead");
  } else {
    // The hardware charges by itself and cannot stop it: straight to the path.
    Notify("Undocking: " + Metres(config_.undock.distance) + " m straight ahead");
    undock_step_ = UndockStep::kDrive;
    Replace(PlanUndock(undock_x_, undock_y_, undock_heading_, config_.undock));
  }
  SetState(State::kUndocking);
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
  Replace(PlanUndock(undock_x_, undock_y_, undock_heading_, config_.undock));
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
  outcome.why = outcome.for_drive ? MoveProblem() : MowProblem();
  if (!outcome.why.empty()) {
    if (outcome.for_drive) {
      RefuseDrive(outcome.why);
    } else {
      RefuseStart(outcome.why);
    }
    SetState(State::kIdle);
  } else if (outcome.for_drive) {
    BeginDrive(drive_x_, drive_y_);
  } else {
    StartMowing();
  }
  Replace(outcome);
}

bool LogicCore::CanDock() const {
  return config_.dock_contact_x < 0;
}

bool LogicCore::Docking() const {
  return state_ == State::kDocking;
}

void LogicCore::SetDockingSettings(double approach_distance, int retry_count) {
  config_.dock.approach_distance = approach_distance;
  config_.dock.retry_count = retry_count;
}

DockingSequence LogicCore::NewDocking() const {
  return DockingSequence(config_.dock, config_.undock, dock_.value_or(DockPose()), DockedHeading());
}

Reason LogicCore::ApproachProblem() const {
  const Point2 p = NewDocking().ApproachPoint();
  if (Reachable(drivable_, obstacles_, p.x, p.y)) return {};
  const double distance = config_.dock.approach_distance;
  return Reason("DOCK_UNREACHABLE", "The way onto the docking station begins " + Metres(distance) +
                                        " m in front of it, outside of the mowing and navigation areas: let an area "
                                        "reach there, or place the station closer to one")
      .With("distance", distance);
}

Reason LogicCore::DockingProblem() const {
  Reason why = Busy();
  if (why.empty()) why = HoldProblem();
  if (!why.empty()) return why;
  if (DockContact()) return Reason("ALREADY_THERE", "The mower is on the docking station already");
  why = Unsafe();
  if (why.empty()) why = PositionProblem();
  if (why.empty()) why = drivable_problem_;
  if (why.empty() && !dock_) why = Reason("NO_DOCK", "The map has no docking station: record it first");
  if (why.empty() && !CanDock()) why = Reason("NO_DOCK", "Docking onto contacts in front is not built");
  if (why.empty()) why = ApproachProblem();
  return why;
}

Reason LogicCore::StartDocking() {
  const Reason why = DockingProblem();
  if (!why.empty()) {
    RefuseDrive(why);
    return why;
  }
  json details = json::object();
  PutReason(&details, "DOCKING", Reason("MANUAL", "Going home: sent by the app"));
  AddEvent("DOCKING", std::move(details));
  BeginDocking();
  return {};
}

void LogicCore::GoHome(const Reason& home_why) {
  // Never after an emergency or without the safety measurement: the mower does not drive off by itself then.
  if (!CanDock() || !dock_ || !Unsafe().empty()) {
    SetState(State::kIdle);
    return;
  }
  // On the contacts already: DOCKED once the watch confirms the charge there, as for any stay on the station.
  if (DockContact()) {
    charge_watch_.Unconfirm(false);
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

void LogicCore::BeginDocking() {
  docking_ = NewDocking();
  docking_leg_due_.reset();
  docking_leg_.reset();
  docking_last_kind_ = DockingLeg::Kind::kApproach;
  docking_hold_ = false;
  docking_failure_.reset();
  docking_gps_wait_ = false;
  docking_gps_since_.reset();
  docking_gps_good_since_.reset();
  docking_charge_.reset();
  docking_checked_ = false;
  docking_out_after_charge_off_.reset();
  docking_charge_touched_ = false;
  docking_charge_stopped_ = false;
  SetState(State::kDocking);
  const Point2 p = docking_.ApproachPoint();
  Notify("Docking: to (" + Metres(p.x) + ", " + Metres(p.y) +
                     ") in front of the docking station, then backwards onto it");
  NextDockingTry();
}

void LogicCore::NextDockingTry() {
  if (!PoseGood()) {
    WaitForDockingGps();
    return;
  }
  // A failed try is counted at once where the mower does not drive out first.
  if (docking_failure_ && !docking_.OnLine(pose_->x, pose_->y)) {
    CountDockingFailure(*docking_failure_);
    return;
  }
  docking_checked_ = false;
  SetDockingLeg(docking_.FirstLeg(pose_->x, pose_->y, pose_->heading));
}

std::optional<DockingLeg> LogicCore::TakeDockingLeg() {
  if (state_ != State::kDocking || !docking_leg_due_) return std::nullopt;
  // A contact that came after the way out was set: charging off first all the same.
  if (docking_leg_due_->kind == DockingLeg::Kind::kOut) {
    DockingLeg out = std::move(*docking_leg_due_);
    docking_leg_due_.reset();
    SetDockingLeg(std::move(out));
    if (!docking_leg_due_) return std::nullopt;
  }
  std::optional<DockingLeg> leg;
  leg.swap(docking_leg_due_);
  docking_leg_ = leg->kind;
  docking_last_kind_ = leg->kind;
  docking_hold_ = false;
  return leg;
}

void LogicCore::OnDockingContact() {
  if (docking_charge_) {
    OnChargeResult(docking_charge_->OnSample(ChargeSampleNow()));
    return;
  }
  // A leg onto the station failed, and before the way out the contact is there after all: the charge decides, once
  // per try. Charging that went off for the way out is allowed again.
  const bool failed_onto =
      docking_last_kind_ == DockingLeg::Kind::kLine || docking_last_kind_ == DockingLeg::Kind::kContact;
  if (DockContact() && docking_failure_ && !docking_leg_ && failed_onto && !docking_checked_) {
    docking_leg_due_.reset();
    docking_out_after_charge_off_.reset();
    docking_failure_.reset();
    if (Pending<ChargeOff>()) {
      Drop<ChargeOff>();
      docking_charge_touched_ = false;
    }
    AllowChargeAgain();
    BeginChargeCheck();
    return;
  }
  // On the way onto the station, or waiting there for a good position; driving out starts on the contacts.
  const bool onto =
      docking_leg_ && (*docking_leg_ == DockingLeg::Kind::kLine || *docking_leg_ == DockingLeg::Kind::kContact);
  if (!onto && !docking_gps_wait_) return;
  if (DockContact()) {
    BeginChargeCheck();
    return;
  }
  if (!onto) return;
  const bool raw = hw_.telemetry_ok && hw_.dock_contact_raw;
  if (raw && !docking_hold_) {
    // G18: one zero command, the driver sends three zero frames and then nothing while the contact settles.
    docking_hold_ = true;
    SetDue<StopRequest>(true);
    Notify("Docking: contact with the docking station, stopped until it is confirmed");
  } else if (!raw && docking_hold_) {
    docking_hold_ = false;
    Notify("Docking: the contact went away before it was confirmed, on slowly");
  }
}

void LogicCore::OnDockingLegEnded(LegEnd end, const Reason& reason) {
  if (state_ != State::kDocking || !docking_leg_) return;
  const DockingLeg::Kind kind = *docking_leg_;
  docking_leg_.reset();
  docking_hold_ = false;
  if (end == LegEnd::kEmergency) {
    Notify("Docking stopped: " + reason.message);
    EndDocking(reason);
    return;
  }
  if (kind == DockingLeg::Kind::kOut) AllowChargeAgain();
  if (end == LegEnd::kGps) {
    WaitForDockingGps();
    return;
  }
  if (end != LegEnd::kDone) {
    if (kind == DockingLeg::Kind::kApproach) {
      DockingTryFailed(Reason("approach_failed", "The way to the docking station failed: " + reason.message));
    } else if (kind == DockingLeg::Kind::kOut && docking_failure_) {
      Reason why = *docking_failure_;
      why.message += "; then driving out failed: " + reason.message;
      CountDockingFailure(why);
    } else {
      DockingTryFailed(Reason("dock_failed", std::string("Docking failed on the ") + DockingLegName(kind) + ": " +
                                                 reason.message));
    }
    return;
  }
  switch (kind) {
    case DockingLeg::Kind::kApproach: docking_leg_due_ = docking_.Line(); break;
    case DockingLeg::Kind::kOut:
      if (docking_failure_) {
        CountDockingFailure(*docking_failure_);
      } else {
        NextDockingTry();
      }
      break;
    case DockingLeg::Kind::kLine: {
      const std::string gate = pose_ ? docking_.CheckGate(pose_->x, pose_->y, pose_->heading,
                                                          pose_->position_accuracy, DockAccuracyLimit())
                                     : "at the gate there was no position";
      if (gate.empty()) {
        Notify("Docking: through the gate, onto the contacts");
        docking_leg_due_ = docking_.Contact();
      } else {
        DockingTryFailed(Reason("dock_failed", "Docking failed: " + gate));
      }
      break;
    }
    case DockingLeg::Kind::kContact:
      DockingTryFailed(Reason("dock_failed", "Docking failed: no contact with the docking station up to " +
                                                 Metres(config_.dock.overshoot) + " m past it"));
      break;
  }
}

void LogicCore::DockingTryFailed(const Reason& why) {
  Notify(why.message);
  // Out to the approach point first; the try counts once the mower got there.
  if (pose_ && docking_last_kind_ != DockingLeg::Kind::kOut && docking_.OnLine(pose_->x, pose_->y)) {
    docking_failure_ = why;
    SetDockingLeg(docking_.Out(pose_->x, pose_->y, pose_->heading));
    return;
  }
  CountDockingFailure(why);
}

void LogicCore::SetDockingLeg(DockingLeg leg) {
  // Off the contacts only with charging off where the driver can stop it, like undocking (G19).
  const bool on_contacts = hw_.telemetry_ok && (hw_.dock_contact || hw_.dock_contact_raw);
  if (leg.kind == DockingLeg::Kind::kOut && charge_stop_ && on_contacts && !docking_charge_stopped_) {
    docking_out_after_charge_off_ = std::move(leg);
    docking_charge_touched_ = true;
    SetDue<ChargeOff>(true);
    Notify("Docking: charging off, then out of the docking station");
    return;
  }
  docking_leg_due_ = std::move(leg);
}

void LogicCore::DockingChargeOff(bool ok, const std::string& message) {
  DockingLeg out = std::move(*docking_out_after_charge_off_);
  docking_out_after_charge_off_.reset();
  if (ok) {
    docking_charge_stopped_ = true;
    docking_leg_due_ = std::move(out);
    return;
  }
  // Not off the contacts while they may still charge: docking ends where the mower stands.
  Reason why = docking_failure_.value_or(Reason("dock_failed", "Docking failed"));
  why.message += "; charging could not be switched off (" + message + "), the mower stays on the docking station";
  Notify(why.message);
  CountDockingFailure(why, true);
  charge_watch_.Unconfirm(why.code == "NO_CHARGE_CURRENT");
}

void LogicCore::AllowChargeAgain() {
  if (!docking_charge_touched_) return;
  docking_charge_touched_ = false;
  docking_charge_stopped_ = false;
  SetDue<ChargeOn>(true);
}

ChargeSample LogicCore::ChargeSampleNow() const {
  ChargeSample sample;
  sample.time = hw_.time_s;
  sample.contact = DockContact();
  sample.battery_ok = hw_.battery_ok;
  sample.charge_current = hw_.battery_current < 0 ? -hw_.battery_current : 0.0;
  sample.percentage = hw_.battery_percentage;
  return sample;
}

void LogicCore::BeginChargeCheck() {
  // The mower stands since the first raw contact: the leg ends, nothing drives while the charge is checked.
  CancelDockingLeg(Reason("STOPPED", "Contact with the docking station"));
  docking_gps_wait_ = false;
  docking_checked_ = true;
  docking_charge_.emplace(config_.charge, hw_.time_s);
  Notify("Docking: contact with the docking station, checking the charge");
  OnChargeResult(docking_charge_->OnSample(ChargeSampleNow()));
}

void LogicCore::OnChargeResult(ChargeCheck::Result result) {
  if (result == ChargeCheck::Result::kWaiting) return;
  const ChargeCheck check = *docking_charge_;
  docking_charge_.reset();
  char text[240];
  switch (result) {
    case ChargeCheck::Result::kCharging:
      std::snprintf(text, sizeof(text), "Docking: charging at %.2f A", check.max_current());
      Notify(text);
      Docked();
      return;
    case ChargeCheck::Result::kFull:
      std::snprintf(text, sizeof(text), "Docking: the contact held, the battery at %d %% needs no charge current",
                    check.percentage());
      Notify(text);
      Docked();
      return;
    case ChargeCheck::Result::kContactLost:
      DockingTryFailed(
          Reason("dock_failed", "Docking failed: the contact with the docking station went away while the charge "
                                "was checked"));
      return;
    case ChargeCheck::Result::kNoCharge:
    default: {
      std::snprintf(text, sizeof(text),
                    "Docking failed: no charge current on the contacts within %.0f s (at most %.2f A, %.2f A "
                    "needed below %.0f %%, the battery at %d %%)",
                    config_.charge.timeout, check.max_current(), config_.charge.current,
                    config_.charge.expected_below, check.percentage());
      const Reason why =
          Reason("NO_CHARGE_CURRENT", text).With("current", check.max_current()).With("limit", config_.charge.current);
      if (!docking_.LastTry()) {
        DockingTryFailed(why);
        return;
      }
      // The last try leaves the mower on the contacts: a person has to look anyway, and charging may still begin.
      Notify(why.message);
      CountDockingFailure(why, true);
      charge_watch_.Unconfirm(true);
      return;
    }
  }
}

void LogicCore::CountDockingFailure(Reason why, bool last) {
  docking_failure_.reset();
  const bool again = docking_.Failed() && !last;
  const char* type = again ? "DOCKING_RETRY" : "DOCKING_FAILED";
  json details = json::object();
  PutReason(&details, type, Reason(why).With("attempts", docking_.failures()));
  AddEvent(type, std::move(details));
  if (!again) {
    Notify("Docking failed, tries: " + std::to_string(docking_.failures()));
    EndDocking(Reason("STOPPED", why.message));
    return;
  }
  Notify("Docking: try " + std::to_string(docking_.failures() + 1) + " of " +
                     std::to_string(docking_.config().retry_count + 1));
  NextDockingTry();
}

void LogicCore::WaitForDockingGps() {
  docking_leg_due_.reset();
  docking_gps_wait_ = true;
  docking_gps_since_.reset();
  docking_gps_good_since_.reset();
  Notify("Docking waits for a good position");
}

void LogicCore::TickDocking(double now_s) {
  if (docking_charge_) {
    OnChargeResult(docking_charge_->OnTick(now_s));
    return;
  }
  if (!docking_gps_wait_) return;
  if (!docking_gps_since_) docking_gps_since_ = now_s;
  if (PoseGood()) {
    if (!docking_gps_good_since_) docking_gps_good_since_ = now_s;
    if (now_s - *docking_gps_good_since_ < config_.gps_wait_time) return;
    docking_gps_wait_ = false;
    Notify("Docking goes on: the position is good again");
    NextDockingTry();
    return;
  }
  docking_gps_good_since_.reset();
  if (now_s - *docking_gps_since_ < config_.gps_timeout) return;
  // Why it waited in vain, with the names of a refused start.
  char text[128];
  std::snprintf(text, sizeof(text), "Docking failed: no good position for %.0f s", config_.gps_timeout);
  Reason why;
  if (!pose_) {
    why = Reason("NO_POSITION", std::string(text) + ": no position from the localization");
  } else if (!pose_->orientation_valid) {
    why = Reason("NO_HEADING", std::string(text) + ": the heading is not known");
  } else {
    why = AccuracyReason(pose_->position_accuracy, config_.max_position_accuracy);
    why.message = std::string(text) + ": " + why.message;
  }
  CountDockingFailure(why, true);
}

void LogicCore::CancelDockingLeg(const Reason& why) {
  docking_leg_due_.reset();
  if (!docking_leg_) return;
  docking_leg_.reset();
  docking_hold_ = false;
  CancelDrive(why);
  SetDue<StopRequest>(true);
}

void LogicCore::Docked() {
  charge_watch_.Confirm();
  AddEvent("DOCKED", json::object());
  Notify("Docked");
  EndDocking(Reason("STOPPED", "Docked"));
}

void LogicCore::EndDocking(const Reason& why) {
  CancelDockingLeg(why);
  docking_failure_.reset();
  docking_gps_wait_ = false;
  docking_charge_.reset();
  docking_out_after_charge_off_.reset();
  AllowChargeAgain();
  SetState(State::kIdle);
}

void LogicCore::WatchCharge() {
  if (state_ != State::kIdle || !hw_.telemetry_ok) {
    charge_watch_.OnBusy(hw_.telemetry_ok && !hw_.dock_contact);
    return;
  }
  const ChargeSample now = ChargeSampleNow();
  const ChargeConfig& c = config_.charge;
  char text[200];
  switch (charge_watch_.OnSample(now, emergency_)) {
    case ChargeWatch::Change::kConfirmed:
      AddEvent("DOCKED", json::object());
      Notify("Docked: charging confirmed on the docking station");
      return;
    case ChargeWatch::Change::kContactLost:
      ChargingLost(Reason("NO_DOCK_CONTACT", "The mower lost the contact with the docking station"));
      return;
    case ChargeWatch::Change::kCurrentLost:
      std::snprintf(text, sizeof(text),
                    "The charge current on the docking station stopped for %.0f s (below %.2f A, the battery at "
                    "%d %%)",
                    c.lost_time, c.current, now.percentage);
      ChargingLost(Reason("NO_CHARGE_CURRENT", text).With("current", now.charge_current).With("limit", c.current));
      return;
    case ChargeWatch::Change::kNoCurrent:
      std::snprintf(text, sizeof(text),
                    "On the docking station, but no charge current for %.0f s (at least %.2f A, the battery at %d %%)",
                    c.timeout, c.current, now.percentage);
      ChargingLost(Reason("NO_CHARGE_CURRENT", text).With("current", now.charge_current).With("limit", c.current));
      return;
    case ChargeWatch::Change::kNone:
    default: return;
  }
}

void LogicCore::ChargingLost(const Reason& why) {
  json details = json::object();
  PutReason(&details, "CHARGING_LOST", why);
  if (hw_.battery_ok) details["battery_percentage"] = hw_.battery_percentage;
  AddEvent("CHARGING_LOST", std::move(details));
  Notify("Charging lost: " + why.message);
}

}  // namespace mowbite_logic
