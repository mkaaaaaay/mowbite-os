#include "mowbite_logic/logic_core.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "mowbite_logic/direct_route.h"

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

double LogicCore::DockedHeading() const {
  const double heading = dock_ ? dock_->heading : 0.0;
  const double facing = ContactsBack() ? heading + M_PI : heading;
  return std::atan2(std::sin(facing), std::cos(facing));
}

bool LogicCore::ContactsBack() const {
  return config_.dock_contact_x < 0;
}

const char* LogicCore::LeaveWords() const {
  return ContactsBack() ? "straight ahead" : "straight back";
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

void LogicCore::SetTurningRadius(double radius) {
  turning_radius_ = radius;
}

DockingSequence LogicCore::NewDocking() const {
  return DockingSequence(config_.dock, config_.undock, dock_.value_or(DockPose()), ContactsBack());
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
  if (!CanDock()) {
    return Reason("NO_CHARGE_CONTACTS", "This mower has no charge contacts to dock onto (geometry.dock_contact_x 0)");
  }
  if (DockContact()) return Reason("ALREADY_THERE", "The mower is on the docking station already");
  why = Unsafe();
  if (why.empty()) why = PositionProblem();
  if (why.empty()) why = drivable_problem_;
  if (why.empty() && !dock_) why = Reason("NO_DOCK", "The map has no docking station: record it first");
  if (why.empty()) why = ApproachProblem();
  return why;
}

Reason LogicCore::StartDocking() {
  Reason why = DockingProblem();
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
  docking_direct_cause_.clear();
  SetState(State::kDocking);
  const Point2 p = docking_.ApproachPoint();
  Notify("Docking: to (" + Metres(p.x) + ", " + Metres(p.y) + ") in front of the docking station, then " +
         (ContactsBack() ? "backwards" : "forwards") + " onto it");
  NextDockingTry();
}

void LogicCore::NextDockingTry() {
  if (!pose_ || !PoseGood()) {
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
    if (kind == DockingLeg::Kind::kApproach && PlannerSilent(reason)) {
      DockingWithoutPlanner(reason);
    } else if (kind == DockingLeg::Kind::kApproach) {
      DockingTryFailed(Reason("approach_failed", "The way to the docking station failed: " + reason.message));
    } else if (kind == DockingLeg::Kind::kDirect) {
      DockingTryFailed(
          Reason("approach_failed", "The straight way to the docking station failed: " + reason.message)
              .With("cause", docking_direct_cause_)
              .With("direct_blocked", false));
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
    case DockingLeg::Kind::kApproach:
    case DockingLeg::Kind::kDirect: docking_leg_due_ = docking_.Line(); break;
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

void LogicCore::DockingWithoutPlanner(const Reason& silent) {
  Reason failed =
      Reason("approach_failed", "The way to the docking station failed: " + silent.message).With("cause", silent.code);
  if (!config_.dock.direct) {
    DockingTryFailed(failed);
    return;
  }
  // The way is checked from where the mower stands, so only with a good position.
  if (!pose_ || !PoseGood()) {
    WaitForDockingGps();
    return;
  }
  // The follower holds base_link within the margin of the leg, by a position it takes up to this accuracy. Without
  // that margin it would not hold the leg to its way: not free.
  const double margin =
      config_.dock.direct_margin > 0 ? config_.dock.direct_margin + config_.max_position_accuracy : 0.0;
  const Point2 to = docking_.ApproachPoint();
  const DirectRouteCheck check =
      CheckDirectRoute({pose_->x, pose_->y}, to, turning_radius_, margin, drivable_, obstacles_);
  if (!check.free) {
    failed.message += "; the straight way is not free: " + check.why;
    DockingTryFailed(failed.With("direct_blocked", true));
    return;
  }
  const double distance = std::hypot(to.x - pose_->x, to.y - pose_->y);
  const std::string text = "No way from the planner (" + silent.message + "): straight to the point in front of the " +
                           "docking station, " + Metres(distance) + " m";
  AddEvent("DOCKING_DIRECT", {{"cause", silent.code},
                              {"distance", distance},
                              {"message", text},
                              {"severity", "warning"},
                              {"resume", "auto"}});
  Notify("Docking: " + text);
  docking_direct_cause_ = silent.code;
  if (auto leg = docking_.Direct(pose_->x, pose_->y)) {
    docking_leg_due_ = std::move(*leg);
  } else {
    docking_leg_due_ = docking_.Line();
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
  DockingLeg out = std::move(*docking_out_after_charge_off_);  // NOLINT(bugprone-unchecked-optional-access)
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

void LogicCore::CountDockingFailure(Reason why, bool last) {  // NOLINT(performance-unnecessary-value-param): may alias
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
