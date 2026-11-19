// Docking onto the station: the tries, their legs, the straight way without the planner and the check of the charge.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>

#include "mowbite_logic/direct_route.h"
#include "mowbite_logic/docking_part.h"

namespace mowbite_logic {

void DockingPart::SetTurningRadius(double radius) {
  turning_radius_ = radius;
}

DockingSequence DockingPart::NewDocking(CoreFacts core) const {
  if (docking_test_) {
    return MarkerDocking(core.config().dock, core.config().undock, core.ContactsBack(), core.config().marker);
  }
  DockConfig dock = core.config().dock;
  // The line by the marker begins where the camera finds it, and so does the one by the position.
  if (Guided(core)) dock.approach_distance = std::min(dock.approach_distance, core.config().marker.range);
  return DockingSequence(dock, core.config().undock, core.dock().value_or(DockPose()), core.ContactsBack());
}

Reason DockingPart::ApproachProblem(CoreFacts core) const {
  const DockingSequence docking = NewDocking(core);
  const Point2 p = docking.ApproachPoint();
  if (Reachable(core.drivable(), core.obstacles(), p.x, p.y)) return {};
  const double distance = docking.config().approach_distance;
  return Reason("DOCK_UNREACHABLE", "The way onto the docking station begins " + Metres(distance) +
                                        " m in front of it, outside of the mowing and navigation areas: let an area "
                                        "reach there, or place the station closer to one")
      .With("distance", distance);
}

Reason DockingPart::DockingProblem(CoreFacts core) const {
  Reason why = DockStartProblem(core);
  if (why.empty()) why = core.PositionProblem();
  if (why.empty()) why = core.drivable_problem();
  if (why.empty() && !core.dock()) why = Reason("NO_DOCK", "The map has no docking station: record it first");
  if (why.empty()) why = ApproachProblem(core);
  return why;
}

Reason DockingPart::StartDocking(CoreAccess core) {
  Reason why = DockingProblem(core);
  if (!why.empty()) {
    core.RefuseDrive(why);
    return why;
  }
  json details = json::object();
  PutReason(&details, "DOCKING", Reason("MANUAL", "Going home: sent by the app"));
  core.AddEvent("DOCKING", std::move(details));
  BeginDocking(core);
  return {};
}

void DockingPart::BeginDocking(CoreAccess core) {
  docking_ = NewDocking(core);
  if (MarkerSource(core)) {
    marker_legs_ = MarkerDocking(core.config().dock, core.config().undock, core.ContactsBack(), core.config().marker);
  }
  docking_leg_due_.reset();
  docking_leg_.reset();
  docking_last_kind_ = DockingLeg::Kind::kApproach;
  docking_hold_ = false;
  docking_failure_.reset();
  docking_gps_wait_ = false;
  docking_obstacle_wait_ = false;
  docking_gps_since_.reset();
  docking_gps_good_since_.reset();
  docking_charge_.reset();
  docking_checked_ = false;
  docking_out_after_charge_off_.reset();
  docking_charge_touched_ = false;
  docking_charge_stopped_ = false;
  docking_direct_cause_.clear();
  core.SetState(State::kDocking);
  const Point2 p = docking_.ApproachPoint();
  core.Notify("Docking: to (" + Metres(p.x) + ", " + Metres(p.y) + ") in front of the docking station, then " +
         (core.ContactsBack() ? "backwards" : "forwards") + " onto it");
  NextDockingTry(core);
}

void DockingPart::NextDockingTry(CoreAccess core) {
  const auto& pose_now = core.pose();
  if (marker_docking_) return NextMarkerTry(core);
  const bool may_use_marker = Guided(core) && !try_by_position_;
  if (!pose_now || !core.PoseGood()) {
    // A marker in sight may guide the try without the map, then without the comparison.
    if (may_use_marker && DockPoseGood(core)) return ChooseGuide(core);
    WaitForDockingGps(core);
    return;
  }
  // A failed try is counted at once where the mower does not drive out first.
  if (docking_failure_ && !docking_.OnLine(pose_now->x, pose_now->y, pose_now->heading)) {
    CountDockingFailure(core, *docking_failure_);
    return;
  }
  docking_checked_ = false;
  DockingLeg leg = docking_.FirstLeg(pose_now->x, pose_now->y, pose_now->heading);
  // At the approach point the try chooses its way.
  if (leg.kind == DockingLeg::Kind::kLine && may_use_marker) return ChooseGuide(core);
  SetDockingLeg(core, std::move(leg));
}

std::optional<DockingLeg> DockingPart::TakeDockingLeg(CoreAccess core) {
  if (core.state() != State::kDocking || !docking_leg_due_) return std::nullopt;
  // A contact that came after the way out was set: charging off first all the same.
  if (docking_leg_due_->kind == DockingLeg::Kind::kOut) {
    DockingLeg out = std::move(*docking_leg_due_);
    docking_leg_due_.reset();
    SetDockingLeg(core, std::move(out));
    if (!docking_leg_due_) return std::nullopt;
  }
  std::optional<DockingLeg> leg;
  leg.swap(docking_leg_due_);
  docking_leg_ = leg->kind;
  docking_leg_frame_ = leg->frame;
  docking_last_kind_ = leg->kind;
  docking_hold_ = false;
  return leg;
}

void DockingPart::OnDockingContact(CoreAccess core) {
  if (docking_charge_) {
    OnChargeResult(core, docking_charge_->OnSample(ChargeSampleNow(core)));
    return;
  }
  // A leg onto the station failed, and before the way out the contact is there after all: the charge decides, once
  // per try. Charging that went off for the way out is allowed again.
  const bool failed_onto = OntoStation(docking_last_kind_);
  if (core.DockContact() && docking_failure_ && !docking_leg_ && failed_onto && !docking_checked_) {
    docking_leg_due_.reset();
    docking_out_after_charge_off_.reset();
    docking_failure_.reset();
    if (core.effects().Pending<ChargeOff>()) {
      core.effects().Drop<ChargeOff>();
      docking_charge_touched_ = false;
    }
    AllowChargeAgain(core);
    BeginChargeCheck(core);
    return;
  }
  // On the way onto the station, or waiting there for a good position; driving out starts on the contacts.
  const bool onto = docking_leg_ && OntoStation(*docking_leg_);
  if (!onto && !docking_gps_wait_) return;
  if (core.DockContact()) {
    BeginChargeCheck(core);
    return;
  }
  if (!onto) return;
  const bool raw = core.hw().telemetry_ok && core.hw().dock_contact_raw;
  if (raw && !docking_hold_) {
    // G18: one zero command, the driver sends three zero frames and then nothing while the contact settles.
    docking_hold_ = true;
    core.effects().SetDue<StopRequest>(true);
    core.Notify("Docking: contact with the docking station, stopped until it is confirmed");
  } else if (!raw && docking_hold_) {
    docking_hold_ = false;
    core.Notify("Docking: the contact went away before it was confirmed, on slowly");
  }
}

void DockingPart::OnDockingLegEnded(CoreAccess core, LegEnd end, const Reason& reason) {
  if (core.state() != State::kDocking || !docking_leg_) return;
  const DockingLeg::Kind kind = *docking_leg_;
  docking_leg_.reset();
  docking_hold_ = false;
  if (end == LegEnd::kEmergency) {
    core.Notify("Docking stopped: " + reason.message);
    EndDocking(core, reason);
    return;
  }
  if (kind == DockingLeg::Kind::kOut) AllowChargeAgain(core);
  if (end == LegEnd::kGps) {
    WaitForDockingGps(core);
    return;
  }
  if (end != LegEnd::kDone) {
    if (kind == DockingLeg::Kind::kApproach && PlannerSilent(reason)) {
      DockingWithoutPlanner(core, reason);
    } else if (kind == DockingLeg::Kind::kApproach) {
      DockingTryFailed(core, Reason("approach_failed", "The way to the docking station failed: " + reason.message));
    } else if (kind == DockingLeg::Kind::kDirect) {
      DockingTryFailed(core, 
          Reason("approach_failed", "The straight way to the docking station failed: " + reason.message)
              .With("cause", docking_direct_cause_)
              .With("direct_blocked", false));
    } else if (kind == DockingLeg::Kind::kOut && docking_failure_) {
      Reason why = *docking_failure_;
      why.message += "; then driving out failed: " + reason.message;
      CountDockingFailure(core, why);
    } else if (kind == DockingLeg::Kind::kOut && std::exchange(test_next_run_, false)) {
      CountDockingFailure(
          core, Reason("dock_failed", "Docking test: driving out of the station failed: " + reason.message), true);
    } else {
      DockingTryFailed(core, Reason("dock_failed", std::string("Docking failed on the ") + DockingLegName(kind) +
                                                       ": " + reason.message));
    }
    return;
  }
  switch (kind) {
    case DockingLeg::Kind::kApproach:
    case DockingLeg::Kind::kDirect:
      if (Guided(core) && !try_by_position_) {
        ChooseGuide(core);
      } else {
        docking_leg_due_ = docking_.Line();
      }
      break;
    case DockingLeg::Kind::kOut:
      if (docking_failure_) {
        CountDockingFailure(core, *docking_failure_);
      } else if (std::exchange(test_next_run_, false)) {
        // The next run of the series begins at the start of the line with a sight of the marker.
        WaitForMarker(core, true);
      } else {
        NextDockingTry(core);
      }
      break;
    case DockingLeg::Kind::kLine: {
      const Reason gate = GateProblem(core);
      if (gate.empty()) {
        core.Notify("Docking: through the gate, onto the contacts");
        docking_leg_due_ = Legs().AfterGate();
      } else {
        DockingTryFailed(core, gate);
      }
      break;
    }
    case DockingLeg::Kind::kClose: docking_leg_due_ = Legs().Contact(); break;
    case DockingLeg::Kind::kContact:
      DockingTryFailed(core, Reason("dock_failed", "Docking failed: no contact with the docking station up to " +
                                                 Metres(core.config().dock.overshoot) + " m past it"));
      break;
  }
}

void DockingPart::DockingWithoutPlanner(CoreAccess core, const Reason& silent) {
  const auto& pose_now = core.pose();
  Reason failed =
      Reason("approach_failed", "The way to the docking station failed: " + silent.message).With("cause", silent.code);
  if (!core.config().dock.direct) {
    DockingTryFailed(core, failed);
    return;
  }
  // The way is checked from where the mower stands, so only with a good position.
  if (!pose_now || !core.PoseGood()) {
    WaitForDockingGps(core);
    return;
  }
  // The follower holds base_link within the margin of the leg, by a position it takes up to this accuracy. Without
  // that margin it would not hold the leg to its way: not free.
  const double margin = core.config().dock.direct_margin > 0
                            ? core.config().dock.direct_margin + core.config().max_position_accuracy
                            : 0.0;
  const Point2 to = docking_.ApproachPoint();
  const DirectRouteCheck check = CheckDirectRoute({pose_now->x, pose_now->y}, to, turning_radius_, margin,
                                                  core.drivable(), core.obstacles());
  if (!check.free) {
    failed.message += "; the straight way is not free: " + check.why;
    DockingTryFailed(core, failed.With("direct_blocked", true));
    return;
  }
  const double distance = std::hypot(to.x - pose_now->x, to.y - pose_now->y);
  const std::string text = "No way from the planner (" + silent.message + "): straight to the point in front of the " +
                           "docking station, " + Metres(distance) + " m";
  core.AddEvent("DOCKING_DIRECT", {{"cause", silent.code},
                              {"distance", distance},
                              {"message", text},
                              {"severity", "warning"},
                              {"resume", "auto"}});
  core.Notify("Docking: " + text);
  docking_direct_cause_ = silent.code;
  if (auto leg = docking_.Direct(pose_now->x, pose_now->y)) {
    docking_leg_due_ = std::move(*leg);
  } else {
    docking_leg_due_ = docking_.Line();
  }
}

void DockingPart::DockingTryFailed(CoreAccess core, const Reason& why) {
  core.Notify(why.message);
  if (marker_docking_) return MarkerTryFailed(core, why);
  // Out to the approach point first, by a good pose; the try counts once the mower got there.
  if (core.PoseGood() && docking_last_kind_ != DockingLeg::Kind::kOut &&
      docking_.OnLine(core.pose()->x, core.pose()->y, core.pose()->heading)) {
    docking_failure_ = why;
    SetDockingLeg(core, docking_.Out(core.pose()->x, core.pose()->y, core.pose()->heading));
    return;
  }
  CountDockingFailure(core, why);
}

void DockingPart::SetDockingLeg(CoreAccess core, DockingLeg leg) {
  // Off the contacts only with charging off where the driver can stop it, like undocking (G19).
  const bool on_contacts = core.hw().telemetry_ok && (core.hw().dock_contact || core.hw().dock_contact_raw);
  if (leg.kind == DockingLeg::Kind::kOut && core.charge_stop() && on_contacts && !docking_charge_stopped_) {
    docking_out_after_charge_off_ = std::move(leg);
    docking_charge_touched_ = true;
    core.effects().SetDue<ChargeOff>(true);
    core.Notify("Docking: charging off, then out of the docking station");
    return;
  }
  docking_leg_due_ = std::move(leg);
}

void DockingPart::DockingChargeOff(CoreAccess core, bool ok, const std::string& message) {
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
  core.Notify(why.message);
  CountDockingFailure(core, why, true);
  charge_watch_.Unconfirm(why.code == "NO_CHARGE_CURRENT");
}

void DockingPart::AllowChargeAgain(CoreAccess core) {
  if (!docking_charge_touched_) return;
  docking_charge_touched_ = false;
  docking_charge_stopped_ = false;
  core.effects().SetDue<ChargeOn>(true);
}

ChargeSample DockingPart::ChargeSampleNow(CoreFacts core) const {
  ChargeSample sample;
  sample.time = core.hw().time_s;
  sample.contact = core.DockContact();
  sample.battery_ok = core.hw().battery_ok;
  sample.charge_current = core.hw().battery_current < 0 ? -core.hw().battery_current : 0.0;
  sample.percentage = core.hw().battery_percentage;
  return sample;
}

void DockingPart::BeginChargeCheck(CoreAccess core) {
  // The mower stands since the first raw contact: the leg ends, nothing drives while the charge is checked.
  CancelDockingLeg(core, Reason("STOPPED", "Contact with the docking station"));
  docking_gps_wait_ = false;
  docking_checked_ = true;
  docking_charge_.emplace(core.config().charge, core.hw().time_s);
  core.Notify("Docking: contact with the docking station, checking the charge");
  OnChargeResult(core, docking_charge_->OnSample(ChargeSampleNow(core)));
}

void DockingPart::OnChargeResult(CoreAccess core, ChargeCheck::Result result) {
  if (result == ChargeCheck::Result::kWaiting) return;
  const ChargeCheck check = *docking_charge_;
  docking_charge_.reset();
  char text[240];
  switch (result) {
    case ChargeCheck::Result::kCharging:
      std::snprintf(text, sizeof(text), "Docking: charging at %.2f A", check.max_current());
      core.Notify(text);
      Docked(core);
      return;
    case ChargeCheck::Result::kFull:
      std::snprintf(text, sizeof(text), "Docking: the contact held, the battery at %d %% needs no charge current",
                    check.percentage());
      core.Notify(text);
      Docked(core);
      return;
    case ChargeCheck::Result::kContactLost:
      DockingTryFailed(core, 
          Reason("dock_failed", "Docking failed: the contact with the docking station went away while the charge "
                                "was checked"));
      return;
    case ChargeCheck::Result::kNoCharge:
    default: {
      std::snprintf(text, sizeof(text),
                    "Docking failed: no charge current on the contacts within %.0f s (at most %.2f A, %.2f A "
                    "needed below %.0f %%, the battery at %d %%)",
                    core.config().charge.timeout, check.max_current(), core.config().charge.current,
                    core.config().charge.expected_below, check.percentage());
      const Reason why = Reason("NO_CHARGE_CURRENT", text)
                             .With("current", check.max_current())
                             .With("limit", core.config().charge.current);
      if (!docking_.LastTry() && !Series()) {
        DockingTryFailed(core, why);
        return;
      }
      // The last try leaves the mower on the contacts: a person has to look anyway, and charging may still begin.
      core.Notify(why.message);
      CountDockingFailure(core, why, true);
      charge_watch_.Unconfirm(true);
      return;
    }
  }
}

// NOLINTNEXTLINE(performance-unnecessary-value-param): may alias
void DockingPart::CountDockingFailure(CoreAccess core, Reason why, bool last) {
  docking_failure_.reset();
  // In a series of docking.test the first failure ends it.
  const bool again = docking_.Failed() && !last && !Series();
  const char* type = again ? "DOCKING_RETRY" : "DOCKING_FAILED";
  if (!again) TestRunEnded(core, why);
  json details = json::object();
  PutReason(&details, type, Reason(why).With("attempts", docking_.failures()).With("guide", Guide()));
  core.AddEvent(type, std::move(details));
  // Each try chooses its way anew.
  if (!docking_test_) marker_docking_ = try_by_marker_ = try_by_position_ = false;
  if (!again) {
    core.Notify("Docking failed, tries: " + std::to_string(docking_.failures()));
    EndDocking(core, Reason("STOPPED", why.message));
    return;
  }
  core.Notify("Docking: try " + std::to_string(docking_.failures() + 1) + " of " +
                     std::to_string(docking_.config().retry_count + 1));
  NextDockingTry(core);
}

void DockingPart::WaitForDockingGps(CoreAccess core) {
  docking_leg_due_.reset();
  docking_gps_wait_ = true;
  docking_gps_since_.reset();
  docking_gps_good_since_.reset();
  core.Notify(marker_docking_ ? "Docking waits for the marker" : "Docking waits for a good position");
}

void DockingPart::TickDocking(CoreAccess core, double now_s) {
  const auto& mower = core.pose();
  // Poses of the marker age also while a leg in the map drives.
  AgeDockPose(now_s);
  if (docking_charge_) {
    OnChargeResult(core, docking_charge_->OnTick(now_s));
    return;
  }
  if (docking_obstacle_wait_) return;
  if (marker_docking_) return TickMarkerDocking(core, now_s);
  if (!docking_gps_wait_) return;
  if (!docking_gps_since_) docking_gps_since_ = now_s;
  if (core.PoseGood()) {
    if (!docking_gps_good_since_) docking_gps_good_since_ = now_s;
    if (now_s - *docking_gps_good_since_ < core.config().gps_wait_time) return;
    docking_gps_wait_ = false;
    core.Notify("Docking goes on: the position is good again");
    NextDockingTry(core);
    return;
  }
  docking_gps_good_since_.reset();
  if (now_s - *docking_gps_since_ < core.config().gps_timeout) return;
  // Why it waited in vain, with the names of a refused start.
  char text[128];
  std::snprintf(text, sizeof(text), "Docking failed: no good position for %.0f s", core.config().gps_timeout);
  Reason why;
  if (!mower) {
    why = Reason("NO_POSITION", std::string(text) + ": no position from the localization");
  } else if (!mower->orientation_valid) {
    why = Reason("NO_HEADING", std::string(text) + ": the heading is not known");
  } else {
    why = core.AccuracyReason(mower->position_accuracy, core.config().max_position_accuracy);
    why.message = std::string(text) + ": " + why.message;
  }
  CountDockingFailure(core, why, true);
}

bool DockingPart::TowardsStation() const {
  const auto towards = [](DockingLeg::Kind kind) {
    return kind == DockingLeg::Kind::kApproach || kind == DockingLeg::Kind::kDirect;
  };
  return (docking_leg_ && towards(*docking_leg_)) || (docking_leg_due_ && towards(docking_leg_due_->kind));
}

void DockingPart::ObstacleHold(CoreAccess core, bool hold, const Reason& why) {
  if (hold == docking_obstacle_wait_ || core.state() != State::kDocking) return;
  if (hold) {
    if (!TowardsStation()) return;
    CancelDockingLeg(core, why);
    docking_obstacle_wait_ = true;
    core.Notify("Docking waits: " + why.message);
    return;
  }
  docking_obstacle_wait_ = false;
  core.Notify("Docking goes on: the way is free again");
  NextDockingTry(core);
}

void DockingPart::CancelDockingLeg(CoreAccess core, const Reason& why) {
  docking_leg_due_.reset();
  if (!docking_leg_) return;
  docking_leg_.reset();
  docking_hold_ = false;
  core.CancelDrive(why);
  core.effects().SetDue<StopRequest>(true);
}

void DockingPart::Docked(CoreAccess core) {
  charge_watch_.Confirm();
  core.AddEvent("DOCKED", {{"guide", Guide()}});
  core.Notify("Docked");
  if (Series()) {
    TestRunEnded(core, Reason());
    if (test_run_ < test_runs_) return DriveOutForNextRun(core);
  }
  EndDocking(core, Reason("STOPPED", "Docked"));
}

void DockingPart::EndDocking(CoreAccess core, const Reason& why) {
  // A run of a series ended on its way by a stop of any kind.
  TestRunEnded(core, Reason("STOPPED", why.message));
  CancelDockingLeg(core, why);
  docking_failure_.reset();
  docking_gps_wait_ = false;
  docking_obstacle_wait_ = false;
  docking_charge_.reset();
  docking_out_after_charge_off_.reset();
  AllowChargeAgain(core);
  EndMarkerDocking();
  core.SetState(State::kIdle);
}
}  // namespace mowbite_logic
