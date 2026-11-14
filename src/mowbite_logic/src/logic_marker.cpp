#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "mowbite_logic/logic_core.h"

namespace mowbite_logic {

namespace {

// Whether the catalog has this word for detail of DOCKING_FAILED: only its names go out.
bool KnownDetail(const std::string& word) {
  const mowbite_api::EventEntry* event = mowbite_api::FindEvent("DOCKING_FAILED");
  if (!event || word.empty()) return false;
  for (const auto& field : event->fields) {
    if (field.name == "detail") return std::find(field.values.begin(), field.values.end(), word) != field.values.end();
  }
  return false;
}

}  // namespace

Reason LogicCore::DockStartProblem() const {
  Reason why = Busy();
  if (why.empty()) why = HoldProblem();
  if (!why.empty()) return why;
  if (!CanDock()) {
    return Reason("NO_CHARGE_CONTACTS", "This mower has no charge contacts to dock onto (geometry.dock_contact_x 0)");
  }
  if (DockContact()) return Reason("ALREADY_THERE", "The mower is on the docking station already");
  return Unsafe();
}

bool LogicCore::MarkerSource() const {
  return config_.marker.range > 0;
}

bool LogicCore::MarkerWanted() const {
  return state_ == State::kDocking && marker_docking_;
}

bool LogicCore::DockFollowerActive() const {
  return !emergency_ && MarkerWanted() && docking_leg_.has_value() && !docking_hold_;
}

const char* LogicCore::Guide() const {
  return marker_docking_ ? "marker" : "position";
}

double LogicCore::MarkerAccuracyLimit(double x, double y) const {
  // As follower and safety filter: the limit of the map pose has no say here.
  return config_.dock_max_position_accuracy + config_.dock_accuracy_per_m * std::hypot(x, y);
}

bool LogicCore::DockPoseGood() const {
  const auto& pose = marker_.pose();
  return pose && marker_.PoseGood(marker_clock_, MarkerAccuracyLimit(pose->x, pose->y));
}

// A sight ends with a good pose once the marker came with its own direction since it began: a pose kept from before
// does not start a leg.
bool LogicCore::SightDone() const {
  return marker_.sight() && marker_.seen() && DockPoseGood();
}

void LogicCore::OnDockMarker(const DockMarkerSample& sample) {
  if (!MarkerWanted()) return;
  marker_clock_ = std::max(marker_clock_, sample.time);
  marker_.OnMarker(sample);
}

void LogicCore::OnDockPose(const DockPoseSample& pose) {
  if (!MarkerWanted()) return;
  marker_clock_ = std::max(marker_clock_, pose.time);
  marker_.OnPose(pose, docking_leg_ == DockingLeg::Kind::kLine);
  if (docking_gps_wait_ && docking_gps_since_ && SightDone()) {
    docking_gps_wait_ = false;
    NextDockingTry();
  }
}

Reason LogicCore::StartDockingTest(const Reason& node_problem) {
  Reason why = DockStartProblem();
  if (why.empty() && !MarkerSource()) {
    why = Reason("MARKER_UNAVAILABLE", "This mower has no dock marker (marker_range 0): no docking test");
  }
  if (why.empty()) why = node_problem;
  if (!why.empty()) {
    RefuseDrive(why);
    return why;
  }
  json details = json::object();
  PutReason(&details, "DOCKING", Reason("TEST", "Docking test by the dock marker, without map and position"));
  AddEvent("DOCKING", std::move(details));
  marker_ = MarkerWatch(config_.marker);
  marker_docking_ = docking_test_ = true;
  BeginDocking();
  return {};
}

void LogicCore::NextMarkerTry() {
  if (!DockPoseGood()) {
    WaitForMarker(true);
    return;
  }
  const DockPoseSample& p = *marker_.pose();  // NOLINT(bugprone-unchecked-optional-access): good means there is one
  if (docking_failure_ && !docking_.OnLine(p.x, p.y, p.heading)) {
    CountDockingFailure(*docking_failure_);
    return;
  }
  if (docking_test_ && !marker_started_) {
    const std::string problem = TestStartProblem(docking_, p.x, p.y, p.heading, config_.marker.cross_track / 2);
    if (!problem.empty()) {
      const Reason why("dock_failed", "Docking test refused: " + problem);
      Notify(why.message);
      CountDockingFailure(why, true);
      return;
    }
  }
  DockingLeg leg = docking_.FirstLeg(p.x, p.y, p.heading);
  // In the station frame there is no way over the areas.
  if (leg.kind == DockingLeg::Kind::kApproach) {
    const Reason why("dock_failed",
                     "Docking failed: by the marker the mower stands neither on the line of the docking station nor "
                     "at its start");
    Notify(why.message);
    CountDockingFailure(why, true);
    return;
  }
  marker_started_ = true;
  docking_checked_ = false;
  SetDockingLeg(std::move(leg));
}

void LogicCore::WaitForMarker(bool sight) {
  WaitForDockingGps();
  marker_sight_ = sight;
}

void LogicCore::MarkerTryFailed(const Reason& why) {
  // Nothing drives blind, nor by a marker whose scale failed or whose source cannot measure (G42).
  if (!DockPoseGood() || marker_untrusted_) {
    CountDockingFailure(why, true);
    return;
  }
  const DockPoseSample& p = *marker_.pose();  // NOLINT(bugprone-unchecked-optional-access): good means there is one
  if (docking_last_kind_ != DockingLeg::Kind::kOut && docking_.OnLine(p.x, p.y, p.heading)) {
    docking_failure_ = why;
    SetDockingLeg(docking_.Out(p.x, p.y, p.heading));
    return;
  }
  CountDockingFailure(why);
}

void LogicCore::TickMarkerDocking(double now_s) {
  marker_clock_ = std::max(marker_clock_, now_s);
  marker_.Expire(now_s);
  if (!docking_gps_wait_) return;
  if (!docking_gps_since_) {
    docking_gps_since_ = now_s;
    marker_.BeginWait(now_s, marker_sight_);
    marker_sight_ = false;
  }
  if (SightDone()) {
    docking_gps_wait_ = false;
    NextDockingTry();
    return;
  }
  if (!marker_.sight() && DockPoseGood()) {
    if (!docking_gps_good_since_) docking_gps_good_since_ = now_s;
    if (now_s - *docking_gps_good_since_ < config_.gps_wait_time) return;
    Notify("Docking goes on: the marker is back");
    docking_gps_wait_ = false;
    NextDockingTry();
    return;
  }
  docking_gps_good_since_.reset();
  char text[200];
  switch (marker_.Check(now_s)) {
    case MarkerWatch::Verdict::kUnavailable:
      MarkerFailed("MARKER_UNAVAILABLE", "the dock camera cannot measure the marker");
      return;
    case MarkerWatch::Verdict::kNotSeen:
      if (marker_.seen()) {
        std::snprintf(text, sizeof(text), "the pose from the marker was not accurate enough within %.0f s",
                      config_.marker.wait);
      } else {
        std::snprintf(text, sizeof(text), "the dock camera saw no marker within %.0f s", config_.marker.wait);
      }
      MarkerFailed("MARKER_NOT_SEEN", text);
      return;
    case MarkerWatch::Verdict::kLost:
      std::snprintf(text, sizeof(text), "the marker was out of sight for %.0f s on the way in", config_.marker.wait);
      MarkerFailed("MARKER_LOST", text);
      return;
    case MarkerWatch::Verdict::kNoDirection: {
      docking_gps_wait_ = false;
      const Reason why("dock_failed",
                       "Docking test refused: the marker gives no direction of its own from here: place the mower "
                       "closer and straighter in front of the docking station");
      Notify(why.message);
      CountDockingFailure(why, true);
      return;
    }
    case MarkerWatch::Verdict::kWaiting:
    default: return;
  }
}

void LogicCore::MarkerFailed(const std::string& cause, const std::string& text) {
  docking_gps_wait_ = false;
  Reason why = Reason("dock_failed", "Docking failed: " + text).With("cause", cause);
  const std::string detail = marker_.Detail();
  if (KnownDetail(detail)) why.With("detail", detail);
  DockingTryFailed(why);
}

std::string LogicCore::GateProblem() {
  if (!marker_docking_) {
    if (!pose_) return "at the gate there was no position";
    return docking_.CheckGate(pose_->x, pose_->y, pose_->heading, pose_->position_accuracy, DockAccuracyLimit());
  }
  if (!DockPoseGood()) return "at the gate the marker gave no good pose";
  // The scale first: a marker of the wrong size guides nothing, whatever else fails here.
  const std::string scale = marker_.ScaleProblem();
  marker_untrusted_ = !scale.empty();
  if (marker_untrusted_) return "at the gate " + scale;
  const DockPoseSample& p = *marker_.pose();  // NOLINT(bugprone-unchecked-optional-access): good means there is one
  return docking_.CheckGate(p.x, p.y, p.heading, p.position_accuracy, MarkerAccuracyLimit(p.x, p.y));
}

void LogicCore::EndMarkerDocking() {
  marker_docking_ = docking_test_ = marker_started_ = marker_untrusted_ = marker_sight_ = false;
  marker_.Reset();
}

}  // namespace mowbite_logic
