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

bool LogicCore::Guided() const {
  return MarkerSource() && config_.marker.guide != DockGuide::kNone;
}

bool LogicCore::MarkerWanted() const {
  return state_ == State::kDocking && (marker_docking_ || (marker_near_ && !try_by_position_));
}

std::optional<double> LogicCore::DockHeadingHint() const {
  if (!MarkerWanted() || docking_test_ || !dock_ || !PoseGood()) return std::nullopt;
  return std::remainder(pose_->heading - dock_->heading, 2 * M_PI);  // NOLINT(bugprone-unchecked-optional-access)
}

bool LogicCore::DockFollowerActive() const {
  return !emergency_ && state_ == State::kDocking && docking_leg_.has_value() &&
         docking_leg_frame_ == LegFrame::kDock && !docking_hold_;
}

const char* LogicCore::Guide() const {
  return try_by_marker_ ? "marker" : "position";
}

const DockingSequence& LogicCore::Legs() const {
  return marker_docking_ ? marker_legs_ : docking_;
}

void LogicCore::WatchMarkerNear() {
  if (marker_near_ || !Guided() || !PoseGood()) return;
  const Point2 p = docking_.ApproachPoint();
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access): good means there is one
  if (std::hypot(pose_->x - p.x, pose_->y - p.y) <= kMarkerRequestDistance) marker_near_ = true;
}

void LogicCore::ChooseGuide() {
  marker_docking_ = try_by_marker_ = marker_near_ = true;
  WaitForMarker(true);
}

void LogicCore::DockByPosition() {
  marker_docking_ = try_by_marker_ = false;
  try_by_position_ = true;
  docking_gps_wait_ = false;
  const std::string text = "Docking by the position: the dock camera cannot measure the marker";
  json details = {{"cause", "MARKER_UNAVAILABLE"}, {"message", text}, {"severity", "warning"}, {"resume", "auto"}};
  const std::string detail = marker_.Detail();
  if (KnownDetail(detail)) details["detail"] = detail;
  AddEvent("DOCKING_WITHOUT_MARKER", std::move(details));
  Notify(text + (detail.empty() ? "" : " (" + detail + ")"));
  NextDockingTry();
}

std::string LogicCore::MarkerMapCheck(bool gate) {
  const auto& marker = marker_.pose();
  if (!dock_ || !PoseGood() || !marker) return "";
  const PoseSnapshot& map = *pose_;  // NOLINT(bugprone-unchecked-optional-access): good means there is one
  const DockPose by_marker = StationByMarker(map.x, map.y, map.heading, *marker);
  // The position of the map errs by its accuracy on top of the limit of the reference check.
  std::string problem = MarkerMapProblem(by_marker, *dock_, config_.dock_reference_limit + map.position_accuracy,
                                         gate ? kMarkerMapHeading : 0.0);
  if (problem.empty() || !docking_test_) return problem;
  Notify("Docking test: " + problem + ", not checked in the test");
  return "";
}

double LogicCore::MarkerAccuracyLimit(double x, double y) const {
  // As follower and safety filter: the limit of the map pose has no say here.
  return config_.dock_max_position_accuracy + config_.dock_accuracy_per_m * std::hypot(x, y);
}

bool LogicCore::DockPoseGood() const {
  const auto& pose = marker_.pose();
  return pose && marker_.PoseGood(marker_clock_, MarkerAccuracyLimit(pose->x, pose->y));
}

// A sight ends with a good pose once the marker came since it began, in docking.test with its own direction (there is
// no hint there): a pose kept from before does not start a leg.
bool LogicCore::SightDone() const {
  const bool came = docking_test_ ? marker_.seen() : marker_.found();
  return marker_.sight() && came && DockPoseGood();
}

void LogicCore::OnDockMarker(const DockMarkerSample& sample) {
  if (!MarkerWanted()) return;
  marker_clock_ = std::max(marker_clock_, sample.time);
  marker_.OnMarker(sample);
}

void LogicCore::OnDockPose(const DockPoseSample& pose) {
  if (!MarkerWanted()) return;
  marker_clock_ = std::max(marker_clock_, pose.time);
  marker_.OnPose(pose, docking_leg_ == DockingLeg::Kind::kLine && docking_leg_frame_ == LegFrame::kDock);
  // The waits below are those of the marker, not of the map.
  if (!marker_docking_) return;
  // Back after a loss only with every pose good for gps_wait_time, also those between the ticks.
  if (docking_gps_wait_ && !DockPoseGood()) docking_gps_good_since_.reset();
  if (docking_gps_wait_ && docking_gps_since_ && SightDone()) {
    docking_gps_wait_ = false;
    NextDockingTry();
  }
}

Reason LogicCore::StartDockingTest(const Reason& node_problem, int runs) {
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
  marker_docking_ = docking_test_ = try_by_marker_ = true;
  test_runs_ = runs;
  test_run_ = 1;
  test_run_open_ = runs > 1;
  test_next_run_ = false;
  if (runs > 1) Notify("Docking test: " + std::to_string(runs) + " dockings one after the other, run 1");
  BeginDocking();
  return {};
}

void LogicCore::TestRunEnded(const Reason& why) {
  if (!test_run_open_) return;
  test_run_open_ = false;
  json details = {{"run", test_run_}, {"of", test_runs_}, {"ok", why.empty()}};
  if (!why.empty()) {
    Reason run(why.code, why.message);
    if (why.fields.contains("cause")) run.With("cause", why.fields["cause"]);
    PutReason(&details, "DOCKING_TEST_RUN", run);
  }
  AddEvent("DOCKING_TEST_RUN", std::move(details));
}

void LogicCore::DriveOutForNextRun() {
  const std::string next = "run " + std::to_string(test_run_ + 1) + " of " + std::to_string(test_runs_);
  if (!DockPoseGood()) {
    CountDockingFailure(Reason("dock_failed", "Docking test ended before " + next +
                                                  ": no good pose of the marker on the contacts to drive out by")
                            .With("cause", "MARKER_LOST"),
                        true);
    return;
  }
  const DockPoseSample& p = *marker_.pose();  // NOLINT(bugprone-unchecked-optional-access): good means there is one
  ++test_run_;
  test_run_open_ = true;
  test_next_run_ = true;
  marker_started_ = false;
  Notify("Docking test: out to the start of the line, then " + next);
  SetDockingLeg(marker_legs_.Out(p.x, p.y, p.heading, test_start_along_));
}

void LogicCore::NextMarkerTry() {
  if (!DockPoseGood()) {
    WaitForMarker(true);
    return;
  }
  const DockPoseSample& p = *marker_.pose();  // NOLINT(bugprone-unchecked-optional-access): good means there is one
  if (docking_failure_ && !marker_legs_.OnLine(p.x, p.y, p.heading)) {
    CountDockingFailure(*docking_failure_);
    return;
  }
  const std::string mismatch = MarkerMapCheck(false);
  if (!mismatch.empty()) {
    marker_untrusted_ = true;
    DockingTryFailed(Reason("dock_failed", "Docking failed: " + mismatch).With("cause", "MARKER_MISMATCH"));
    return;
  }
  // A way out to the next run of a series that the marker stopped goes on to where the run begins.
  if (test_next_run_) {
    if (marker_legs_.OutLeft(p.x, p.y, test_start_along_)) {
      SetDockingLeg(marker_legs_.Out(p.x, p.y, p.heading, test_start_along_));
    } else {
      test_next_run_ = false;
      WaitForMarker(true);
    }
    return;
  }
  if (docking_test_ && !marker_started_) {
    const std::string problem = TestStartProblem(marker_legs_, config_.marker, p, config_.marker.cross_track / 2);
    if (!problem.empty()) {
      const Reason why = Reason("dock_failed", "Docking test refused: " + problem).With("cause", "TEST_START");
      Notify(why.message);
      CountDockingFailure(why, true);
      return;
    }
    if (test_run_ <= 1) test_start_along_ = std::max(marker_legs_.Along(p.x, p.y), 0.0);
  }
  DockingLeg leg = marker_legs_.FirstLeg(p.x, p.y, p.heading);
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
  // Nothing drives blind, nor by a marker whose scale failed or that does not fit the map (G42).
  if (marker_untrusted_) {
    CountDockingFailure(why, true);
    return;
  }
  const bool out_done = docking_last_kind_ == DockingLeg::Kind::kOut;
  if (DockPoseGood()) {
    const DockPoseSample& p = *marker_.pose();  // NOLINT(bugprone-unchecked-optional-access): good means there is one
    if (!out_done && marker_legs_.OnLine(p.x, p.y, p.heading)) {
      docking_failure_ = why;
      SetDockingLeg(marker_legs_.Out(p.x, p.y, p.heading));
      return;
    }
    CountDockingFailure(why);
    return;
  }
  // Without a good pose of the marker out of docking.test the map may take the mower out; without either it stays.
  if (docking_test_ || !PoseGood()) {
    CountDockingFailure(why, true);
    return;
  }
  marker_docking_ = false;
  const PoseSnapshot& map = *pose_;  // NOLINT(bugprone-unchecked-optional-access): good means there is one
  if (!out_done && docking_.OnLine(map.x, map.y, map.heading)) {
    docking_failure_ = why;
    SetDockingLeg(docking_.Out(map.x, map.y, map.heading));
    return;
  }
  CountDockingFailure(why);
}

void LogicCore::TickMarkerDocking(double now_s) {
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
      if (!docking_test_ && marker_.sight() && config_.marker.guide == DockGuide::kPrefer) {
        DockByPosition();
        return;
      }
      MarkerFailed("MARKER_UNAVAILABLE", "the dock camera cannot measure the marker");
      return;
    case MarkerWatch::Verdict::kNoDirection:
      if (docking_test_) {
        docking_gps_wait_ = false;
        const Reason why = Reason("dock_failed",
                                  "Docking test refused: the marker gives no direction of its own from here: place the "
                                  "mower closer and straighter in front of the docking station")
                               .With("cause", "TEST_START");
        Notify(why.message);
        CountDockingFailure(why, true);
        return;
      }
      [[fallthrough]];
    case MarkerWatch::Verdict::kNotSeen:
      if (marker_.found()) {
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

Reason LogicCore::GateProblem() {
  const auto failed = [](const std::string& text) {
    return text.empty() ? Reason() : Reason("dock_failed", "Docking failed: " + text);
  };
  if (!marker_docking_) {
    if (!pose_) return failed("at the gate there was no position");
    return failed(docking_.CheckGate(docking_.Across(pose_->x, pose_->y), pose_->heading, pose_->position_accuracy,
                                     DockAccuracyLimit()));
  }
  if (!DockPoseGood()) return failed("at the gate the marker gave no good pose");
  // The scale and the map first: a marker of the wrong size or at the wrong place guides nothing, whatever else fails.
  const std::string scale = marker_.ScaleProblem();
  marker_untrusted_ = !scale.empty();
  if (marker_untrusted_) return failed("at the gate " + scale);
  const std::string map = MarkerMapCheck(true);
  marker_untrusted_ = !map.empty();
  if (marker_untrusted_) return failed("at the gate " + map).With("cause", "MARKER_MISMATCH");
  const DockPoseSample& p = *marker_.pose();  // NOLINT(bugprone-unchecked-optional-access): good means there is one
  // Beside the line by the marker beside the axis: the marker yaw turns the station frame about the marker.
  const double side = SideOffset(marker_legs_, config_.marker, p.x, p.y, p.heading);
  return failed(marker_legs_.CheckGate(side, p.heading, p.position_accuracy, MarkerAccuracyLimit(p.x, p.y)));
}

void LogicCore::EndMarkerDocking() {
  marker_docking_ = docking_test_ = marker_started_ = marker_untrusted_ = marker_sight_ = false;
  test_runs_ = test_run_ = 0;
  test_run_open_ = test_next_run_ = false;
  test_start_along_ = 0.0;
  try_by_marker_ = try_by_position_ = marker_near_ = false;
  marker_.Reset();
}

}  // namespace mowbite_logic
