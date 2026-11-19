#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "mowbite_logic/docking_part.h"

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

Reason DockingPart::DockStartProblem(CoreFacts core) const {
  Reason why = core.Busy();
  if (why.empty()) why = core.HoldProblem();
  if (!why.empty()) return why;
  if (!core.CanDock()) {
    return Reason("NO_CHARGE_CONTACTS", "This mower has no charge contacts to dock onto (geometry.dock_contact_x 0)");
  }
  if (core.DockContact()) return Reason("ALREADY_THERE", "The mower is on the docking station already");
  return core.Unsafe();
}

bool DockingPart::MarkerSource(CoreFacts core) const {
  return core.config().marker.range > 0;
}

bool DockingPart::Guided(CoreFacts core) const {
  return MarkerSource(core) && core.config().marker.guide != DockGuide::kNone;
}

bool DockingPart::MarkerWanted(CoreFacts core) const {
  return core.state() == State::kDocking && (marker_docking_ || (marker_near_ && !try_by_position_));
}

std::optional<double> DockingPart::DockHeadingHint(CoreFacts core) const {
  if (!MarkerWanted(core) || docking_test_ || !core.dock() || !core.PoseGood()) return std::nullopt;
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  return std::remainder(core.pose()->heading - core.dock()->heading, 2 * M_PI);
}

bool DockingPart::DockFollowerActive(CoreFacts core) const {
  return !core.emergency() && core.state() == State::kDocking && docking_leg_.has_value() &&
         docking_leg_frame_ == LegFrame::kDock && !docking_hold_;
}

const char* DockingPart::Guide() const {
  return try_by_marker_ ? "marker" : "position";
}

const DockingSequence& DockingPart::Legs() const {
  return marker_docking_ ? marker_legs_ : docking_;
}

void DockingPart::WatchMarkerNear(CoreFacts core) {
  if (marker_near_ || !Guided(core) || !core.PoseGood()) return;
  const Point2 p = docking_.ApproachPoint();
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access): good means there is one
  if (std::hypot(core.pose()->x - p.x, core.pose()->y - p.y) <= kMarkerRequestDistance) marker_near_ = true;
}

void DockingPart::ChooseGuide(CoreAccess core) {
  marker_docking_ = try_by_marker_ = marker_near_ = true;
  WaitForMarker(core, true);
}

void DockingPart::DockByPosition(CoreAccess core) {
  marker_docking_ = try_by_marker_ = false;
  try_by_position_ = true;
  docking_gps_wait_ = false;
  const std::string text = "Docking by the position: the dock camera cannot measure the marker";
  json details = {{"cause", "MARKER_UNAVAILABLE"}, {"message", text}, {"severity", "warning"}, {"resume", "auto"}};
  const std::string detail = marker_.Detail();
  if (KnownDetail(detail)) details["detail"] = detail;
  core.AddEvent("DOCKING_WITHOUT_MARKER", std::move(details));
  core.Notify(text + (detail.empty() ? "" : " (" + detail + ")"));
  NextDockingTry(core);
}

std::string DockingPart::MarkerMapCheck(CoreAccess core, bool gate) {
  const auto& mower = core.pose();
  const auto& station = core.dock();
  const auto& marker = marker_.pose();
  if (!station || !core.PoseGood() || !marker) return "";
  const PoseSnapshot& map = *mower;  // NOLINT(bugprone-unchecked-optional-access): good means there is one
  const DockPose by_marker = StationByMarker(map.x, map.y, map.heading, *marker);
  // The position of the map errs by its accuracy on top of the limit of the reference check.
  std::string problem = MarkerMapProblem(by_marker, *station,
                                         core.config().dock_reference_limit + map.position_accuracy,
                                         gate ? kMarkerMapHeading : 0.0);
  if (problem.empty() || !docking_test_) return problem;
  core.Notify("Docking test: " + problem + ", not checked in the test");
  return "";
}

double DockingPart::MarkerAccuracyLimit(CoreFacts core, double x, double y) const {
  // As follower and safety filter: the limit of the map pose has no say here.
  return core.config().dock_max_position_accuracy + core.config().dock_accuracy_per_m * std::hypot(x, y);
}

bool DockingPart::DockPoseGood(CoreFacts core) const {
  const auto& pose = marker_.pose();
  return pose && marker_.PoseGood(marker_clock_, MarkerAccuracyLimit(core, pose->x, pose->y));
}

// A sight ends with a good pose once the marker came since it began, in docking.test with its own direction (there is
// no hint there): a pose kept from before does not start a leg.
bool DockingPart::SightDone(CoreFacts core) const {
  const bool came = docking_test_ ? marker_.seen() : marker_.found();
  return marker_.sight() && came && DockPoseGood(core);
}

void DockingPart::OnDockMarker(CoreFacts core, const DockMarkerSample& sample) {
  if (!MarkerWanted(core)) return;
  marker_clock_ = std::max(marker_clock_, sample.time);
  marker_.OnMarker(sample);
}

void DockingPart::OnDockPose(CoreAccess core, const DockPoseSample& pose) {
  if (!MarkerWanted(core)) return;
  marker_clock_ = std::max(marker_clock_, pose.time);
  marker_.OnPose(pose, docking_leg_ == DockingLeg::Kind::kLine && docking_leg_frame_ == LegFrame::kDock);
  // The waits below are those of the marker, not of the map.
  if (!marker_docking_) return;
  // Back after a loss only with every pose good for gps_wait_time, also those between the ticks.
  if (docking_gps_wait_ && !DockPoseGood(core)) docking_gps_good_since_.reset();
  if (docking_gps_wait_ && docking_gps_since_ && SightDone(core)) {
    docking_gps_wait_ = false;
    NextDockingTry(core);
  }
}

Reason DockingPart::StartDockingTest(CoreAccess core, const Reason& node_problem, int runs) {
  Reason why = DockStartProblem(core);
  if (why.empty() && !MarkerSource(core)) {
    why = Reason("MARKER_UNAVAILABLE", "This mower has no dock marker (marker_range 0): no docking test");
  }
  if (why.empty()) why = node_problem;
  if (!why.empty()) {
    core.RefuseDrive(why);
    return why;
  }
  json details = json::object();
  PutReason(&details, "DOCKING", Reason("TEST", "Docking test by the dock marker, without map and position"));
  core.AddEvent("DOCKING", std::move(details));
  marker_.Reset();
  marker_docking_ = docking_test_ = try_by_marker_ = true;
  test_runs_ = runs;
  test_run_ = 1;
  test_run_open_ = runs > 1;
  test_next_run_ = false;
  if (runs > 1) core.Notify("Docking test: " + std::to_string(runs) + " dockings one after the other, run 1");
  BeginDocking(core);
  return {};
}

void DockingPart::TestRunEnded(CoreAccess core, const Reason& why) {
  if (!test_run_open_) return;
  test_run_open_ = false;
  json details = {{"run", test_run_}, {"of", test_runs_}, {"ok", why.empty()}};
  if (!why.empty()) {
    Reason run(why.code, why.message);
    if (why.fields.contains("cause")) run.With("cause", why.fields["cause"]);
    PutReason(&details, "DOCKING_TEST_RUN", run);
  }
  core.AddEvent("DOCKING_TEST_RUN", std::move(details));
}

void DockingPart::DriveOutForNextRun(CoreAccess core) {
  const std::string next = "run " + std::to_string(test_run_ + 1) + " of " + std::to_string(test_runs_);
  if (!DockPoseGood(core)) {
    CountDockingFailure(core, Reason("dock_failed", "Docking test ended before " + next +
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
  core.Notify("Docking test: out to the start of the line, then " + next);
  SetDockingLeg(core, marker_legs_.Out(p.x, p.y, p.heading, test_start_along_));
}

void DockingPart::NextMarkerTry(CoreAccess core) {
  if (!DockPoseGood(core)) {
    WaitForMarker(core, true);
    return;
  }
  const DockPoseSample& p = *marker_.pose();  // NOLINT(bugprone-unchecked-optional-access): good means there is one
  if (docking_failure_ && !marker_legs_.OnLine(p.x, p.y, p.heading)) {
    CountDockingFailure(core, *docking_failure_);
    return;
  }
  const std::string mismatch = MarkerMapCheck(core, false);
  if (!mismatch.empty()) {
    marker_untrusted_ = true;
    DockingTryFailed(core, Reason("dock_failed", "Docking failed: " + mismatch).With("cause", "MARKER_MISMATCH"));
    return;
  }
  // A way out to the next run of a series that the marker stopped goes on to where the run begins.
  if (test_next_run_) {
    if (marker_legs_.OutLeft(p.x, p.y, test_start_along_)) {
      SetDockingLeg(core, marker_legs_.Out(p.x, p.y, p.heading, test_start_along_));
    } else {
      test_next_run_ = false;
      WaitForMarker(core, true);
    }
    return;
  }
  if (docking_test_ && !marker_started_) {
    const std::string problem =
        TestStartProblem(marker_legs_, core.config().marker, p, core.config().marker.cross_track / 2);
    if (!problem.empty()) {
      const Reason why = Reason("dock_failed", "Docking test refused: " + problem).With("cause", "TEST_START");
      core.Notify(why.message);
      CountDockingFailure(core, why, true);
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
    core.Notify(why.message);
    CountDockingFailure(core, why, true);
    return;
  }
  marker_started_ = true;
  docking_checked_ = false;
  SetDockingLeg(core, std::move(leg));
}

void DockingPart::WaitForMarker(CoreAccess core, bool sight) {
  WaitForDockingGps(core);
  marker_sight_ = sight;
}

void DockingPart::MarkerTryFailed(CoreAccess core, const Reason& why) {
  // Nothing drives blind, nor by a marker whose scale failed or that does not fit the map (G42).
  if (marker_untrusted_) {
    CountDockingFailure(core, why, true);
    return;
  }
  const bool out_done = docking_last_kind_ == DockingLeg::Kind::kOut;
  if (DockPoseGood(core)) {
    const DockPoseSample& p = *marker_.pose();  // NOLINT(bugprone-unchecked-optional-access): good means there is one
    if (!out_done && marker_legs_.OnLine(p.x, p.y, p.heading)) {
      docking_failure_ = why;
      SetDockingLeg(core, marker_legs_.Out(p.x, p.y, p.heading));
      return;
    }
    CountDockingFailure(core, why);
    return;
  }
  // Without a good pose of the marker out of docking.test the map may take the mower out; without either it stays.
  if (docking_test_ || !core.PoseGood()) {
    CountDockingFailure(core, why, true);
    return;
  }
  marker_docking_ = false;
  const PoseSnapshot& map = *core.pose();  // NOLINT(bugprone-unchecked-optional-access): good means there is one
  if (!out_done && docking_.OnLine(map.x, map.y, map.heading)) {
    docking_failure_ = why;
    SetDockingLeg(core, docking_.Out(map.x, map.y, map.heading));
    return;
  }
  CountDockingFailure(core, why);
}

void DockingPart::TickMarkerDocking(CoreAccess core, double now_s) {
  marker_.Expire(now_s);
  if (!docking_gps_wait_) return;
  if (!docking_gps_since_) {
    docking_gps_since_ = now_s;
    marker_.BeginWait(now_s, marker_sight_);
    marker_sight_ = false;
  }
  if (SightDone(core)) {
    docking_gps_wait_ = false;
    NextDockingTry(core);
    return;
  }
  if (!marker_.sight() && DockPoseGood(core)) {
    if (!docking_gps_good_since_) docking_gps_good_since_ = now_s;
    if (now_s - *docking_gps_good_since_ < core.config().gps_wait_time) return;
    core.Notify("Docking goes on: the marker is back");
    docking_gps_wait_ = false;
    NextDockingTry(core);
    return;
  }
  docking_gps_good_since_.reset();
  char text[200];
  switch (marker_.Check(now_s)) {
    case MarkerWatch::Verdict::kUnavailable:
      if (!docking_test_ && marker_.sight() && core.config().marker.guide == DockGuide::kPrefer) {
        DockByPosition(core);
        return;
      }
      MarkerFailed(core, "MARKER_UNAVAILABLE", "the dock camera cannot measure the marker");
      return;
    case MarkerWatch::Verdict::kNoDirection:
      if (docking_test_) {
        docking_gps_wait_ = false;
        const Reason why = Reason("dock_failed",
                                  "Docking test refused: the marker gives no direction of its own from here: place the "
                                  "mower closer and straighter in front of the docking station")
                               .With("cause", "TEST_START");
        core.Notify(why.message);
        CountDockingFailure(core, why, true);
        return;
      }
      [[fallthrough]];
    case MarkerWatch::Verdict::kNotSeen:
      if (marker_.found()) {
        std::snprintf(text, sizeof(text), "the pose from the marker was not accurate enough within %.0f s",
                      core.config().marker.wait);
      } else {
        std::snprintf(text, sizeof(text), "the dock camera saw no marker within %.0f s", core.config().marker.wait);
      }
      MarkerFailed(core, "MARKER_NOT_SEEN", text);
      return;
    case MarkerWatch::Verdict::kLost:
      std::snprintf(text, sizeof(text), "the marker was out of sight for %.0f s on the way in",
                    core.config().marker.wait);
      MarkerFailed(core, "MARKER_LOST", text);
      return;
    case MarkerWatch::Verdict::kWaiting:
    default: return;
  }
}

void DockingPart::MarkerFailed(CoreAccess core, const std::string& cause, const std::string& text) {
  docking_gps_wait_ = false;
  Reason why = Reason("dock_failed", "Docking failed: " + text).With("cause", cause);
  const std::string detail = marker_.Detail();
  if (KnownDetail(detail)) why.With("detail", detail);
  DockingTryFailed(core, why);
}

Reason DockingPart::GateProblem(CoreAccess core) {
  const auto& mower = core.pose();
  const auto failed = [](const std::string& text) {
    return text.empty() ? Reason() : Reason("dock_failed", "Docking failed: " + text);
  };
  if (!marker_docking_) {
    if (!mower || !mower->orientation_valid) return failed("at the gate there was no position with a heading");
    return failed(docking_.CheckGate(docking_.Across(mower->x, mower->y), mower->heading,
                                     mower->position_accuracy, core.DockAccuracyLimit()));
  }
  if (!DockPoseGood(core)) return failed("at the gate the marker gave no good pose");
  // The scale and the map first: a marker of the wrong size or at the wrong place guides nothing, whatever else fails.
  const std::string scale = marker_.ScaleProblem();
  marker_untrusted_ = !scale.empty();
  if (marker_untrusted_) return failed("at the gate " + scale);
  const std::string map = MarkerMapCheck(core, true);
  marker_untrusted_ = !map.empty();
  if (marker_untrusted_) return failed("at the gate " + map).With("cause", "MARKER_MISMATCH");
  const DockPoseSample& p = *marker_.pose();  // NOLINT(bugprone-unchecked-optional-access): good means there is one
  // Beside the line by the marker beside the axis: the marker yaw turns the station frame about the marker.
  const double side = SideOffset(marker_legs_, core.config().marker, p.x, p.y, p.heading);
  return failed(marker_legs_.CheckGate(side, p.heading, p.position_accuracy, MarkerAccuracyLimit(core, p.x, p.y)));
}

void DockingPart::EndMarkerDocking() {
  marker_docking_ = docking_test_ = marker_started_ = marker_untrusted_ = marker_sight_ = false;
  test_runs_ = test_run_ = 0;
  test_run_open_ = test_next_run_ = false;
  test_start_along_ = 0.0;
  try_by_marker_ = try_by_position_ = marker_near_ = false;
  marker_.Reset();
}

}  // namespace mowbite_logic
