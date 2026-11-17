#ifndef MOWBITE_LOGIC_DOCKING_PART_H
#define MOWBITE_LOGIC_DOCKING_PART_H

#include <optional>
#include <string>

#include "mowbite_logic/core_access.h"

namespace mowbite_logic {

// DOCKING onto the station of the map or by the dock marker with the series of docking.test, the check of the charge
// on the contacts and the watch of the charge in IDLE (logic_docking.cpp, logic_marker.cpp, logic_charge.cpp). The
// calls are those of LogicCore with the same names.
class DockingPart {
 public:
  explicit DockingPart(const ChargeConfig& charge) : charge_watch_(charge) {}

  void SetTurningRadius(double radius);
  Reason ApproachProblem(CoreFacts core) const;
  Reason DockingProblem(CoreFacts core) const;
  Reason StartDocking(CoreAccess core);
  void BeginDocking(CoreAccess core);
  std::optional<DockingLeg> TakeDockingLeg(CoreAccess core);
  void OnDockingContact(CoreAccess core);
  void OnDockingLegEnded(CoreAccess core, LegEnd end, const Reason& reason);
  // Only while out_after_charge_off() holds the way out; OnChargeOff checks it.
  void DockingChargeOff(CoreAccess core, bool ok, const std::string& message);
  // DOCKING_RETRY and the next try, or DOCKING_FAILED and IDLE (also when last). A copy: why may be the failure kept.
  void CountDockingFailure(CoreAccess core, Reason why, bool last = false);
  void WaitForDockingGps(CoreAccess core);
  void TickDocking(CoreAccess core, double now_s);
  // Stops the leg on its way, if any.
  void CancelDockingLeg(CoreAccess core, const Reason& why);
  // IDLE; a leg on its way stops with why.
  void EndDocking(CoreAccess core, const Reason& why);
  bool MarkerSource(CoreFacts core) const;
  bool Guided(CoreFacts core) const;
  bool MarkerWanted(CoreFacts core) const;
  std::optional<double> DockHeadingHint(CoreFacts core) const;
  bool DockFollowerActive(CoreFacts core) const;
  // Docking asks for the marker once the map puts the mower within kMarkerRequestDistance of the approach point.
  void WatchMarkerNear(CoreFacts core);
  void OnDockMarker(CoreFacts core, const DockMarkerSample& sample);
  void OnDockPose(CoreAccess core, const DockPoseSample& pose);
  // runs 1 to 10: with more than one a series (runs above 1): after each DOCKED out to the start of the line in the
  // station frame and docking again, every run ends with DOCKING_TEST_RUN, the first failure ends the series.
  Reason StartDockingTest(CoreAccess core, const Reason& node_problem, int runs);
  // IDLE on the contacts: DOCKED once the charge is confirmed there, CHARGING_LOST when it stops (ChargeWatch).
  void WatchCharge(CoreAccess core);
  ChargeWatch& charge_watch() {
    return charge_watch_;
  }
  const std::optional<DockingLeg>& out_after_charge_off() const {
    return docking_out_after_charge_off_;
  }
  DockingLeg::Kind last_kind() const {
    return docking_last_kind_;
  }
  // A leg in the map on its way, not stopped at a first contact: the follower drives it.
  bool MapLegActive() const {
    return docking_leg_ && docking_leg_frame_ == LegFrame::kMap && !docking_hold_;
  }
  // A leg in the map on its way or due.
  bool MapLeg() const {
    return (docking_leg_ && docking_leg_frame_ == LegFrame::kMap) ||
           (docking_leg_due_ && docking_leg_due_->frame == LegFrame::kMap);
  }

 private:
  // Docking with the current settings onto the station of the map, the line within marker_range where Guided; in
  // docking.test onto the station of the marker.
  DockingSequence NewDocking(CoreFacts core) const;
  // The first leg of the next try from where the mower stands, or the wait for a good position.
  void NextDockingTry(CoreAccess core);
  // Busy, hold, no charge contacts, on the contacts already, unsafe: what refuses docking.start and docking.test.
  Reason DockStartProblem(CoreFacts core) const;
  bool Series() const {
    return docking_test_ && test_runs_ > 1;
  }
  // The run of the series ends: DOCKING_TEST_RUN, ok without a reason; nothing without an open run.
  void TestRunEnded(CoreAccess core, const Reason& why);
  // After a DOCKED of the series: out of the station to the start of the line for the next run, charging off first;
  // without a good pose of the marker the series ends.
  void DriveOutForNextRun(CoreAccess core);
  // Docking by the marker (logic_marker.cpp): the next try in the station frame or the sight; a failed try, which
  // drives out only with a good pose of the marker; the waits; why the mower may not go on at the gate (also by the
  // map); a try failed by the marker; the end.
  void NextMarkerTry(CoreAccess core);
  // At the approach point of a try that may use the marker: its sight. Where the source cannot measure under prefer:
  // the try by the position, with DOCKING_WITHOUT_MARKER.
  void ChooseGuide(CoreAccess core);
  void DockByPosition(CoreAccess core);
  // The legs of the try: in the station frame by the marker, else in the map.
  const DockingSequence& Legs() const;
  // Why the station by the marker does not fit the one of the map (gate: also the heading), empty if it does or there
  // is no good position to compare; in docking.test only told.
  std::string MarkerMapCheck(CoreAccess core, bool gate);
  void WaitForMarker(CoreAccess core, bool sight);
  void MarkerTryFailed(CoreAccess core, const Reason& why);
  void TickMarkerDocking(CoreAccess core, double now_s);
  Reason GateProblem(CoreAccess core);
  void MarkerFailed(CoreAccess core, const std::string& cause, const std::string& text);
  void EndMarkerDocking();
  // The limit for a pose from the dock marker at (x, y) in the station frame.
  double MarkerAccuracyLimit(CoreFacts core, double x, double y) const;
  bool DockPoseGood(CoreFacts core) const;
  bool SightDone(CoreFacts core) const;
  // marker or position, for the events of docking.
  const char* Guide() const;
  // A try failed: counted at once, or once the mower drove out to the approach point when it stands on the line.
  void DockingTryFailed(CoreAccess core, const Reason& why);
  // The planner gave no answer for the approach of a try: the straight way, or the try failed.
  void DockingWithoutPlanner(CoreAccess core, const Reason& silent);
  // The leg for the node; a way out from the contacts waits until charging is off, where the driver can stop it.
  void SetDockingLeg(CoreAccess core, DockingLeg leg);
  // Once the docking switched charging off: allowed again after the way out and when docking ends.
  void AllowChargeAgain(CoreAccess core);
  // The check of the charge from the debounced contact on, its samples and how it ended.
  void BeginChargeCheck(CoreAccess core);
  ChargeSample ChargeSampleNow(CoreFacts core) const;
  void OnChargeResult(CoreAccess core, ChargeCheck::Result result);
  void ChargingLost(CoreAccess core, const Reason& why);
  void Docked(CoreAccess core);

  // Docking: its tries, the leg due for the node and the kind of the one on its way (and of the last one), stopped at a
  // first contact, a failed try waiting for the mower to drive out, and the wait for a good position.
  DockingSequence docking_;
  std::optional<DockingLeg> docking_leg_due_;
  std::optional<DockingLeg::Kind> docking_leg_;
  DockingLeg::Kind docking_last_kind_ = DockingLeg::Kind::kApproach;
  bool docking_hold_ = false;
  std::optional<Reason> docking_failure_;
  bool docking_gps_wait_ = false;
  std::optional<double> docking_gps_since_;
  std::optional<double> docking_gps_good_since_;
  // The check of the charge on the contacts and whether it ran in this try; the way out kept until charging is off;
  // whether this docking switched charging off (stopped once the driver confirmed it) and has to allow it again.
  std::optional<ChargeCheck> docking_charge_;
  bool docking_checked_ = false;
  std::optional<DockingLeg> docking_out_after_charge_off_;
  bool docking_charge_touched_ = false;
  bool docking_charge_stopped_ = false;
  // Why the planner gave no way for the try that drives straight, and the turning circle of the body for its check.
  std::string docking_direct_cause_;
  double turning_radius_ = 0;
  // Docking by the marker: this docking, the test, whether a leg went out, whether the scale of the marker failed (no
  // drive by it then), whether the next wait is a sight, what the marker told, and the latest monotonic s of it.
  bool marker_docking_ = false;
  bool docking_test_ = false;
  // The series of docking.test: its runs, the one on its way (from the drive out on), whether its event is due, and
  // whether the drive out of the station leads to the next run.
  int test_runs_ = 0;
  int test_run_ = 0;
  bool test_run_open_ = false;
  bool test_next_run_ = false;
  // m along the line from its start where run 1 began: the drive out for the next run ends there, where the marker
  // gave a direction.
  double test_start_along_ = 0.0;
  bool marker_started_ = false;
  bool marker_untrusted_ = false;
  bool marker_sight_ = false;
  // The legs in the station frame; the try chose the marker or the position; docking asks for the marker; the frame of
  // the leg on its way.
  DockingSequence marker_legs_;
  bool try_by_marker_ = false;
  bool try_by_position_ = false;
  bool marker_near_ = false;
  LegFrame docking_leg_frame_ = LegFrame::kMap;
  MarkerWatch marker_;
  double marker_clock_ = 0;
  // Whether charging is confirmed on the station, and its loss.
  ChargeWatch charge_watch_;
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_DOCKING_PART_H
