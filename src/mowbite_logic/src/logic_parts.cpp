// The calls of LogicCore that belong to one of its parts.
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "mowbite_logic/logic_core.h"

namespace mowbite_logic {

void LogicCore::OnCalibrationStatus(const CalibrationStatus& status, double now_s) {
  calibration_.OnCalibrationStatus(status, now_s);
}

std::vector<Reason> LogicCore::CalibrationProblems(double now_s) const {
  return calibration_.CalibrationProblems(CoreFacts(*this), now_s);
}

std::optional<CalibrationStatus> LogicCore::CalibrationStatusNow(double now_s) const {
  return calibration_.CalibrationStatusNow(CoreFacts(*this), now_s);
}

Reason LogicCore::StartCalibration(double now_s) {
  return calibration_.StartCalibration(CoreAccess(*this), now_s);
}

void LogicCore::RefuseCalibration(const Reason& why) {
  calibration_.RefuseCalibration(CoreAccess(*this), why);
}

void LogicCore::OnCalibrationSign(double now_s) {
  calibration_.OnCalibrationSign(CoreFacts(*this), now_s);
}

void LogicCore::OnCalibrationFeedback(const std::string& phase, int index, double now_s) {
  calibration_.OnCalibrationFeedback(CoreAccess(*this), phase, index, now_s);
}

void LogicCore::OnCalibrationEnded(const CalibrationOutcome& outcome) {
  calibration_.OnCalibrationEnded(CoreAccess(*this), outcome);
}

bool LogicCore::AbortCalibration(const Reason& why) {
  return calibration_.AbortCalibration(CoreAccess(*this), why);
}

bool LogicCore::TickCalibration(double now_s) {
  return calibration_.TickCalibration(CoreAccess(*this), now_s);
}

bool LogicCore::Calibrating() const {
  return state_ == State::kCalibrating;
}

bool LogicCore::CalibrationActive() const {
  return state_ == State::kCalibrating && !emergency_;
}

bool LogicCore::AreaRecording() const {
  return state_ == State::kAreaRecording;
}

void LogicCore::OnMapStatus(MapDatum datum, const std::string& reason) {
  recording_.OnMapStatus(CoreAccess(*this), datum, reason);
}

void LogicCore::OnWheelOdom(double vx, double now_s) {
  recording_.OnWheelOdom(CoreAccess(*this), vx, now_s);
}

void LogicCore::OnMapEditResult(bool ok, const std::string& id, const std::string& code,
                                const std::string& message) {
  recording_.OnMapEditResult(CoreAccess(*this), ok, id, code, message);
}

json LogicCore::MapOverlay() const {
  return recording_.MapOverlay();
}

bool LogicCore::TakeOverlayChanged() {
  return recording_.TakeOverlayChanged();
}

void LogicCore::RecordedBodyKept(int areas, const std::string& code, const std::string& message) {
  recording_.RecordedBodyKept(CoreAccess(*this), areas, code, message);
}

std::optional<std::pair<double, double>> LogicCore::OnTeleop(double vx, double vz, double now_s) {
  return recording_.OnTeleop(CoreFacts(*this), vx, vz, now_s);
}

std::string LogicCore::RefuseReset(const Reason& why) {
  return release_.RefuseReset(CoreAccess(*this), why);
}

void LogicCore::OnReleaseAnswered(double now_s) {
  release_.OnReleaseAnswered(now_s);
}

bool LogicCore::OnStopRelease() {
  return release_.OnStopRelease(CoreFacts(*this));
}

std::string LogicCore::ButtonCommand(Button button, Press press) const {
  if (state_ == State::kSetup || emergency_) return "";
  // Charging only START held counts: the mower starts from the station.
  const bool start = button == Button::kStart && press == Press::kHeld;
  if (hw_.battery_ok && hw_.dock_contact && !start) return "";
  const bool mowing = state_ == State::kMowing;
  const int pause = mowing_.pause();
  if (button == Button::kHome && press == Press::kShort) {
    return mowing ? "mowing.abort" : state_ == State::kIdle ? "docking.start" : "";
  }
  if (start) {
    if (state_ == State::kIdle) return "mowing.start";
    return mowing && (pause & (kPauseWaitsForUser | kPauseObstacle)) ? "mowing.continue" : "";
  }
  if (button == Button::kStart && press == Press::kShort) {
    return mowing && !(pause & kPauseManual) ? "mowing.pause" : "";
  }
  if (button == Button::kPower && press == Press::kHeld) {
    if (mowing) return pause & kPauseManual ? "" : "mowing.pause";
    if (state_ == State::kDocking) return "docking.abort";
    if (state_ == State::kUndocking) return "undocking.abort";
  }
  return "";
}

Reason LogicCore::OnSetupMotion(bool active, const std::string& step, double now_s) {
  return setup_.OnSetupMotion(CoreAccess(*this), active, step, now_s);
}

void LogicCore::OnSetupBlade(const std::string& step, int blade) {
  setup_.OnSetupBlade(CoreFacts(*this), step, blade);
}

bool LogicCore::SetupBladeOnce() const {
  return setup_.SetupBladeOnce(CoreFacts(*this));
}

bool LogicCore::TickSetup(double now_s) {
  return setup_.TickSetup(CoreAccess(*this), now_s);
}

bool LogicCore::InSetup() const {
  return state_ == State::kSetup;
}

bool LogicCore::SetupActive() const {
  return state_ == State::kSetup && !emergency_;
}

Reason LogicCore::StartRun(const ScheduleRun& run, const Reason& node_problem, double now_s) {
  return schedule_.StartRun(CoreAccess(*this), run, node_problem, now_s);
}

void LogicCore::WatchAnnouncement() {
  schedule_.WatchAnnouncement(CoreAccess(*this));
}

void LogicCore::CommandCancels() {
  schedule_.CommandCancels(CoreAccess(*this));
}

void LogicCore::SetRunPhase(RunPhase phase) {
  schedule_.SetRunPhase(CoreAccess(*this), phase);
}

void LogicCore::RunEnds(const std::string& code, const std::string& detail) {
  schedule_.RunEnds(code, detail);
}

void LogicCore::FinishRun() {
  schedule_.FinishRun(CoreAccess(*this));
}

Reason LogicCore::HomeWhy(HomeReason reason) const {
  return home_.HomeWhy(CoreFacts(*this), reason);
}

void LogicCore::SetSunsetSettings(bool dock_at_sunset) {
  home_.SetSunsetSettings(CoreFacts(*this), dock_at_sunset);
}

void LogicCore::TrySunset(double now_s) {
  home_.TrySunset(CoreAccess(*this), now_s);
}

void LogicCore::OnGnssFix(double x, double y) {
  undocking_.OnGnssFix(CoreAccess(*this), x, y);
}

void LogicCore::BeginDockCheck(DockCheck purpose) {
  undocking_.BeginDockCheck(CoreAccess(*this), purpose);
}

void LogicCore::CheckDockBeforeDrive(double x, double y) {
  undocking_.CheckDockBeforeDrive(CoreAccess(*this), x, y);
}

bool LogicCore::DockCheckPending() const {
  return undocking_.DockCheckPending();
}

Reason LogicCore::OnDockCheck(bool ok, double x, double y, float accuracy, const std::string& message) {
  return undocking_.OnDockCheck(CoreAccess(*this), ok, x, y, accuracy, message);
}

Reason LogicCore::RefuseDockCheck(const Reason& why) {
  return undocking_.RefuseDockCheck(CoreAccess(*this), why);
}

void LogicCore::AgeDockPose(double now_s) {
  docking_part_.AgeDockPose(now_s);
}

void LogicCore::AgeObstacles(double now_s) {
  obstacle_part_.Tick(now_s);
  ApplyObstacleHold();
}

bool LogicCore::MowingDockCheck() const {
  return undocking_.dock_check() == DockCheck::kMowing;
}

bool LogicCore::UndockingForDrive() const {
  return undocking_.UndockingForDrive(CoreFacts(*this));
}

void LogicCore::StopUndocking(const std::optional<Reason>& failure, const Reason& why) {
  undocking_.StopUndocking(CoreAccess(*this), failure, why);
}

void LogicCore::AbortUndocking(const Reason& why) {
  undocking_.AbortUndocking(CoreAccess(*this), why);
}

void LogicCore::OnUndockEnded(LegEnd end, const Reason& reason) {
  undocking_.OnUndockEnded(CoreAccess(*this), end, reason);
}

void LogicCore::EndUndocking(const std::optional<Reason>& failure, const Reason& why) {
  undocking_.EndUndocking(CoreAccess(*this), failure, why);
}

Reason LogicCore::DockingProblem() const {
  return docking_part_.DockingProblem(CoreFacts(*this));
}

Reason LogicCore::StartDocking() {
  return docking_part_.StartDocking(CoreAccess(*this));
}

std::optional<DockingLeg> LogicCore::TakeDockingLeg() {
  ApplyObstacleHold();
  return docking_part_.TakeDockingLeg(CoreAccess(*this));
}

void LogicCore::OnDockingLegEnded(LegEnd end, const Reason& reason) {
  docking_part_.OnDockingLegEnded(CoreAccess(*this), end, reason);
}

bool LogicCore::MarkerSource() const {
  return docking_part_.MarkerSource(CoreFacts(*this));
}

bool LogicCore::Guided() const {
  return docking_part_.Guided(CoreFacts(*this));
}

bool LogicCore::MarkerWanted() const {
  return docking_part_.MarkerWanted(CoreFacts(*this));
}

std::optional<double> LogicCore::DockHeadingHint() const {
  return docking_part_.DockHeadingHint(CoreFacts(*this));
}

bool LogicCore::DockFollowerActive() const {
  return docking_part_.DockFollowerActive(CoreFacts(*this));
}

void LogicCore::OnDockMarker(const DockMarkerSample& sample) {
  docking_part_.OnDockMarker(CoreFacts(*this), sample);
}

void LogicCore::OnDockPose(const DockPoseSample& pose) {
  docking_part_.OnDockPose(CoreAccess(*this), pose);
}

void LogicCore::SetTurningRadius(double radius) {
  docking_part_.SetTurningRadius(radius);
}

Reason LogicCore::ApproachProblem() const {
  return docking_part_.ApproachProblem(CoreFacts(*this));
}

void LogicCore::BeginDocking() {
  docking_part_.BeginDocking(CoreAccess(*this));
}

Reason LogicCore::StartDockingTest(const Reason& node_problem, int runs) {
  return docking_part_.StartDockingTest(CoreAccess(*this), node_problem, runs);
}

void LogicCore::WatchMarkerNear() {
  docking_part_.WatchMarkerNear(CoreFacts(*this));
}

void LogicCore::OnDockingContact() {
  docking_part_.OnDockingContact(CoreAccess(*this));
}

void LogicCore::DockingChargeOff(bool ok, const std::string& message) {
  docking_part_.DockingChargeOff(CoreAccess(*this), ok, message);
}

void LogicCore::WatchCharge() {
  docking_part_.WatchCharge(CoreAccess(*this));
}

void LogicCore::CountDockingFailure(Reason why, bool last) {
  docking_part_.CountDockingFailure(CoreAccess(*this), std::move(why), last);
}

void LogicCore::WaitForDockingGps() {
  docking_part_.WaitForDockingGps(CoreAccess(*this));
}

void LogicCore::TickDocking(double now_s) {
  docking_part_.TickDocking(CoreAccess(*this), now_s);
}

void LogicCore::CancelDockingLeg(const Reason& why) {
  docking_part_.CancelDockingLeg(CoreAccess(*this), why);
}

void LogicCore::EndDocking(const Reason& why) {
  docking_part_.EndDocking(CoreAccess(*this), why);
}

JobStep LogicCore::NextJobStep(double now_s) {
  // No leg starts while an obstacle holds the mower.
  ApplyObstacleHold();
  return mowing_.NextJobStep(CoreAccess(*this), now_s);
}

void LogicCore::OnJobPlanned(std::vector<JobPath> paths, const std::string& fingerprint) {
  mowing_.OnJobPlanned(CoreAccess(*this), std::move(paths), fingerprint);
}

void LogicCore::OnJobPlanFailed(const Reason& why) {
  mowing_.OnJobPlanFailed(CoreAccess(*this), why);
}

void LogicCore::OnJobProgress(size_t index) {
  mowing_.OnJobProgress(CoreFacts(*this), index);
}

void LogicCore::OnLegEnded(LegEnd end, size_t index, const Reason& reason) {
  mowing_.OnLegEnded(CoreAccess(*this), end, index, reason);
}

void LogicCore::OnCutAnswer(const CutAnswer& answer) {
  mowing_.OnCutAnswer(CoreAccess(*this), answer);
  ApplyObstacleHold();
}

void LogicCore::ApplySkips() {
  mowing_.ApplySkips(CoreAccess(*this));
}

void LogicCore::Pause(int why, const Reason& reason) {
  mowing_.Pause(CoreAccess(*this), why, reason);
}

void LogicCore::Unpause(int why) {
  mowing_.Unpause(CoreAccess(*this), why);
}

bool LogicCore::LegDrives() const {
  return mowing_.LegDrives();
}

void LogicCore::OnObstacleStatus(const ObstacleSample& sample, double now_s) {
  obstacle_part_.OnStatus(sample, now_s, ObstacleDriving());
  ApplyObstacleHold();
}

void LogicCore::SetObstacleMode(int setting, int mode_max) {
  obstacle_part_.SetMode(setting, mode_max);
  ApplyObstacleHold();
}

bool LogicCore::GateGo() const {
  return !obstacle_part_.Holding();
}

bool LogicCore::GateSlow() const {
  return obstacle_part_.Slow();
}

bool LogicCore::GateWanted() const {
  return obstacle_part_.Armed() &&
         (state_ == State::kMowing || (state_ == State::kDocking && docking_part_.TowardsStation()));
}

bool LogicCore::ObstacleDriving() const {
  return (state_ == State::kMowing && mowing_.LegDrives()) ||
         (state_ == State::kDocking && docking_part_.TowardsStation() && docking_part_.MapLegActive());
}

void LogicCore::ApplyObstacleHold() {
  for (ObstacleNote& note : obstacle_part_.TakeNotes()) effects_.push_back(std::move(note));
  const bool hold = obstacle_part_.Holding();
  if (state_ == State::kMowing) {
    const bool paused = (mowing_.pause() & kPauseObstacle) != 0;
    if (hold && !paused) Pause(kPauseObstacle, obstacle_part_.Why());
    if (!hold && paused) Unpause(kPauseObstacle);
  } else if (state_ == State::kDocking) {
    docking_part_.ObstacleHold(CoreAccess(*this), hold, obstacle_part_.Why());
  }
}

}  // namespace mowbite_logic
