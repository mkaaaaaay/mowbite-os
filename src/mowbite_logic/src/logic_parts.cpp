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

}  // namespace mowbite_logic
