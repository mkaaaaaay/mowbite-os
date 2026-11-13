#include <algorithm>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include "mowbite_logic/logic_core.h"

namespace mowbite_logic {

void LogicCore::OnCalibrationStatus(const CalibrationStatus& status, double now_s) {
  calibration_status_ = status;
  calibration_status_s_ = now_s;
}

std::optional<CalibrationStatus> LogicCore::CalibrationStatusNow(double now_s) const {
  if (!calibration_status_ || now_s - calibration_status_s_ > config_.calibration.status_timeout) return std::nullopt;
  return calibration_status_;
}

std::vector<Reason> LogicCore::CalibrationProblems(double now_s) const {
  std::vector<Reason> problems;
  // One reason of each name; the first one found tells best.
  const auto add = [&problems](const Reason& why) {
    if (why.empty()) return;
    for (const auto& known : problems) {
      if (known.code == why.code) return;
    }
    problems.push_back(why);
  };
  add(Busy());
  add(HoldProblem());
  add(Unsafe());
  if (DockContact()) add(Reason("ON_DOCK", "The mower is on the docking station: calibrate on the lawn"));
  if (!pose_) {
    add(Reason("NO_POSITION", "No position from the localization"));
  } else {
    const double limit = std::min(config_.calibration.max_position_accuracy, config_.max_position_accuracy);
    if (!(pose_->position_accuracy <= limit)) {
      Reason why = AccuracyReason(pose_->position_accuracy, limit);
      why.message += " for a calibration";
      add(why);
    }
    if (!pose_->orientation_valid) add(Reason("NO_HEADING", "The heading is not aligned yet"));
  }
  if (!hw_.battery_ok) {
    add(Reason("BATTERY_LOW", "No battery level from the driver"));
  } else if (hw_.battery_percentage < config_.calibration.min_battery) {
    char text[120];
    std::snprintf(text, sizeof(text), "Battery %d %%, at least %d %% for a calibration", hw_.battery_percentage,
                  config_.calibration.min_battery);
    add(Reason("BATTERY_LOW", text)
            .With("battery_percentage", static_cast<int>(hw_.battery_percentage))
            .With("limit", config_.calibration.min_battery));
  }
  add(drivable_problem_);
  const auto status = CalibrationStatusNow(now_s);
  if (!status) {
    add(Reason("CALIBRATION_MISSING", "The calibration program is not running"));
    return problems;
  }
  if (status->running && state_ != State::kCalibrating) {
    add(Reason("STILL_STOPPING", "The last calibration drive is still stopping, try again in a moment"));
  }
  for (const auto& why : status->problems) add(why);
  if (status->lane_length <= 0) add(Reason("NO_ROOM", "No room for a calibration drive here"));
  return problems;
}

Reason LogicCore::StartCalibration(double now_s) {
  CommandCancels();
  const std::vector<Reason> problems = CalibrationProblems(now_s);
  if (!problems.empty()) {
    RefuseCalibration(problems.front());
    return problems.front();
  }
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access): no problems means CalibrationProblems found a status
  const CalibrationStatus status = *CalibrationStatusNow(now_s);
  Replace(CalibrationGoal{status.heading, status.lane_length});
  Drop<CalibrationCancel>();
  calibration_sign_s_ = now_s;
  calibration_phase_ = "start";
  calibration_index_ = 0;
  home_.Reset();
  SetState(State::kCalibrating);
  AddEvent("CALIBRATION", {{"phase", "start"}, {"index", 0}});
  Notify("Calibration: lanes of " + Metres(status.lane_length) + " m, heading " +
                     Degrees(status.heading) + " deg");
  return {};
}

void LogicCore::RefuseCalibration(const Reason& why) {
  CommandCancels();
  json details = json::object();
  PutReason(&details, "CALIBRATION_REFUSED", why);
  AddEvent("CALIBRATION_REFUSED", std::move(details));
}

void LogicCore::OnCalibrationSign(double now_s) {
  if (state_ == State::kCalibrating) calibration_sign_s_ = now_s;
}

void LogicCore::OnCalibrationFeedback(const std::string& phase, int index, double now_s) {
  if (state_ != State::kCalibrating) return;
  calibration_sign_s_ = now_s;
  if (phase == calibration_phase_ && index == calibration_index_) return;
  calibration_phase_ = phase;
  calibration_index_ = index;
  AddEvent("CALIBRATION", {{"phase", phase}, {"index", index}});
}

void LogicCore::OnCalibrationEnded(const CalibrationOutcome& outcome) {
  if (state_ != State::kCalibrating) return;
  if (outcome.reason == "DONE") {
    AddEvent("CALIBRATED", {{"status", outcome.status},
                            {"summary", outcome.summary},
                            {"severity", outcome.status == "done" ? "info" : "warning"}});
    Notify("Calibration done: " + outcome.summary);
    EndCalibration(std::nullopt, false);
    return;
  }
  // A program that ends without saying why counts as gone.
  EndCalibration(Reason(outcome.reason.empty() ? "CALIBRATION_MISSING" : outcome.reason, outcome.message), false);
}

bool LogicCore::AbortCalibration(const Reason& why) {
  CommandCancels();
  if (state_ != State::kCalibrating) return false;
  EndCalibration(why, true);
  return true;
}

bool LogicCore::TickCalibration(double now_s) {
  if (state_ != State::kCalibrating) return false;
  const double quiet = now_s - calibration_sign_s_;
  if (quiet > config_.calibration.timeout) {
    char text[96];
    std::snprintf(text, sizeof(text), "No sign of the calibration program for %.1f s", quiet);
    EndCalibration(Reason("CALIBRATION_MISSING", text).With("seconds", quiet).With("limit", config_.calibration.timeout),
                   true);
    return true;
  }
  // The battery rules of mowing: a voltage dips under load.
  home_.OnSample(HomeSampleOf(hw_, now_s));
  const bool critical = home_.Holds(HomeReason::kBatteryCritical);
  if (critical || home_.Holds(HomeReason::kBatteryLow)) {
    const double voltage = critical ? hw_.battery_voltage : home_.WindowVoltage();
    const double limit = critical ? config_.battery_critical_voltage : config_.battery_empty_voltage;
    char text[160];
    std::snprintf(text, sizeof(text), "Battery %.1f V is below %.1f V", voltage, limit);
    EndCalibration(Reason("BATTERY_LOW", text).With("battery_voltage", voltage).With("limit", limit), true);
    return true;
  }
  return false;
}

bool LogicCore::Calibrating() const {
  return state_ == State::kCalibrating;
}

bool LogicCore::CalibrationActive() const {
  return state_ == State::kCalibrating && !emergency_;
}

void LogicCore::EndCalibration(const std::optional<Reason>& failure, bool cancel) {
  if (failure) {
    json details = json::object();
    PutReason(&details, "CALIBRATION_FAILED", *failure);
    AddEvent("CALIBRATION_FAILED", std::move(details));
    Notify("Calibration stopped: " + failure->message);
  }
  AddEvent("CALIBRATION", {{"phase", "end"}, {"index", calibration_index_}});
  if (cancel) SetDue<CalibrationCancel>(true);
  Drop<CalibrationGoal>();
  home_.Reset();
  // One zero follows from leaving the state; the mower stays where it stopped.
  SetState(State::kIdle);
}

}  // namespace mowbite_logic
