#include <algorithm>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include "mowbite_logic/calibration_part.h"

namespace mowbite_logic {

void CalibrationPart::OnCalibrationStatus(const CalibrationStatus& status, double now_s) {
  calibration_status_ = status;
  calibration_status_s_ = now_s;
}

std::optional<CalibrationStatus> CalibrationPart::CalibrationStatusNow(CoreFacts core, double now_s) const {
  if (!calibration_status_ || now_s - calibration_status_s_ > core.config().calibration.status_timeout) {
    return std::nullopt;
  }
  return calibration_status_;
}

std::vector<Reason> CalibrationPart::CalibrationProblems(CoreFacts core, double now_s) const {
  const auto& pose_now = core.pose();
  std::vector<Reason> problems;
  // One reason of each name; the first one found tells best.
  const auto add = [&problems](const Reason& why) {
    if (why.empty()) return;
    for (const auto& known : problems) {
      if (known.code == why.code) return;
    }
    problems.push_back(why);
  };
  add(core.Busy());
  add(core.HoldProblem());
  add(core.Unsafe());
  if (core.DockContact()) add(Reason("ON_DOCK", "The mower is on the docking station: calibrate on the lawn"));
  if (!pose_now) {
    add(Reason("NO_POSITION", "No position from the localization"));
  } else {
    const double limit =
        std::min(core.config().calibration.max_position_accuracy, core.config().max_position_accuracy);
    if (!(pose_now->position_accuracy <= limit)) {
      Reason why = core.AccuracyReason(pose_now->position_accuracy, limit);
      why.message += " for a calibration";
      add(why);
    }
    if (!pose_now->orientation_valid) add(Reason("NO_HEADING", "The heading is not aligned yet"));
  }
  if (!core.hw().battery_ok) {
    add(Reason("BATTERY_LOW", "No battery level from the driver"));
  } else if (core.hw().battery_percentage < core.config().calibration.min_battery) {
    char text[120];
    std::snprintf(text, sizeof(text), "Battery %d %%, at least %d %% for a calibration", core.hw().battery_percentage,
                  core.config().calibration.min_battery);
    add(Reason("BATTERY_LOW", text)
            .With("battery_percentage", static_cast<int>(core.hw().battery_percentage))
            .With("limit", core.config().calibration.min_battery));
  }
  add(core.drivable_problem());
  const auto status = CalibrationStatusNow(core, now_s);
  if (!status) {
    add(Reason("CALIBRATION_MISSING", "The calibration program is not running"));
    return problems;
  }
  if (status->running && core.state() != State::kCalibrating) {
    add(Reason("STILL_STOPPING", "The last calibration drive is still stopping, try again in a moment"));
  }
  for (const auto& why : status->problems) add(why);
  if (status->lane_length <= 0) add(Reason("NO_ROOM", "No room for a calibration drive here"));
  return problems;
}

Reason CalibrationPart::StartCalibration(CoreAccess core, double now_s) {
  core.CommandCancels();
  const std::vector<Reason> problems = CalibrationProblems(core, now_s);
  if (!problems.empty()) {
    RefuseCalibration(core, problems.front());
    return problems.front();
  }
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access): no problems means CalibrationProblems found a status
  const CalibrationStatus status = *CalibrationStatusNow(core, now_s);
  core.effects().Replace(CalibrationGoal{status.heading, status.lane_length});
  core.effects().Drop<CalibrationCancel>();
  calibration_sign_s_ = now_s;
  calibration_phase_ = "start";
  calibration_index_ = 0;
  core.home().Reset();
  core.SetState(State::kCalibrating);
  core.AddEvent("CALIBRATION", {{"phase", "start"}, {"index", 0}});
  core.Notify("Calibration: lanes of " + Metres(status.lane_length) + " m, heading " +
                     Degrees(status.heading) + " deg");
  return {};
}

void CalibrationPart::RefuseCalibration(CoreAccess core, const Reason& why) {
  core.CommandCancels();
  json details = json::object();
  PutReason(&details, "CALIBRATION_REFUSED", why);
  core.AddEvent("CALIBRATION_REFUSED", std::move(details));
}

void CalibrationPart::OnCalibrationSign(CoreFacts core, double now_s) {
  if (core.state() == State::kCalibrating) calibration_sign_s_ = now_s;
}

void CalibrationPart::OnCalibrationFeedback(CoreAccess core, const std::string& phase, int index, double now_s) {
  if (core.state() != State::kCalibrating) return;
  calibration_sign_s_ = now_s;
  if (phase == calibration_phase_ && index == calibration_index_) return;
  calibration_phase_ = phase;
  calibration_index_ = index;
  core.AddEvent("CALIBRATION", {{"phase", phase}, {"index", index}});
}

void CalibrationPart::OnCalibrationEnded(CoreAccess core, const CalibrationOutcome& outcome) {
  if (core.state() != State::kCalibrating) return;
  if (outcome.reason == "DONE") {
    core.AddEvent("CALIBRATED", {{"status", outcome.status},
                            {"summary", outcome.summary},
                            {"severity", outcome.status == "done" ? "info" : "warning"}});
    core.Notify("Calibration done: " + outcome.summary);
    EndCalibration(core, std::nullopt, false);
    return;
  }
  // A program that ends without saying why counts as gone.
  EndCalibration(core, Reason(outcome.reason.empty() ? "CALIBRATION_MISSING" : outcome.reason, outcome.message), false);
}

bool CalibrationPart::AbortCalibration(CoreAccess core, const Reason& why) {
  core.CommandCancels();
  if (core.state() != State::kCalibrating) return false;
  EndCalibration(core, why, true);
  return true;
}

bool CalibrationPart::TickCalibration(CoreAccess core, double now_s) {
  if (core.state() != State::kCalibrating) return false;
  const double quiet = now_s - calibration_sign_s_;
  if (quiet > core.config().calibration.timeout) {
    char text[96];
    std::snprintf(text, sizeof(text), "No sign of the calibration program for %.1f s", quiet);
    EndCalibration(core,
                   Reason("CALIBRATION_MISSING", text)
                       .With("seconds", quiet)
                       .With("limit", core.config().calibration.timeout),
                   true);
    return true;
  }
  // The battery rules of mowing: a voltage dips under load.
  core.home().OnSample(HomeSampleOf(core.hw(), now_s));
  const bool critical = core.home().Holds(HomeReason::kBatteryCritical);
  if (critical || core.home().Holds(HomeReason::kBatteryLow)) {
    const double voltage = critical ? core.hw().battery_voltage : core.home().WindowVoltage();
    const double limit = critical ? core.config().battery_critical_voltage : core.config().battery_empty_voltage;
    char text[160];
    std::snprintf(text, sizeof(text), "Battery %.1f V is below %.1f V", voltage, limit);
    EndCalibration(core, Reason("BATTERY_LOW", text).With("battery_voltage", voltage).With("limit", limit), true);
    return true;
  }
  return false;
}

void CalibrationPart::EndCalibration(CoreAccess core, const std::optional<Reason>& failure, bool cancel) {
  if (failure) {
    json details = json::object();
    PutReason(&details, "CALIBRATION_FAILED", *failure);
    core.AddEvent("CALIBRATION_FAILED", std::move(details));
    core.Notify("Calibration stopped: " + failure->message);
  }
  core.AddEvent("CALIBRATION", {{"phase", "end"}, {"index", calibration_index_}});
  if (cancel) core.effects().SetDue<CalibrationCancel>(true);
  core.effects().Drop<CalibrationGoal>();
  core.home().Reset();
  // One zero follows from leaving the state; the mower stays where it stopped.
  core.SetState(State::kIdle);
}

}  // namespace mowbite_logic
