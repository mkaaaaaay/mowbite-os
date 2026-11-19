// The step summary: once every safety step passed, it writes the date of the safety measurement and the stand time
// without the host into the parameter file of the program, restarts the OS and holds hw/capabilities against it. A
// date the program wrote opens the lock only with a summary that passed: until then a reset in the lock keeps it
// closed.
#include <algorithm>
#include <cmath>
#include <ctime>

#include "mowbite_api/setup.h"
#include "mowbite_setup/program.h"
#include "program_util.h"

namespace mowbite_setup {

using namespace util;

namespace {

// Stand times of the driver against the one written (s): hw/capabilities carries a float32.
constexpr double kFailsafeTolerance = 0.005;

std::string UtcDate(double wall) {
  const time_t seconds = static_cast<time_t>(std::floor(wall));
  struct tm parts {};
  gmtime_r(&seconds, &parts);
  char text[16];
  std::strftime(text, sizeof(text), "%Y-%m-%d", &parts);
  return text;
}

json ValueOf(const json& applied, const char* key) {
  return applied.is_object() && applied.contains(key) ? applied[key] : json();
}

}  // namespace

bool Program::IsSummary(const Step& step) {
  return step.check == "summary";
}

const Step* Program::SummaryStep() const {
  const auto found = std::find_if(list_.steps.begin(), list_.steps.end(), IsSummary);
  return found == list_.steps.end() ? nullptr : &*found;
}

void Program::OnClock(bool ok, const std::string& reason) {
  clock_ok_ = ok;
  clock_reason_ = reason;
}

std::optional<double> Program::LongestSilence() const {
  std::optional<double> longest;
  for (const Step& step : list_.steps) {
    if (step.check != "silence" || records_.at(step.id).state != "ok") continue;
    for (const auto& value : records_.at(step.id).values) {
      if (!value.is_object() || !value.contains("value") || !value["value"].is_number()) continue;
      if (value.value("id", "").rfind("round_", 0) != 0) continue;
      longest = std::max(longest.value_or(0.0), value["value"].get<double>());
    }
  }
  return longest;
}

std::vector<const Step*> Program::SafetyStepsDue() const {
  std::vector<const Step*> due;
  for (const Step& step : list_.steps) {
    if (!mowbite_api::FindSetupCheck(step.check)->safety) continue;
    const bool cause = std::any_of(lock_.causes.begin(), lock_.causes.end(),
                                   [&step](const auto& c) { return !c.reset && c.step == step.id; });
    if (records_.at(step.id).state != "ok" || cause) due.push_back(&step);
  }
  return due;
}

json Program::DimensionsDue() const {
  json due = json::array();
  for (const Step& step : list_.steps) {
    if (step.stage == "dimensions" && records_.at(step.id).state != "ok") due.push_back(step.id);
  }
  return due;
}

std::vector<Refusal> Program::SummaryProblems(double now) const {
  std::vector<Refusal> problems;
  const json dimensions = DimensionsDue();
  if (!dimensions.empty()) {
    problems.push_back(Refuse("LOCKED", "The dimensions come first: " + dimensions.dump(), {{"needs", dimensions}}));
  }
  if (!capabilities_known_ || !Fresh(hardware_at_, now, kFresh)) {
    problems.push_back(Refuse("DRIVER_MISSING", "The driver of the platform reports no status"));
  }
  if (!layer_set_) {
    problems.push_back(Refuse("STORAGE_NOT_WRITABLE", "No data_dir for the date", {{"file", "setup.yaml"}}));
  }
  json steps = json::array();
  json skipped = json::array();
  for (const Step* step : SafetyStepsDue()) {
    steps.push_back(step->id);
    if (records_.at(step->id).state == "skipped") skipped.push_back(step->id);
  }
  // A passed silence without a stand time leaves the stand time to write unknown.
  if (steps.empty() && !LongestSilence()) {
    for (const Step& step : list_.steps) {
      if (step.check == "silence") steps.push_back(step.id);
    }
  }
  if (!steps.empty()) {
    problems.push_back(Refuse("SAFETY_STEPS", "Safety steps are not ok: " + steps.dump(),
                              {{"steps", steps}, {"skipped", skipped}}));
  }
  if (!clock_ok_) {
    problems.push_back(Refuse("CLOCK_UNSAFE", "The clock is not safe: " + clock_reason_,
                              {{"clock_reason", clock_reason_}}));
  }
  if (run_) {
    problems.push_back(Refuse("BUSY", StepOf(*run_).id + " is running", {{"step", StepOf(*run_).id}}));
  } else if (restart_wanted_ || restart_sent_ >= 0) {
    problems.push_back(Refuse("BUSY", "The OS restarts", {{"state", "RESTART"}}));
  } else if (world_.state != "IDLE") {
    problems.push_back(Refuse("BUSY", "The mower is not in IDLE", {{"state", world_.state}}));
  } else if (hardware_.hold) {
    problems.push_back(Refuse("BUSY", "An update holds the mower"));
  } else if (learning_) {
    problems.push_back(Refuse("BUSY", "The driver learns its inputs"));
  }
  if (hardware_.emergency_reason != 0) {
    problems.push_back(Refuse("EMERGENCY", "An emergency stands: the OS restarts only without"));
  }
  return problems;
}

void Program::BeginSummary(const Step& step, double wall) {
  const std::string& node = step.Word("node");
  const std::string date = UtcDate(wall);
  const double failsafe = LongestSilence().value_or(0.0);
  RecordOf(step).applied = {{"date", date},
                            {"failsafe", failsafe},
                            {"before",
                             {{"date", layer_.Get(node, mowbite_api::kSafetyDate)},
                              {"failsafe", layer_.Get(node, mowbite_api::kFailsafeWithoutHost)}}}};
  layer_.Set(node, mowbite_api::kSafetyDate, date);
  layer_.Set(node, mowbite_api::kFailsafeWithoutHost, failsafe);
  // The date the program writes counts only once the step passed: should it not, the reset keeps the lock closed.
  if (!shadowed_.count(node + "." + mowbite_api::kSafetyDate)) CloseWithReset(wall);
  WantRestart();
}

Program::Run Program::SummaryRun(const Step& step, double wall) const {
  Run run;
  run.step = static_cast<size_t>(&step - list_.steps.data());
  run.began = -1;
  run.wall_began = wall;
  run.restarted = true;
  return run;
}

json Program::SummaryValues(const Step& step) const {
  const json& applied = records_.at(step.id).applied;
  return json::array({Measured("summary", "date", ValueOf(applied, "date"), nullptr, nullptr, nullptr),
                      Measured("summary", "failsafe", ValueOf(applied, "failsafe"), nullptr, nullptr, nullptr)});
}

void Program::TickSummary(double now, double wall) {
  Run& run = *run_;
  const Step& step = StepOf(run);
  const double restart_time = step.Number("restart_time");
  if (!run.restarted) {
    if (restart_sent_ >= 0 && now - restart_sent_ > restart_time) {
      restart_sent_ = -1;
      Fail("RESTART_FAILED", "The OS did not restart", {{"limit", restart_time}}, now, wall);
    }
    return;
  }
  if (run.began < 0) run.began = now;
  if (!capabilities_known_) {
    if (now - run.began > restart_time) {
      Fail("NO_DATA", "No hw/capabilities after the restart", {{"source", "hw/capabilities"}}, now, wall);
    }
    return;
  }
  const json& applied = RecordOf(step).applied;
  const std::string& node = step.Word("node");
  const std::string date_key = node + "." + mowbite_api::kSafetyDate;
  const std::string failsafe_key = node + "." + mowbite_api::kFailsafeWithoutHost;
  const std::string& date = capabilities_.safety_measurement_date;
  const json written = ValueOf(applied, "date");
  // What changed between the start and the restart: a safety step or a dimension that is not ok any more.
  const json dimensions = DimensionsDue();
  if (!SafetyStepsDue().empty() || !dimensions.empty()) {
    Fail("NOT_APPLIED", "Safety steps or the dimensions " + dimensions.dump() + " are not ok any more",
         json::object(), now, wall);
    return;
  }
  // A date the file kept by hand sets counts as it is; an empty one keeps the lock closed.
  if (shadowed_.count(date_key) ? date.empty() : written != date) {
    json fields = json::object();
    if (shadowed_.count(date_key)) fields["key"] = date_key;
    Fail("NOT_APPLIED", "hw/capabilities shows the date '" + date + "', not " + written.dump(), std::move(fields), now,
         wall);
    return;
  }
  const json failsafe = ValueOf(applied, "failsafe");
  if (!shadowed_.count(failsafe_key) &&
      (!failsafe.is_number() ||
       std::abs(capabilities_.failsafe_without_host_s - failsafe.get<double>()) > kFailsafeTolerance)) {
    Fail("NOT_APPLIED",
         "hw/capabilities shows failsafe_without_host " + std::to_string(capabilities_.failsafe_without_host_s) +
             ", not " + failsafe.dump(),
         json::object(), now, wall);
    return;
  }
  Pass(step, now, wall);
}

void Program::ForgetSummary(const Step& step) {
  Record& record = RecordOf(step);
  const json before = ValueOf(record.applied, "before");
  record.applied = nullptr;
  restart_wanted_ = false;
  const std::string& node = step.Word("node");
  for (const auto& [key, name] : {std::pair{"date", mowbite_api::kSafetyDate},
                                  std::pair{"failsafe", mowbite_api::kFailsafeWithoutHost}}) {
    const json value = ValueOf(before, key);
    if (value.is_null()) {
      layer_.Erase(node, name);
    } else {
      layer_.Set(node, name, value);
    }
  }
  layer_dirty_ = true;
}

void Program::ReopenSummary() {
  for (const Step& step : list_.steps) {
    if (IsSummary(step) && RecordOf(step).state == "ok") RecordOf(step).state = "open";
  }
}

std::string Program::SafetySource() const {
  const std::string& date = capabilities_.safety_measurement_date;
  if (date.empty()) return "";
  const Step* summary = SummaryStep();
  if (!summary) return "hand";
  const std::string& node = summary->Word("node");
  if (shadowed_.count(node + "." + mowbite_api::kSafetyDate)) return "hand";
  const bool written = layer_.Get(node, mowbite_api::kSafetyDate) == json(date);
  return written || date == reset_date_ ? "program" : "hand";
}

bool Program::EraseSettings() {
  if (!layer_set_) return false;
  bool erased = false;
  for (const Step& step : list_.steps) {
    std::vector<const char*> keys;
    if (IsSummary(step)) keys = {mowbite_api::kSafetyDate, mowbite_api::kFailsafeWithoutHost};
    if (IsBlade(step)) keys = {mowbite_api::kAllowBlade};
    for (const char* key : keys) {
      if (layer_.Get(step.Word("node"), key).is_null()) continue;
      layer_.Erase(step.Word("node"), key);
      erased = layer_dirty_ = true;
    }
  }
  return erased;
}

void Program::ResetSettings(double wall) {
  // The date in the file of the program counts as its own also before hw/capabilities came, unless the file kept by
  // hand overrules it.
  const Step* summary = SummaryStep();
  json written;
  if (summary && !shadowed_.count(summary->Word("node") + "." + mowbite_api::kSafetyDate)) {
    written = layer_.Get(summary->Word("node"), mowbite_api::kSafetyDate);
  }
  const bool program = SafetySource() == "program" || (written.is_string() && !written.get<std::string>().empty());
  settings_due_ = EraseSettings();
  // A date of the file kept by hand: the lock stays as it is.
  if (!program) return;
  reset_date_ = capabilities_.safety_measurement_date.empty() ? written.get<std::string>()
                                                              : capabilities_.safety_measurement_date;
  CloseWithReset(wall);
}

}  // namespace mowbite_setup
