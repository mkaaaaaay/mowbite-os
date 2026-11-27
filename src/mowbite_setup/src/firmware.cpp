// The owner measurement of firmware (steps with firmware, stage firmware): while one of its steps runs the program
// sends setup/firmware_measure, and the driver lets it drive and turn the blade within its own limits for firmware it
// neither lists nor has measured. A run is every step of the stage in order; the step firmware ends it, and once
// enough runs passed in a row it writes measured_firmware for exactly this firmware on exactly this device.
#include <algorithm>
#include <cmath>
#include <set>

#include "mowbite_api/catalog.h"
#include "mowbite_api/setup.h"
#include "mowbite_setup/program.h"
#include "program_util.h"

namespace mowbite_setup {

using namespace util;

namespace {

// The driver lets a step measure within this time of its start, its latch released (s).
constexpr double kOpenWait = 5.0;
// The signal goes out this long before the first release: two of it at 4 Hz (s).
constexpr double kOpenLead = 0.5;
// Outside of a step latch the program clears a switched stop or lift this often, as the driver does by itself
// without the measurement (s).
constexpr double kClearEvery = 1.0;
// A clear asked for goes out within this time (s): the signal comes at 4 Hz and the wheels stand.
constexpr double kClearWait = 2.0;
// press of a step latch: the first clear comes this long after the input switched (s).
constexpr double kPressLead = 0.5;
// Two hw/link_stats of one driver this far apart or more count for errors only, not for rates (s).
constexpr double kLinkGap = 5.0;
// Operator errors of one part of one step in a run; one more ends the run without counting it as failed.
constexpr int kOperatorErrors = 2;
// Runs kept for the export; the oldest passed one goes first.
constexpr size_t kKeptRuns = 30;

// What ends a step without a measurement of the device: the user, the app, the driver or the OS, never the board.
bool OperatorError(const std::string& reason) {
  static const std::set<std::string> reasons = {
      "NOT_DONE",      "STOPPED",   "INTERRUPTED", "RELEASED",  "NO_DATA",          "LOGIC_REFUSED",
      "DRIVER_REFUSED", "NOT_OPENED", "RESTARTS_LOW", "EMERGENCY", "NO_RETURN", "FIRMWARE_CHANGED"};
  return reasons.count(reason) > 0;
}

template <typename Sum>
json LinkJson(const Sum& sum) {
  json frames = json::object();
  for (const auto& [module, count] : sum.frames) frames[module] = count;
  json unknown = json::object();
  for (const auto& [id, count] : sum.unknown) unknown[std::to_string(id)] = count;
  return {{"checksum_errors", sum.checksum_errors}, {"length_errors", sum.length_errors},
          {"unknown_frames", sum.unknown_frames},   {"seconds", Rounded(sum.seconds, 10)},
          {"frames", std::move(frames)},            {"unknown", std::move(unknown)}};
}

}  // namespace

void Program::SetMeasureForce(bool force) {
  force_ = force;
}

const Step* Program::FirmwareStep() const {
  for (const Step& step : list_.steps) {
    if (step.check == "firmware") return &step;
  }
  return nullptr;
}

const Step* Program::LinkStep() const {
  for (const Step& step : list_.steps) {
    if (step.check == "link") return &step;
  }
  return nullptr;
}

bool Program::MeasureShown() const {
  if (!capabilities_known_ || !FirmwareStep()) return false;
  // While a run goes on, also when a new driver does not know its firmware yet.
  const json& runs = MeasureRecord().value("runs", json::array());
  if ((run_ && StepOf(*run_).firmware) || (!runs.empty() && runs.back().value("result", "") == "running")) return true;
  const std::string& source = capabilities_.firmware_source;
  return source == "none" || source == "older" || (force_ && (source == "listed" || source == "measured"));
}

bool Program::MeasureOffered() const {
  return MeasureShown() && capabilities_.firmware_source != "older";
}

json& Program::Measure() {
  json& measure = RecordOf(*FirmwareStep()).measure;
  if (!measure.is_object()) {
    measure = {{"passed", 0}, {"needed", 1}, {"failed", 0}, {"written", nullptr}, {"runs", json::array()}};
  }
  return measure;
}

const json& Program::MeasureRecord() const {
  static const json none = json::object();
  const Step* step = FirmwareStep();
  const json& measure = step ? records_.at(step->id).measure : none;
  return measure.is_object() ? measure : none;
}

json* Program::CurrentRun() {
  if (!FirmwareStep()) return nullptr;
  json& runs = Measure()["runs"];
  if (runs.empty() || runs.back().value("result", "") != "running") return nullptr;
  return &runs.back();
}

// The version of a board of hw/capabilities, empty while it is not known.
std::string Program::BoardVersion(const std::string& board) const {
  for (const auto& [name, version] : capabilities_.firmware) {
    if (name == board) return version;
  }
  return "";
}

// "<version>/<version>/<device_hash>" by the boards of the step firmware, in their order.
std::string Program::FirmwareEntry() const {
  std::string entry;
  for (const auto& board : FirmwareStep()->lists.at("boards")) entry += BoardVersion(board) + "/";
  return entry + capabilities_.device_hash;
}

std::vector<Refusal> Program::MeasureProblems(const Step& step) const {
  std::vector<Refusal> problems;
  if (!MeasureOffered()) {
    if (MeasureShown()) {
      problems.push_back(Refuse("FIRMWARE_OLDER", "A board is older than every listed firmware: update it first"));
    } else {
      problems.push_back(Refuse("NOT_OFFERED", "The driver lets this firmware drive, or it is not known yet"));
    }
    return problems;
  }
  if (capabilities_.device_hash.empty()) {
    problems.push_back(Refuse("NO_DEVICE_ID", "hw/capabilities has no device_hash"));
  }
  bool known = true;
  for (const auto& board : FirmwareStep()->lists.at("boards")) known = known && !BoardVersion(board).empty();
  if (!known) problems.push_back(Refuse("NOT_OFFERED", "Not every board reported its firmware yet"));
  // In order: every step with firmware ahead of it passed in the run of now; the first one begins a run.
  const json& runs = MeasureRecord().value("runs", json::array());
  const json run = !runs.empty() && runs.back().value("result", "") == "running" ? runs.back() : json();
  json needs = json::array();
  bool first = true;
  for (const Step& other : list_.steps) {
    if (&other == &step) break;
    if (!other.firmware) continue;
    first = false;
    const bool passed = run.is_object() && run["steps"].contains(other.id) && run["steps"][other.id]["state"] == "ok";
    if (!passed) needs.push_back(other.id);
  }
  if (!first && !needs.empty()) {
    problems.push_back(Refuse("LOCKED", step.id + " needs other steps first", {{"needs", needs}}));
  }
  return problems;
}

bool Program::StartMeasure(const Step& step, double wall) {
  if (CurrentRun()) return false;
  // The blade of the stage is allowed before the run: its restart would cut the run off.
  for (const Step& other : list_.steps) {
    if (other.firmware && IsBlade(other) && AllowBlade(other)) return true;
  }
  for (const Step& other : list_.steps) {
    if (!other.firmware || &other == &step) continue;
    Record& record = RecordOf(other);
    if (other.check == "firmware") {
      if (record.state != "running") record.state = "open";
    } else {
      record = Record();
    }
  }
  json& measure = Measure();
  json& runs = measure["runs"];
  const int n = runs.empty() ? 1 : runs.back().value("n", 0) + 1;
  runs.push_back({{"n", n},
                  {"t0", std::floor(wall)},
                  {"t1", nullptr},
                  {"firmware", FirmwareText()},
                  {"entry", FirmwareEntry()},
                  {"result", "running"},
                  {"reason", nullptr},
                  {"steps", json::object()},
                  {"operator_errors", json::array()},
                  {"link", nullptr}});
  run_link_ = LinkSum();
  link_ids_.clear();
  return false;
}

void Program::NoteMeasured(const Step& step, const std::string& state, const std::string& reason,
                           const std::string& message, const json& fields, double wall) {
  json* run = CurrentRun();
  if (!run) return;
  const Record& record = records_.at(step.id);
  const json part = fields.contains("part") ? fields["part"] : json();
  (*run)["steps"][step.id] = {{"state", state},
                              {"reason", reason.empty() ? json() : json(reason)},
                              {"message", message.empty() ? json() : json(message)},
                              {"part", part},
                              {"attempt", record.attempt},
                              {"t", std::floor(wall)},
                              {"values", record.values},
                              {"rounds", record.rounds}};
  if (state == "ok") return;
  if (state == "failed" && !OperatorError(reason)) {
    EndMeasureRun("failed", step.id + " " + reason, wall);
    return;
  }
  json& errors = (*run)["operator_errors"];
  errors.push_back({{"step", step.id}, {"part", part}, {"reason", reason}, {"t", std::floor(wall)}});
  const auto same = std::count_if(errors.begin(), errors.end(), [&](const json& error) {
    return error["step"] == step.id && error["part"] == part;
  });
  if (same > kOperatorErrors) EndMeasureRun("aborted", "OPERATOR_ERRORS " + step.id, wall);
}

void Program::EndMeasureRun(const std::string& result, const std::string& reason, double wall) {
  json* run = CurrentRun();
  if (!run) return;
  (*run)["result"] = result;
  (*run)["reason"] = reason.empty() ? json() : json(reason);
  (*run)["t1"] = std::floor(wall);
  (*run)["link"] = LinkJson(run_link_);
  json& measure = Measure();
  if (result == "failed") {
    measure["passed"] = 0;
    measure["needed"] = 2;
    measure["failed"] = measure.value("failed", 0) + 1;
    const json entry = (*run)["entry"];
    if (Revoke(entry)) {
      events_.push_back({"SETUP", {{"what", "firmware_revoked"}, {"entry", entry}}});
      WantRestart();
    }
  } else if (result == "passed") {
    measure["passed"] = measure.value("passed", 0) + 1;
  }
  json& runs = measure["runs"];
  while (runs.size() > kKeptRuns) {
    const auto passed = std::find_if(runs.begin(), runs.end(), [](const json& r) { return r["result"] == "passed"; });
    runs.erase(passed != runs.end() ? passed : runs.begin());
  }
}

bool Program::Revoke(const json& entry) {
  const Step* step = FirmwareStep();
  if (!step) return false;
  const std::string& node = step->Word("node");
  json list = layer_.Get(node, mowbite_api::kMeasuredFirmware);
  if (!list.is_array()) return false;
  const auto end = std::remove(list.begin(), list.end(), entry);
  if (end == list.end()) return false;
  list.erase(end, list.end());
  if (list.empty()) {
    layer_.Erase(node, mowbite_api::kMeasuredFirmware);
  } else {
    layer_.Set(node, mowbite_api::kMeasuredFirmware, list);
  }
  layer_dirty_ = true;
  if (Measure()["written"] == entry) Measure()["written"] = nullptr;
  return true;
}

std::optional<MeasureSignal> Program::MeasureNow() const {
  if (!run_) return std::nullopt;
  const Step& step = StepOf(*run_);
  if (!step.firmware || step.check == "firmware") return std::nullopt;
  // A step starts only with all its points confirmed: one that lists blade_removed had it confirmed.
  const bool removed = std::any_of(step.confirm.begin(), step.confirm.end(),
                                   [](const Named& point) { return point.id == "blade_removed"; });
  return MeasureSignal{step.id, capabilities_.firmware, capabilities_.device_hash, clear_seq_, removed};
}

std::optional<std::string> Program::TakeMeasureEnd() {
  std::optional<std::string> step;
  step.swap(measure_end_);
  return step;
}

// The driver latched FIRMWARE_INCOMPATIBLE at its start and keeps the bit in its latch until a release. With the
// signal it opens, and the program releases (ReleaseMeasured); once the emergency is gone the step begins. Listed
// firmware with force has nothing to open.
void Program::TickOpening(double now, double wall) {
  Run& run = *run_;
  if (!Fresh(hardware_at_, now, kNoData)) {
    Fail("NO_DATA", "No hw/status any more", {{"source", "hw/status"}}, now, wall);
    return;
  }
  const uint32_t reason = hardware_.emergency_reason;
  if (reason == 0) {
    BeginMeasured(now, wall);
    return;
  }
  if (now - run.began > kOpenWait) {
    const std::string names = mowbite_api::EmergencyNames(static_cast<int>(reason));
    if ((reason & ~(FirmwareBit() | LatchBit())) != 0) {
      Fail("EMERGENCY", "Emergency " + names, {{"emergency", names}}, now, wall);
    } else {
      Fail("NOT_OPENED", "The driver did not let the measurement drive: " + names, json::object(), now, wall);
    }
    return;
  }
  if (now - run.began >= kOpenLead) ReleaseMeasured(now, "the firmware measurement opened");
}

void Program::BeginMeasured(double now, double wall) {
  Run& run = *run_;
  run.opening = false;
  run.began = now;
  run.wall_began = wall;
  run.link = LinkSum();
  const Step& step = StepOf(run);
  if (IsDrive(step)) {
    BeginDrive(now, wall);
  } else if (!step.parts.empty()) {
    BeginPart(now, wall);
  }
  Changed();
}

// The driver latched the firmware again after the opening: its guard ended the measurement (a stall of its link, a
// lost signal), whatever step of it runs.
bool Program::MeasureClosed() const {
  const Step& step = StepOf(*run_);
  return step.firmware && step.check != "firmware" && (hardware_.emergency_reason & FirmwareBit()) != 0 &&
         (ReturnBits() & FirmwareBit()) == 0;
}

uint32_t Program::ReturnBits() const {
  const Run& run = *run_;
  const Step& step = StepOf(run);
  if (!step.firmware || step.check != "driver_end") return 0;
  const bool returning = run.drive == Drive::kReturn || run.drive == Drive::kQuiet || run.drive == Drive::kLatch ||
                         run.drive == Drive::kRelease;
  return returning ? FirmwareBit() : 0;
}

void Program::AskClear(double now) {
  ++clear_seq_;
  clear_auto_at_ = now;
}

// Releases the latch of the window and the emergencies of the inputs the measurement brought, at most once a second
// and only while no input shows: a release with a flag standing would clear the board as well.
void Program::ReleaseMeasured(double now, const std::string& why) {
  static const uint32_t allowed = FirmwareBit() | LatchBit() | InputBits();
  Run& run = *run_;
  const uint32_t reason = hardware_.emergency_reason;
  if (reason == 0 || (reason & ~allowed) != 0 || Signal("stop") || Signal("lift") || Signal("bumper")) return;
  if (run.open_release >= 0 && now - run.open_release < 1.0) return;
  run.open_release = now;
  release_ = ReleaseRequest{hardware_.latch_count, "setup: " + why};
}

// Without the measurement the driver clears a stop or lift that stays switched after the input; with it, the program
// asks for that clear (setup/firmware_measure clear_seq), also while a step opens. Not in the parts of a step latch,
// which measure exactly that.
void Program::TickClears(double now) {
  if (!run_) return;
  const Step& step = StepOf(*run_);
  if (!step.firmware || step.check == "firmware" || (step.check == "latch" && !run_->opening)) return;
  if (!hardware_.stop && !hardware_.lift) return;
  if (clear_auto_at_ >= 0 && now - clear_auto_at_ < kClearEvery) return;
  AskClear(now);
}

void Program::OnLinkStats(const LinkCounts& counts, double now) {
  if (counts.clear_seq != clear_sent_) {
    clear_sent_ = counts.clear_seq;
    clear_sent_at_ = now;
  }
  LinkSum delta;
  const LinkCounts* last = link_last_ ? &*link_last_ : nullptr;
  bool same = last && counts.checksum_errors >= last->checksum_errors &&
              counts.length_errors >= last->length_errors && counts.unknown_frames >= last->unknown_frames;
  for (const auto& [module, count] : same ? counts.frames : decltype(counts.frames)()) {
    const auto before = last->frames.find(module);
    if (before != last->frames.end() && count < before->second) same = false;
  }
  // A new driver counts from 0: its errors count, its frames only from its next message on.
  const LinkCounts zero;
  const LinkCounts& from = same ? *last : zero;
  delta.checksum_errors = counts.checksum_errors - from.checksum_errors;
  delta.length_errors = counts.length_errors - from.length_errors;
  delta.unknown_frames = counts.unknown_frames - from.unknown_frames;
  for (const auto& [id, count] : counts.unknown) {
    const auto before = from.unknown.find(id);
    const uint64_t was = before == from.unknown.end() ? 0 : before->second;
    if (count > was) delta.unknown[id] = count - was;
  }
  const double dt = same ? counts.stamp - last->stamp : 0;
  if (same && dt > 0 && dt < kLinkGap) {
    delta.seconds = dt;
    for (const auto& [module, count] : counts.frames) {
      const auto before = last->frames.find(module);
      delta.frames[module] = count - (before == last->frames.end() ? 0 : before->second);
    }
  }
  delta.messages = 1;
  const auto add = [&delta](LinkSum& sum) {
    sum.checksum_errors += delta.checksum_errors;
    sum.length_errors += delta.length_errors;
    sum.unknown_frames += delta.unknown_frames;
    sum.seconds += delta.seconds;
    sum.messages += delta.messages;
    for (const auto& [module, count] : delta.frames) sum.frames[module] += count;
    for (const auto& [id, count] : delta.unknown) sum.unknown[id] += count;
  };
  if (CurrentRun()) add(run_link_);
  if (run_ && !run_->opening && StepOf(*run_).check == "link") add(run_->link);
  link_last_ = counts;
  link_at_ = now;
}

json Program::LinkValues(const LinkSum& sum, const Step& step, bool final) const {
  json values = json::array();
  const auto judged = [final](bool ok) { return final ? json(ok) : json(); };
  const struct {
    const char* modules;
    const char* min;
    const char* max;
  } groups[] = {{"fast", "fast_min", "fast_max"}, {"medium", "medium_min", nullptr}, {"slow", "slow_min", nullptr}};
  for (const auto& group : groups) {
    const double min = step.Number(group.min);
    const json max = group.max ? json(step.Number(group.max)) : json();
    for (const auto& module : step.lists.at(group.modules)) {
      const auto found = sum.frames.find(module);
      const double frames = found == sum.frames.end() ? 0.0 : static_cast<double>(found->second);
      const double rate = sum.seconds > 0 ? frames / sum.seconds : 0.0;
      json value = Measured("link", "rate", Rounded(rate, 10), min, max,
                            judged(rate >= min && (max.is_null() || rate <= max.get<double>())));
      value["id"] = module;
      for (auto& language : value["texts"].items()) {
        language.value()["name"] = language.value().value("name", "") + " " + module;
      }
      values.push_back(std::move(value));
    }
  }
  const double checksum = static_cast<double>(sum.checksum_errors);
  const double length = static_cast<double>(sum.length_errors);
  values.push_back(Measured("link", "checksum_errors", checksum, nullptr, step.Number("checksum_max"),
                            judged(checksum <= step.Number("checksum_max"))));
  values.push_back(Measured("link", "length_errors", length, nullptr, step.Number("length_max"),
                            judged(length <= step.Number("length_max"))));
  values.push_back(Measured("link", "unknown_frames", static_cast<double>(sum.unknown_frames), nullptr, nullptr,
                            nullptr));
  return values;
}

std::optional<json> Program::LinkFault(const LinkSum& sum, const Step& step) const {
  for (const auto& value : LinkValues(sum, step, true)) {
    if (value["ok"] != false) continue;
    json fields = {{"value_id", value["id"]}, {"value", value["value"]}, {"unit", value["unit"]}};
    fields["limit"] = !value["min"].is_null() && value["value"] < value["min"] ? value["min"] : value["max"];
    return fields;
  }
  return std::nullopt;
}

void Program::TickLink(double now, double wall) {
  Run& run = *run_;
  const Step& step = StepOf(run);
  if (!Fresh(hardware_at_, now, kNoData)) {
    Fail("NO_DATA", "No hw/status any more", {{"source", "hw/status"}}, now, wall);
    return;
  }
  if (!Fresh(link_at_, now, kNoData)) {
    Fail("NO_DATA", "No hw/link_stats", {{"source", "hw/link_stats"}}, now, wall);
    return;
  }
  for (const char* group : {"stop", "lift", "bumper"}) {
    if (!Signal(group)) continue;
    Fail("WRONG_INPUT", std::string("The input ") + group + " switched with nothing touched", {{"seen", group}}, now,
         wall);
    return;
  }
  if (hardware_.emergency_reason != 0) {
    const std::string names = mowbite_api::EmergencyNames(static_cast<int>(hardware_.emergency_reason));
    Fail("EMERGENCY", "Emergency " + names, {{"emergency", names}}, now, wall);
    return;
  }
  if (now - run.began < step.Number("time")) return;
  if (const auto fault = LinkFault(run.link, step)) {
    Fail("LIMIT", (*fault)["value_id"].get<std::string>() + " is not as it has to be", *fault, now, wall);
    return;
  }
  link_ids_.clear();
  for (const auto& [id, count] : run.link.unknown) link_ids_.insert(id);
  Pass(step, now, wall);
}

// A part of a step latch; see the check in the catalog. The program sends no clear but its own in it.
void Program::TickLatch(double now, double wall) {
  Run& run = *run_;
  const Step& step = StepOf(run);
  const Part& part = step.parts[run.part];
  if (!Fresh(hardware_at_, now, kNoData)) {
    Fail("NO_DATA", "No hw/status any more", {{"source", "hw/status"}}, now, wall);
    return;
  }
  const bool signal = Signal(part.role);
  const std::string group = GroupOf(part.role);
  for (const char* other : {"stop", "lift", "bumper"}) {
    // Lifting the front may touch the bumper.
    if (other == group || (group == "lift" && std::string(other) == "bumper") || !Signal(other)) continue;
    Fail("WRONG_INPUT", std::string("The input ") + other + " switched, not " + part.role, {{"seen", other}}, now,
         wall);
    return;
  }
  const auto limit = [&](const std::string& message, double value, double bound) {
    Fail("LIMIT", message, {{"value_id", part.id}, {"value", Rounded(value, 100)}, {"limit", bound}, {"unit", "s"}},
         now, wall);
  };
  // The clear of the part went out: since when, below 0 not yet.
  const double out = clear_sent_ >= run.clear && run.clear != 0 ? std::max(clear_sent_at_, run.drive_began) : -1;
  const bool cleared = out >= 0 && run.released >= out;
  switch (run.latch) {
    case Latch::kRest:
      if (part.mode == "release") {
        run.latch = Latch::kSettle;
        return;
      }
      if (!signal) ReleaseMeasured(now, "a part of " + step.id + " was checked");
      if (now - run.part_began > step.Number("time")) {
        Fail("NOT_DONE", "The input " + part.role + " did not switch in time", json::object(), now, wall);
      } else if (!run.armed) {
        run.armed = !signal;
      } else if (signal) {
        run.edge = now;
        run.released = -1;
        run.latch = Latch::kHeld;
      }
      return;
    case Latch::kHeld:
      if (!signal) {
        const double held = run.released - run.edge;
        if (part.mode == "short" && part.role != "stop" && held > step.Number("fall_max")) {
          Fail("NOT_DONE", "The input " + part.role + " stood " + std::to_string(held) + " s: pressed too long?",
               json::object(), now, wall);
        } else if (part.mode == "short" && part.role != "stop") {
          // A side of the bumper is not held by the board: it falls with the input, no clear needed.
          run.measured[run.part] = held;
          LatchPartDone(now, wall);
        } else {
          limit("The input " + part.role + " fell without a clear", held,
                part.mode == "short" ? step.Number("hold_min") : 0);
        }
        return;
      }
      if (part.mode == "short" && part.role != "stop") {
        if (now - run.edge > step.Number("time")) {
          limit("The input " + part.role + " did not fall without a clear", now - run.edge, step.Number("fall_max"));
        }
        return;
      }
      if (part.mode == "short" && now - run.edge >= step.Number("hold_min")) {
        run.measured[run.part] = now - run.edge;
      } else if (part.mode != "press" || now - run.edge < kPressLead) {
        return;
      }
      AskClear(now);
      run.clear = clear_seq_;
      run.drive_began = now;
      run.clears = 1;
      run.latch = Latch::kClear;
      return;
    case Latch::kSettle:
      if (!signal) {
        limit("The input " + part.role + " fell without a clear", now - run.part_began, step.Number("settle"));
      } else if (now - run.part_began >= step.Number("settle")) {
        AskClear(now);
        run.clear = clear_seq_;
        run.drive_began = now;
        run.released = -1;
        run.latch = Latch::kClear;
      }
      return;
    case Latch::kClear:
      if (out < 0) {
        if (now - run.drive_began > kClearWait) {
          Fail("NO_DATA", "The clear did not go out", {{"source", "hw/link_stats"}}, now, wall);
        }
        return;
      }
      if (part.mode == "press") {
        // Held: every clear leaves it switched.
        if (!signal || cleared) {
          limit("A clear ended the held input " + part.role, std::max(0.0, run.released - out), 0);
          return;
        }
        if (now - out < step.Number("clear_every")) return;
        if (run.clears >= std::lround(step.Number("clears"))) {
          run.measured[run.part] = run.clears;
          LatchPartDone(now, wall);
          return;
        }
        AskClear(now);
        run.clear = clear_seq_;
        run.drive_began = now;
        ++run.clears;
        return;
      }
      if (cleared) {
        const double took = run.released - out;
        if (took > step.Number("clear_max")) {
          limit("The clear ended " + part.role + " only after " + std::to_string(took) + " s", took,
                step.Number("clear_max"));
          return;
        }
        if (part.mode == "release") run.measured[run.part] = took;
        LatchPartDone(now, wall);
      } else if (signal && now - out > step.Number("clear_max")) {
        // The board kept it: still held or the mower still lifted, the board alone cannot tell.
        Fail("NOT_DONE", "The input " + part.role + " stayed after the clear: still pressed?", json::object(), now,
             wall);
      }
      return;
  }
}

void Program::LatchPartDone(double now, double wall) {
  Run& run = *run_;
  const Step& step = StepOf(run);
  run.part_states[run.part] = "ok";
  if (run.part + 1 < step.parts.size()) {
    ++run.part;
  } else if (!NextRound(now, wall)) {
    Pass(step, now, wall);
    return;
  }
  BeginPart(now, wall);
  run.latch = Latch::kRest;
  run.armed = false;
  run.clear = 0;
  run.clears = 0;
  run.released = -1;
}

json Program::LatchValues(const Run& run, const Step& step) const {
  json values = json::array();
  for (size_t i = 0; i < step.parts.size(); ++i) {
    const Part& part = step.parts[i];
    const auto& measured = run.measured[i];
    const std::string& state = run.part_states[i];
    const json ok = state == "ok" ? json(true) : state == "failed" ? json(false) : json();
    const bool press = part.mode == "press";
    const json min = press ? json(step.Number("clears")) : part.role == "stop" && part.mode == "short"
                                                               ? json(step.Number("hold_min"))
                                                               : json();
    const json max = part.mode == "release" ? json(step.Number("clear_max")) : json();
    values.push_back(Value(part.id, measured ? json(Rounded(*measured, 100)) : json(), press ? "" : "s", min, max, ok,
                           part.texts));
  }
  return values;
}

int Program::Rounds(const Step& step) const {
  return step.numbers.count("rounds") && step.check != "silence" ? static_cast<int>(std::lround(step.Number("rounds")))
                                                                 : 1;
}

bool Program::NextRound(double now, double wall) {
  Run& run = *run_;
  const Step& step = StepOf(run);
  if (run.round + 1 >= Rounds(step)) return false;
  run.rounds.push_back({{"round", run.round + 1}, {"values", Values(step, now, true)}});
  ++run.round;
  run.part = 0;
  std::fill(run.part_states.begin(), run.part_states.end(), "open");
  std::fill(run.measured.begin(), run.measured.end(), std::nullopt);
  run.stands.clear();
  run.returned_after.reset();
  (void)wall;
  return true;
}

// The step firmware: the run is over; it counts once the link held over all of it.
void Program::BeginFirmware(const Step& step, double now, double wall) {
  json* run = CurrentRun();
  if (!run || (*run)["entry"] != FirmwareEntry()) {
    EndMeasureRun("aborted", "FIRMWARE_CHANGED", wall);
    Fail("FIRMWARE_CHANGED", "The firmware or the device id changed during the run", json::object(), now, wall);
    return;
  }
  const Step* link = LinkStep();
  if (const auto fault = LinkFault(run_link_, *link)) {
    EndMeasureRun("failed", "link " + (*fault)["value_id"].get<std::string>(), wall);
    Fail("LIMIT", (*fault)["value_id"].get<std::string>() + " over the whole run is not as it has to be", *fault, now,
         wall);
    return;
  }
  for (const auto& [id, count] : run_link_.unknown) {
    if (link_ids_.count(id)) continue;
    EndMeasureRun("failed", "link unknown_new", wall);
    Fail("LIMIT", "Frames of the unknown module " + std::to_string(id) + " came after the step link",
         {{"value_id", "unknown_new"}, {"value", id}, {"limit", nullptr}, {"unit", ""}}, now, wall);
    return;
  }
  EndMeasureRun("passed", "", wall);
  json& measure = Measure();
  const int passed = measure.value("passed", 0);
  const int needed = measure.value("needed", 1);
  Record& record = RecordOf(step);
  if (passed < needed) {
    record.values = FirmwareValues(step);
    EndRun(now);
    End(step, "open", "RUN_AGAIN",
        "The run passed; " + std::to_string(needed - passed) + " more in a row are needed after a run that failed",
        {{"passed", passed}, {"needed", needed}}, wall);
    return;
  }
  if (capabilities_.firmware_source != "none") {
    // Forced on listed or measured firmware: measured and kept for the export, nothing written.
    Pass(step, now, wall);
    return;
  }
  const std::string& node = step.Word("node");
  const json before = layer_.Get(node, mowbite_api::kMeasuredFirmware);
  json list = before.is_array() ? before : json::array();
  const std::string entry = FirmwareEntry();
  if (std::find(list.begin(), list.end(), entry) == list.end()) list.push_back(entry);
  layer_.Set(node, mowbite_api::kMeasuredFirmware, list);
  record.applied = {{"entry", entry}, {"before", before}};
  measure["written"] = entry;
  WantRestart();
}

void Program::TickFirmware(double now, double wall) {
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
  if (!capabilities_known_ || capabilities_.firmware_source.empty()) {
    if (now - run.began > restart_time) {
      Fail("NO_DATA", "No firmware_source in hw/capabilities after the restart", {{"source", "hw/capabilities"}}, now,
           wall);
    }
    return;
  }
  if (capabilities_.firmware_source != "measured") {
    Fail("NOT_APPLIED", "hw/capabilities shows firmware_source " + capabilities_.firmware_source + ", not measured",
         json::object(), now, wall);
    return;
  }
  events_.push_back({"SETUP", {{"what", "firmware_measured"}, {"entry", RecordOf(step).applied.value("entry", "")}}});
  Pass(step, now, wall);
}

json Program::FirmwareValues(const Step& step) const {
  const json& measure = MeasureRecord();
  const int passed = measure.value("passed", 0);
  const int needed = measure.value("needed", 1);
  json values = json::array();
  values.push_back(Measured("firmware", "runs", passed, needed, nullptr, passed >= needed));
  if (const Step* link = LinkStep()) {
    for (auto& value : LinkValues(run_link_, *link, true)) values.push_back(std::move(value));
  }
  const json written = records_.at(step.id).applied.is_object() ? records_.at(step.id).applied["entry"]
                                                                : measure.value("written", json());
  values.push_back(Measured("firmware", "written", written.is_string() ? written : json(""), nullptr, nullptr,
                            nullptr));
  return values;
}

void Program::ForgetFirmware(const Step& step) {
  Record& record = RecordOf(step);
  const json before = record.applied.value("before", json());
  record.applied = nullptr;
  restart_wanted_ = false;
  const std::string& node = step.Word("node");
  if (before.is_null()) {
    layer_.Erase(node, mowbite_api::kMeasuredFirmware);
  } else {
    layer_.Set(node, mowbite_api::kMeasuredFirmware, before);
  }
  layer_dirty_ = true;
  Measure()["written"] = nullptr;
}

bool Program::SilenceLastSetpoint() const {
  return run_ && StepOf(*run_).check == "silence" && StepOf(*run_).Word("last_setpoint") == "yes";
}

json Program::Export() const {
  const json& measure = MeasureRecord();
  return {{"platform", platform_},
          {"api", mowbite_api::kApi},
          {"catalog", list_.catalog},
          {"firmware_source", capabilities_known_ ? json(capabilities_.firmware_source) : json()},
          {"device_hash", capabilities_.device_hash},
          {"passed", measure.value("passed", 0)},
          {"needed", measure.value("needed", 1)},
          {"failed", measure.value("failed", 0)},
          {"written", measure.value("written", json())},
          {"runs", measure.value("runs", json::array())}};
}

}  // namespace mowbite_setup
