#include "mowbite_setup/program.h"

#include <algorithm>
#include <cmath>

#include "mowbite_api/catalog.h"
#include "mowbite_api/setup.h"

namespace mowbite_setup {

namespace {

// A source counts as there with a message this young (s); a running step gives up after kNoData without one.
constexpr double kFresh = 1.0;
constexpr double kNoData = 3.0;
// A step imu takes the gravity of the first half second as the mower at rest; a part begins once it is back within a
// third of the angle asked for.
constexpr double kRestTime = 0.5;
constexpr double kGravityFilter = 0.2;
// Turn rates below this (rad/s) do not count as turning: the drift of a gyro at rest.
constexpr double kTurnFloor = 0.05;
// The rate of the positions is taken over this time (s).
constexpr double kFixWindow = 5.0;
// IDLE off the contacts for this long after docking: the mower did not make it by itself.
constexpr double kDockedWait = 3.0;
// An emergency that is on this long (s) before its input shows was brought by something else.
constexpr double kLatchLead = 0.3;
constexpr double kDegrees = 180.0 / M_PI;

constexpr const char* kStepEvent = "SETUP_STEP";

const char* const kGroups[] = {"stop", "lift", "bumper"};

// Bits of emergency_reason of hw/status the checks ask for, by their names in the catalog.
uint32_t FirmwareBit() {
  static const uint32_t bit = EmergencyBit("FIRMWARE_INCOMPATIBLE");
  return bit;
}

uint32_t TiltBit() {
  static const uint32_t bit = EmergencyBit("TILT");
  return bit;
}

bool IsTurn(const std::string& move) {
  return move.rfind("turn", 0) == 0;
}

std::string GroupOf(const std::string& role) {
  return role.rfind("bumper", 0) == 0 ? "bumper" : role;
}

std::string Joined(const std::set<std::string>& names) {
  std::string text;
  for (const auto& name : names) text += (text.empty() ? "" : ", ") + name;
  return text;
}

double Rounded(double value, double scale) {
  return std::round(value * scale) / scale;
}

// A value of values[]: bounds and ok are null where there is none or it is not judged yet.
json Value(const std::string& id, json value, const std::string& unit, json min, json max, json ok, json texts) {
  return {{"id", id},   {"value", std::move(value)}, {"unit", unit}, {"min", std::move(min)},
          {"max", std::move(max)}, {"ok", std::move(ok)},       {"texts", std::move(texts)}};
}

// A value a check of the catalog measures, by its id, with the unit and the texts from there.
json Measured(const char* check, const std::string& id, json value, json min, json max, json ok) {
  std::string unit;
  json texts = json::object();
  for (const auto& measure : mowbite_api::FindSetupCheck(check)->values) {
    if (measure.id != id) continue;
    unit = measure.unit;
    texts = mowbite_api::Texts(measure.texts);
  }
  return Value(id, std::move(value), unit, std::move(min), std::move(max), std::move(ok), std::move(texts));
}

Refusal Refuse(std::string reason, std::string message, json data = json::object()) {
  return {std::move(reason), std::move(message), std::move(data), ""};
}

Refusal BadParam(std::string field, std::string message) {
  return {"", std::move(message), json::object(), std::move(field)};
}

bool Done(const std::string& state) {
  return state == "ok" || state == "skipped";
}

}  // namespace

Program::Program(StepList list, std::string platform) : list_(std::move(list)), platform_(std::move(platform)) {
  for (const Step& step : list_.steps) records_[step.id] = Record();
}

bool Program::Load(const std::string& text, double wall, std::string* why) {
  const auto fail = [why](const std::string& message) {
    if (why) *why = message;
    return false;
  };
  const json file = json::parse(text, nullptr, false);
  if (!file.is_object()) return fail("it is no JSON object");
  if (!file.contains("schema") || file["schema"] != 1) return fail("it is of another schema");
  if (!file.contains("platform") || file["platform"] != platform_) return fail("it is of another platform");
  if (!file.contains("steps") || !file["steps"].is_object()) return fail("steps is no object");
  std::map<std::string, Record> records;
  for (const Step& step : list_.steps) {
    Record record;
    if (file["steps"].contains(step.id)) {
      const json& saved = file["steps"][step.id];
      if (!saved.is_object()) return fail(step.id + " is no object");
      std::string state = "open";
      if (saved.contains("state")) {
        if (!saved["state"].is_string()) return fail(step.id + " has no state of a step");
        state = saved["state"].get<std::string>();
      }
      if (state != "open" && state != "running" && !Done(state) && state != "failed") {
        return fail(step.id + " has no state of a step");
      }
      record.state = state;
      if (saved.contains("attempt") && saved["attempt"].is_number_integer()) {
        record.attempt = static_cast<int>(std::clamp<int64_t>(saved["attempt"].get<int64_t>(), 0, 1000000000));
      }
      if (saved.contains("result")) record.result = saved["result"];
      if (saved.contains("values") && saved["values"].is_array()) record.values = saved["values"];
      if (saved.contains("parts") && saved["parts"].is_array()) record.parts = saved["parts"];
      if (saved.contains("answers") && saved["answers"].is_object()) record.answers = saved["answers"];
    }
    records[step.id] = std::move(record);
  }
  records_ = std::move(records);
  run_.reset();
  if (file.contains("version") && file["version"].is_number_integer()) {
    version_ = static_cast<int>(std::clamp<int64_t>(file["version"].get<int64_t>(), 0, 1000000000)) + 1;
  }
  for (const Step& step : list_.steps) {
    if (RecordOf(step).state == "running") End(step, "open", "INTERRUPTED", "The OS restarted while it ran", {}, wall);
  }
  return true;
}

std::string Program::Text() const {
  json steps = json::object();
  for (const Step& step : list_.steps) {
    const Record& record = records_.at(step.id);
    steps[step.id] = {{"state", record.state},   {"attempt", record.attempt}, {"result", record.result},
                      {"values", record.values}, {"parts", record.parts},     {"answers", record.answers}};
  }
  const json file = {{"schema", 1},
                     {"platform", platform_},
                     {"catalog", list_.catalog},
                     {"version", version_},
                     {"steps", std::move(steps)}};
  // Texts of a board (firmware) may hold bytes that are no UTF-8.
  return file.dump(2, ' ', false, json::error_handler_t::replace) + "\n";
}

bool Program::Fresh(double last, double now, double limit) {
  return last >= 0 && now - last <= limit;
}

bool Program::IsRun(const Step& step) const {
  return run_ && &StepOf(*run_) == &step;
}

bool Program::SolutionOk(const Step& step) const {
  return solution_ == 4 || (solution_ == 3 && step.Word("solution") == "float");
}

bool Program::Signal(const std::string& role) const {
  if (role == "stop") return hardware_.stop;
  if (role == "lift") return hardware_.lift;
  if (role == "bumper_left") return hardware_.bumper_left;
  if (role == "bumper_right") return hardware_.bumper_right;
  return hardware_.bumper || hardware_.bumper_left || hardware_.bumper_right;
}

void Program::OnHardware(const Hardware& hardware, double now) {
  hardware_ = hardware;
  hardware_at_ = now;
  if (!run_ || StepOf(*run_).check != "board") return;
  ++run_->status_count;
  for (const char* group : kGroups) {
    if (Signal(group)) run_->inputs_seen.insert(group);
  }
  for (const auto& motor : hardware.motors_down) run_->motors_down.insert(motor);
}

void Program::OnCapabilities(const Capabilities& capabilities) {
  capabilities_ = capabilities;
  capabilities_known_ = true;
}

void Program::OnImu(const ImuSample& sample, double now) {
  imu_at_ = now;
  if (!run_) return;
  Run& run = *run_;
  const Step& step = StepOf(run);
  if (step.check == "board") {
    run.gravity_sum += std::sqrt(sample.ax * sample.ax + sample.ay * sample.ay + sample.az * sample.az);
    ++run.gravity_count;
    return;
  }
  if (step.check != "imu") return;
  const double a[3] = {sample.ax, sample.ay, sample.az};
  const double dt = run.last_imu < 0 ? 0.0 : std::clamp(now - run.last_imu, 0.0, 0.1);
  run.last_imu = now;
  if (now - run.began <= kRestTime) {
    for (int i = 0; i < 3; ++i) run.rest[i] += a[i];
    ++run.rest_count;
  }
  if (!run.gravity_known) {
    std::copy(a, a + 3, run.gravity);
    run.gravity_known = true;
  } else {
    const double share = dt / (kGravityFilter + dt);
    for (int i = 0; i < 3; ++i) run.gravity[i] += share * (a[i] - run.gravity[i]);
  }
  if (run.phase == Phase::kArmed && std::abs(sample.gz) > kTurnFloor) run.yaw += sample.gz * dt;
}

void Program::OnFix(int solution, double accuracy, double now) {
  solution_ = solution;
  accuracy_ = accuracy;
  fixes_.push_back(now);
  while (!fixes_.empty() && now - fixes_.front() > kFixWindow) fixes_.pop_front();
}

void Program::OnWorld(const World& world) {
  if (world.state != world_.state) {
    if (world.state == "UNDOCKING") {
      undocked_ = true;
      mowed_ = false;
      docking_ = false;
    } else if (world.state == "MOWING") {
      mowed_ = true;
    } else if (world.state == "DOCKING") {
      docking_ = true;
    } else if (world.state == "AREA_RECORDING") {
      undocked_ = mowed_ = docking_ = false;
    }
    docking_idle_since_ = -1;
  }
  world_ = world;
}

void Program::SetWritable(bool writable) {
  writable_ = writable;
}

// The mower came back onto the contacts by itself: a docking test when it left them by itself before, a trial run when
// it mowed in between.
void Program::TrackWorld(double now) {
  if (!docking_ || world_.state != "IDLE") return;
  if (hardware_.dock_contact) {
    if (undocked_) reached_.insert("dock_test");
    if (mowed_) reached_.insert("mow");
  } else {
    if (docking_idle_since_ < 0) docking_idle_since_ = now;
    if (now - docking_idle_since_ < kDockedWait) return;
  }
  undocked_ = mowed_ = docking_ = false;
  docking_idle_since_ = -1;
}

double Program::FixRate(double now) const {
  int count = 0;
  for (const double at : fixes_) {
    if (now - at <= kFixWindow) ++count;
  }
  return count / kFixWindow;
}

bool Program::NeedsMet(const Step& step) const {
  return std::all_of(step.needs.begin(), step.needs.end(),
                     [this](const std::string& need) { return records_.at(need).state == "ok"; });
}

std::string Program::Shown(const Step& step) const {
  const std::string& state = records_.at(step.id).state;
  return state == "open" && !NeedsMet(step) ? "locked" : state;
}

bool Program::StageDone(const std::string& stage) const {
  for (const Step& step : list_.steps) {
    if (step.stage == stage && !Done(records_.at(step.id).state)) return false;
  }
  return true;
}

bool Program::Complete() const {
  return std::all_of(list_.steps.begin(), list_.steps.end(),
                     [this](const Step& step) { return Done(records_.at(step.id).state); });
}

std::string Program::FirmwareText() const {
  std::string text;
  for (const auto& [board, version] : capabilities_.firmware) {
    text += (text.empty() ? "" : ", ") + board + " " + version;
  }
  return text;
}

void Program::Changed() {
  ++version_;
  changed_ = true;
}

bool Program::TakeChanged() {
  const bool changed = changed_;
  changed_ = false;
  return changed;
}

std::vector<Event> Program::TakeEvents() {
  std::vector<Event> events;
  events.swap(events_);
  return events;
}

// state: ok, failed, skipped, or open for a step that was stopped or cut off, which the event tells as failed.
void Program::End(const Step& step, const std::string& state, const std::string& reason, const std::string& message,
                  json fields, double wall) {
  const bool stage_done = StageDone(step.stage);
  const bool complete = Complete();
  Record& record = RecordOf(step);
  record.state = state;
  const auto field = [&fields](const char* key) { return fields.contains(key) ? fields[key] : json(); };
  record.result = {{"ok", state == "ok"},
                   {"reason", reason.empty() ? json() : json(reason)},
                   {"message", message.empty() ? json() : json(message)},
                   {"part", field("part")},
                   {"skip", field("skip")},
                   {"note", field("note")},
                   {"t", std::floor(wall)},
                   {"attempt", record.attempt},
                   {"firmware", FirmwareText()}};
  json details = {{"step", step.id}, {"state", state == "open" ? "failed" : state}, {"attempt", record.attempt}};
  if (!reason.empty()) {
    const mowbite_api::Rating rating = mowbite_api::RateReason(kStepEvent, reason);
    details["reason"] = reason;
    details["message"] = message;
    details["severity"] = rating.severity;
    details["resume"] = rating.resume;
  }
  for (const auto& item : fields.items()) {
    if (!item.value().is_null()) details[item.key()] = item.value();
  }
  events_.push_back({kStepEvent, std::move(details)});
  if (!stage_done && StageDone(step.stage)) {
    events_.push_back({"SETUP", {{"what", "stage_done"}, {"stage", step.stage}}});
  }
  if (!complete && Complete()) events_.push_back({"SETUP", {{"what", "complete"}}});
  Changed();
}

void Program::Pass(const Step& step, double now, double wall) {
  Record& record = RecordOf(step);
  if (IsRun(step)) {
    record.values = Values(step, now, true);
    record.parts = json(std::vector<std::string>(step.parts.size(), "ok"));
    run_.reset();
  }
  End(step, "ok", "", "", {}, wall);
}

void Program::Fail(const std::string& reason, const std::string& message, json fields, double now, double wall) {
  const Step& step = StepOf(*run_);
  Record& record = RecordOf(step);
  const bool stopped = reason == "STOPPED";
  record.values = Values(step, now, true);
  if (!step.parts.empty()) {
    run_->part_states[run_->part] = stopped ? "open" : "failed";
    record.parts = run_->part_states;
    if (!stopped) fields["part"] = step.parts[run_->part].id;
  }
  run_.reset();
  End(step, stopped ? "open" : "failed", reason, message, std::move(fields), wall);
}

std::optional<Refusal> Program::Common(const json& params, const Step** step) const {
  if (!params.is_object()) return BadParam("params", "The params are no object");
  if (step) {
    if (!params.contains("step") || !params["step"].is_string()) return BadParam("step", "step is missing or no text");
    const std::string id = params["step"].get<std::string>();
    *step = list_.Find(id);
    if (!*step) return Refuse("UNKNOWN_STEP", "No step " + id, {{"step", id}});
  }
  if (!params.contains("version") || !params["version"].is_number_integer()) {
    return BadParam("version", "version is missing or no whole number");
  }
  if (params["version"].get<int64_t>() != version_) {
    return Refuse("VERSION", "The setup is at version " + std::to_string(version_) + " now: read it again",
                  {{"version", version_}});
  }
  if (!writable_) return Refuse("STORAGE_NOT_WRITABLE", "setup.json cannot be written", {{"file", "setup.json"}});
  return std::nullopt;
}

std::optional<Refusal> Program::Start(const json& params, double now, double wall) {
  const Step* found = nullptr;
  if (auto refusal = Common(params, &found)) return refusal;
  const Step& step = *found;
  for (const auto& item : params.items()) {
    if (item.key() != "step" && item.key() != "version" && item.key() != "confirm") {
      return BadParam(item.key(), item.key() + " is no param of setup.start");
    }
  }
  if (!step.start) return Refuse("WRONG_METHOD", step.id + " is not begun with setup.start");
  if (run_) return Refuse("BUSY", StepOf(*run_).id + " is running", {{"step", StepOf(*run_).id}});
  if (!NeedsMet(step)) {
    json needs = json::array();
    for (const auto& need : step.needs) {
      if (records_.at(need).state != "ok") needs.push_back(need);
    }
    return Refuse("LOCKED", step.id + " needs other steps first", {{"needs", needs}});
  }
  if (step.kind == "measure" && !world_.state.empty() && world_.state != "IDLE") {
    return Refuse("BUSY", "The mower is in " + world_.state, {{"state", world_.state}});
  }
  std::set<std::string> confirmed;
  if (params.contains("confirm")) {
    if (!params["confirm"].is_array()) return BadParam("confirm", "confirm is no list");
    for (const auto& point : params["confirm"]) {
      if (!point.is_string()) return BadParam("confirm", "confirm lists names of points");
      confirmed.insert(point.get<std::string>());
    }
  }
  json missing = json::array();
  for (const Named& point : step.confirm) {
    if (!confirmed.count(point.id)) missing.push_back(point.id);
  }
  if (!missing.empty()) return Refuse("CONFIRM_MISSING", "Points to confirm are missing", {{"missing", missing}});
  if ((step.check == "board" || step.check == "input") && !Fresh(hardware_at_, now, kFresh)) {
    return Refuse("DRIVER_MISSING", "The driver of the platform reports no status");
  }

  Record& record = RecordOf(step);
  ++record.attempt;
  record.values = json::array();
  record.parts = json::array();
  if (step.check == "confirm") {
    Pass(step, now, wall);
    return std::nullopt;
  }
  record.state = "running";
  Run run;
  run.step = static_cast<size_t>(&step - list_.steps.data());
  run.began = now;
  run.wall_began = wall;
  run.link_errors_first = hardware_.link_errors;
  run.part_states.assign(step.parts.size(), "open");
  run.measured.assign(step.parts.size(), std::nullopt);
  run_ = std::move(run);
  if (!step.parts.empty()) BeginPart(now, wall);
  Changed();
  return std::nullopt;
}

std::optional<Refusal> Program::Answer(const json& params, double wall) {
  const Step* found = nullptr;
  if (auto refusal = Common(params, &found)) return refusal;
  const Step& step = *found;
  if (step.check != "choice" && step.check != "number") {
    return Refuse("WRONG_METHOD", step.id + " takes no setup.answer");
  }
  if (!NeedsMet(step)) return Refuse("LOCKED", step.id + " needs other steps first", {{"needs", step.needs}});
  const std::string& name = step.Word("name");
  if (!params.contains("values") || !params["values"].is_object()) return BadParam("values", "values is no object");
  const json& values = params["values"];
  if (values.size() != 1 || !values.contains(name)) {
    return Refuse("INVALID_VALUE", step.id + " takes the value " + name + " alone", {{"name", name}});
  }
  const json& value = values[name];
  bool valid = false;
  if (step.check == "choice") {
    const auto& allowed = step.lists.at("values");
    valid = value.is_string() && std::find(allowed.begin(), allowed.end(), value.get<std::string>()) != allowed.end();
  } else {
    valid = value.is_number() && value.get<double>() >= step.Number("min") && value.get<double>() <= step.Number("max");
  }
  if (!valid) return Refuse("INVALID_VALUE", name + " is no value " + step.id + " takes", {{"name", name}});
  Record& record = RecordOf(step);
  ++record.attempt;
  record.answers = {{name, value}};
  Pass(step, 0, wall);
  return std::nullopt;
}

std::optional<Refusal> Program::Skip(const json& params, double wall) {
  const Step* found = nullptr;
  if (auto refusal = Common(params, &found)) return refusal;
  const Step& step = *found;
  if (!step.skippable) return Refuse("NOT_SKIPPABLE", step.id + " cannot be skipped");
  if (IsRun(step)) return Refuse("BUSY", step.id + " is running", {{"step", step.id}});
  if (!params.contains("reason") || !params["reason"].is_string()) return BadParam("reason", "reason is no text");
  const std::string reason = params["reason"].get<std::string>();
  const bool known = std::any_of(step.skip_reasons.begin(), step.skip_reasons.end(),
                                 [&reason](const Named& named) { return named.id == reason; });
  if (!known) return Refuse("INVALID_VALUE", reason + " is no reason to skip " + step.id, {{"name", "reason"}});
  std::string note;
  if (params.contains("note")) {
    if (!params["note"].is_string()) return BadParam("note", "note is no text");
    note = params["note"].get<std::string>();
  }
  if (note.size() > static_cast<size_t>(mowbite_api::kSetupMaxSkipText) || (reason == "other" && note.empty())) {
    return Refuse("INVALID_VALUE", "note is missing with other, or too long", {{"name", "note"}});
  }
  json fields = {{"skip", reason}};
  if (!note.empty()) fields["note"] = note;
  End(step, "skipped", "", "", std::move(fields), wall);
  return std::nullopt;
}

std::optional<Refusal> Program::Abort(const json& params, double now, double wall) {
  if (!params.is_null() && !params.is_object()) return BadParam("params", "The params are no object");
  if (!run_) return std::nullopt;
  if (params.is_object() && params.contains("step")) {
    if (!params["step"].is_string()) return BadParam("step", "step is no text");
    if (params["step"].get<std::string>() != StepOf(*run_).id) return std::nullopt;
  }
  Fail("STOPPED", "Stopped by setup.abort", json::object(), now, wall);
  return std::nullopt;
}

std::optional<Refusal> Program::Reset(const json& params) {
  if (auto refusal = Common(params, nullptr)) return refusal;
  run_.reset();
  for (auto& [id, record] : records_) record = Record();
  reached_.clear();
  events_.push_back({"SETUP", {{"what", "reset"}}});
  Changed();
  return std::nullopt;
}

void Program::BeginPart(double now, double wall) {
  Run& run = *run_;
  run.part_began = now;
  run.part_wall = wall;
  run.phase = Phase::kRest;
  run.edge = 0;
  run.released = 0;
  run.latched_since = -1;
  run.other.clear();
  run.yaw = 0;
  run.part_states[run.part] = "waiting";
}

void Program::PartDone(double now, double wall) {
  Run& run = *run_;
  run.part_states[run.part] = "ok";
  if (run.part + 1 == StepOf(run).parts.size()) {
    Pass(StepOf(run), now, wall);
    return;
  }
  ++run.part;
  BeginPart(now, wall);
}

void Program::Tick(double now, double wall) {
  TrackWorld(now);
  if (run_) {
    const std::string& check = StepOf(*run_).check;
    if (check == "board") TickBoard(now, wall);
    if (check == "input") TickInput(now, wall);
    if (check == "imu") TickImu(now, wall);
    if (check == "position") TickPosition(now, wall);
  }
  TickWaiting(wall);
}

json Program::BoardValues(const Run& run, const Step& step, double elapsed, bool final) const {
  const auto judged = [final](bool ok) { return final ? json(ok) : json(); };
  const auto board = [](const std::string& id, json value, json min, json max, json ok) {
    return Measured("board", id, std::move(value), std::move(min), std::move(max), std::move(ok));
  };
  json values = json::array();
  const double rate = run.status_count / std::max(elapsed, 0.1);
  values.push_back(board("status_rate", Rounded(rate, 10), step.Number("status_rate"), nullptr,
                         judged(rate >= step.Number("status_rate"))));
  bool firmware =
      capabilities_known_ && !capabilities_.firmware.empty() && !(hardware_.emergency_reason & FirmwareBit());
  for (const auto& entry : capabilities_.firmware) firmware = firmware && !entry.second.empty();
  values.push_back(board("firmware", FirmwareText(), nullptr, nullptr, judged(firmware)));
  const double errors = static_cast<double>(hardware_.link_errors - run.link_errors_first);
  values.push_back(board("link_errors", errors, nullptr, step.Number("link_errors"),
                         judged(errors <= step.Number("link_errors"))));
  values.push_back(board("inputs", Joined(run.inputs_seen), nullptr, nullptr, judged(run.inputs_seen.empty())));
  values.push_back(board("motors", Joined(run.motors_down), nullptr, nullptr, judged(run.motors_down.empty())));
  values.push_back(board("battery", hardware_.battery ? json(Rounded(hardware_.battery_percentage, 1)) : json(),
                         step.Number("battery"), nullptr,
                         judged(hardware_.battery && hardware_.battery_percentage >= step.Number("battery"))));
  if (step.Word("imu") == "yes") {
    const double gravity = run.gravity_count > 0 ? run.gravity_sum / run.gravity_count : 0.0;
    values.push_back(board("gravity", run.gravity_count > 0 ? json(Rounded(gravity, 100)) : json(),
                           mowbite_api::kSetupGravityMin, mowbite_api::kSetupGravityMax,
                           judged(run.gravity_count > 0 && gravity >= mowbite_api::kSetupGravityMin &&
                                  gravity <= mowbite_api::kSetupGravityMax)));
  }
  return values;
}

void Program::TickBoard(double now, double wall) {
  const Run& run = *run_;
  const Step& step = StepOf(run);
  if (now - run.began < step.Number("time")) return;
  if (run.status_count == 0) {
    Fail("NO_DATA", "No hw/status while it watched", {{"source", "hw/status"}}, now, wall);
    return;
  }
  if (!capabilities_known_) {
    Fail("NO_DATA", "No hw/capabilities", {{"source", "hw/capabilities"}}, now, wall);
    return;
  }
  if (step.Word("imu") == "yes" && run.gravity_count == 0) {
    Fail("NO_DATA", "No imu/data while it watched", {{"source", "imu/data"}}, now, wall);
    return;
  }
  for (const auto& value : BoardValues(run, step, step.Number("time"), true)) {
    if (value["ok"] == true) continue;
    const std::string id = value["id"];
    json fields = {{"value_id", id}, {"value", value["value"]}, {"unit", value["unit"]}};
    if (!value["min"].is_null()) fields["limit"] = value["min"];
    if (!value["max"].is_null() && id != "gravity") fields["limit"] = value["max"];
    Fail("LIMIT", id + " is not as it has to be", std::move(fields), now, wall);
    return;
  }
  Pass(step, now, wall);
}

void Program::TickInput(double now, double wall) {
  Run& run = *run_;
  const Step& step = StepOf(run);
  const Part& part = step.parts[run.part];
  if (!Fresh(hardware_at_, now, kNoData)) {
    Fail("NO_DATA", "No hw/status any more", {{"source", "hw/status"}}, now, wall);
    return;
  }
  const uint32_t bit = EmergencyBit(step.Word("emergency"));
  const bool signal = Signal(part.role);
  const bool latched = bit != 0 && (hardware_.emergency_reason & bit) != 0;
  if (now - run.part_began > step.Number("time")) {
    if (run.other.empty()) {
      Fail("NOT_SEEN", "The input " + part.role + " did not switch and come back in time", {}, now, wall);
    } else {
      Fail("WRONG_INPUT", "The input " + run.other + " switched, not " + part.role, {{"seen", run.other}}, now, wall);
    }
    return;
  }
  switch (run.phase) {
    case Phase::kRest:
      if (!signal && !latched) {
        run.latched_since = -1;
        run.phase = Phase::kArmed;
      }
      break;
    case Phase::kArmed:
      if (signal) {
        run.edge = now;
        run.phase = Phase::kSwitched;
        break;
      }
      for (const char* group : kGroups) {
        if (GroupOf(part.role) != group && Signal(group)) run.other = group;
      }
      if (part.role == "bumper_left" && hardware_.bumper_right) run.other = "bumper_right";
      if (part.role == "bumper_right" && hardware_.bumper_left) run.other = "bumper_left";
      // An emergency that is on before the input shows is not of this input: wait for its release first.
      if (!latched) {
        run.latched_since = -1;
      } else if (run.latched_since < 0) {
        run.latched_since = now;
      } else if (now - run.latched_since > kLatchLead) {
        run.phase = Phase::kRest;
      }
      break;
    case Phase::kSwitched:
      if (part.mode == "press") {
        if (bit == 0) {
          run.phase = Phase::kWatch;
        } else if (latched) {
          run.measured[run.part] = now - run.edge;
          run.phase = Phase::kWatch;
        } else if (now - run.edge > step.Number("within")) {
          Fail("NO_EMERGENCY", "No emergency " + step.Word("emergency") + " after the input " + part.role,
               {{"limit", step.Number("within")}}, now, wall);
        }
      } else if (now - run.edge >= step.Number("short_max")) {
        // Held too long to count as short: once more, after the emergency it may have brought is released.
        run.phase = Phase::kRest;
      } else if (latched) {
        Fail("STOPPED_EARLY", "A short switch of " + part.role + " brought the emergency", {}, now, wall);
      } else if (!signal) {
        run.released = now;
        run.phase = Phase::kWatch;
      }
      break;
    case Phase::kWatch:
      if (part.mode == "press") {
        if (!signal) PartDone(now, wall);
      } else if (latched) {
        Fail("STOPPED_EARLY", "A short switch of " + part.role + " brought the emergency", {}, now, wall);
      } else if (now - run.released >= step.Number("within")) {
        PartDone(now, wall);
      }
      break;
  }
}

// The angle of the move since the mower stood at rest (deg), nothing before both gravities are known.
std::optional<double> Program::ImuValue(const Run& run, const std::string& move) const {
  if (move == "turn_left") return run.yaw * kDegrees;
  if (move == "turn_right") return -run.yaw * kDegrees;
  if (run.rest_count == 0 || !run.gravity_known) return std::nullopt;
  const double rest[3] = {run.rest[0] / run.rest_count, run.rest[1] / run.rest_count, run.rest[2] / run.rest_count};
  const double pitch = (std::atan2(run.gravity[0], run.gravity[2]) - std::atan2(rest[0], rest[2])) * kDegrees;
  const double roll = (std::atan2(run.gravity[1], run.gravity[2]) - std::atan2(rest[1], rest[2])) * kDegrees;
  if (move == "front_up") return pitch;
  if (move == "left_up") return roll;
  if (move == "right_up") return -roll;
  return std::max(std::abs(pitch), std::abs(roll));
}

void Program::TickImu(double now, double wall) {
  Run& run = *run_;
  const Step& step = StepOf(run);
  const Part& part = step.parts[run.part];
  if (!Fresh(imu_at_, now, kNoData) && now - run.began > kNoData) {
    Fail("NO_DATA", "No imu/data", {{"source", "imu/data"}}, now, wall);
    return;
  }
  if (now - run.began <= kRestTime) {
    // The time of the part counts from the end of the rest.
    run.part_began = now;
    return;
  }
  const bool tilt_stop = part.move == "tilt_stop";
  if (now - run.part_began > step.Number("time")) {
    if (tilt_stop) {
      Fail("NO_EMERGENCY", "No emergency TILT in time", {{"limit", step.Number("time")}}, now, wall);
    } else {
      Fail("NOT_SEEN", "The IMU did not show the move " + part.move + " in time", {}, now, wall);
    }
    return;
  }
  const std::optional<double> value = ImuValue(run, part.move);
  if (!value) return;
  const bool turn = IsTurn(part.move);
  const bool tilted = (hardware_.emergency_reason & TiltBit()) != 0;
  if (run.phase == Phase::kRest) {
    bool ready = turn;
    if (tilt_stop) {
      ready = !tilted;
    } else if (!turn) {
      const std::optional<double> level = ImuValue(run, "tilt_stop");
      ready = level && *level < step.Number("tilt") / 3;
    }
    if (ready) run.phase = Phase::kArmed;
    return;
  }
  if (tilt_stop) {
    if (tilted) {
      run.measured[run.part] = *value;
      PartDone(now, wall);
    }
    return;
  }
  const double asked = step.Number(turn ? "turn" : "tilt");
  if (*value >= asked) {
    run.measured[run.part] = *value;
    PartDone(now, wall);
  } else if (*value <= -asked) {
    run.measured[run.part] = *value;
    Fail("WRONG_SIGN", "The IMU shows the move " + part.move + " the other way round", {}, now, wall);
  }
}

void Program::TickPosition(double now, double wall) {
  Run& run = *run_;
  const Step& step = StepOf(run);
  const bool data = !fixes_.empty() && Fresh(fixes_.back(), now, kNoData);
  if (!data && now - run.began > kNoData) {
    Fail("NO_DATA", "No gps/absolute_pose", {{"source", "gps/absolute_pose"}}, now, wall);
    return;
  }
  const bool solution = SolutionOk(step);
  const bool accuracy = accuracy_ <= step.Number("accuracy");
  const double rate = FixRate(now);
  const bool often = rate >= step.Number("rate");
  if (data && solution && accuracy && often) {
    if (run.good_since < 0) run.good_since = now;
    if (now - run.good_since >= step.Number("time")) {
      Pass(step, now, wall);
      return;
    }
  } else {
    run.good_since = -1;
  }
  if (now - run.began <= step.Number("timeout")) return;
  if (!solution) {
    Fail("NO_RTK", "No position of the solution " + step.Word("solution") + " in time", {}, now, wall);
  } else if (!accuracy) {
    const json fields = {{"value_id", "accuracy"},
                         {"value", Rounded(accuracy_, 1000)},
                         {"limit", step.Number("accuracy")},
                         {"unit", "m"}};
    Fail("LIMIT", "The position is not accurate enough", fields, now, wall);
  } else {
    const json fields = {{"value_id", "rate"},
                         {"value", Rounded(rate, 10)},
                         {"limit", step.Number("rate")},
                         {"unit", "Hz"}};
    Fail("LIMIT", "Positions do not come often enough", fields, now, wall);
  }
}

// The steps the OS ends by what it sees: a condition of done_when, the confirmed keys of geometry.
void Program::TickWaiting(double wall) {
  if (!writable_) return;
  for (const Step& step : list_.steps) {
    if (step.start || !step.automatic) continue;
    Record& record = RecordOf(step);
    if (record.state == "ok" || !NeedsMet(step)) continue;
    bool met = false;
    if (step.check == "geometry") {
      const auto& keys = step.lists.at("keys");
      met = std::all_of(keys.begin(), keys.end(),
                        [this](const std::string& key) { return world_.confirmed.count(key) != 0; });
    } else {
      const std::string& condition = step.Word("condition");
      if (condition == "ntrip") met = world_.ntrip;
      if (condition == "station") met = world_.docking_stations > 0 && hardware_.dock_contact;
      if (condition == "area") met = world_.areas > 0 && world_.mowing_allowed;
      if (condition == "dock_test" || condition == "mow") met = reached_.count(condition) != 0;
    }
    if (!met) continue;
    ++record.attempt;
    Pass(step, 0, wall);
  }
}

json Program::Values(const Step& step, double now, bool final) const {
  if (!IsRun(step)) return records_.at(step.id).values;
  const Run& run = *run_;
  if (step.check == "board") {
    return BoardValues(run, step, final ? step.Number("time") : now - run.began, final);
  }
  json values = json::array();
  if (step.check == "input" || step.check == "imu") {
    const bool input = step.check == "input";
    for (size_t i = 0; i < step.parts.size(); ++i) {
      const Part& part = step.parts[i];
      if (input && (part.mode != "press" || step.Word("emergency") == "NONE")) continue;
      std::optional<double> measured = run.measured[i];
      if (!input && !measured && i == run.part && part.move != "tilt_stop") measured = ImuValue(run, part.move);
      const std::string& state = run.part_states[i];
      const json ok = state == "ok" ? json(true) : state == "failed" ? json(false) : json();
      const char* asked = IsTurn(part.move) ? "turn" : "tilt";
      values.push_back(Value(part.id, measured ? json(Rounded(*measured, input ? 100 : 10)) : json(),
                             input ? "s" : "deg",
                             input || part.move == "tilt_stop" ? json() : json(step.Number(asked)),
                             input ? json(step.Number("within")) : json(), ok, part.texts));
    }
    return values;
  }
  const char* const names[] = {"none", "single", "single", "float", "fixed", "none"};
  const double rate = FixRate(now);
  values.push_back(
      Measured("position", "solution", names[std::clamp(solution_, 0, 5)], nullptr, nullptr, SolutionOk(step)));
  values.push_back(Measured("position", "accuracy", Rounded(accuracy_, 1000), nullptr, step.Number("accuracy"),
                            accuracy_ <= step.Number("accuracy")));
  values.push_back(Measured("position", "rate", Rounded(rate, 10), step.Number("rate"), nullptr,
                            rate >= step.Number("rate")));
  return values;
}

json Program::Blocked(const Step& step, double now) const {
  json blocked = json::array();
  const auto add = [&blocked](const char* reason, const std::string& message) {
    blocked.push_back({{"reason", reason}, {"message", message}});
  };
  if (!writable_) add("STORAGE_NOT_WRITABLE", "setup.json cannot be written");
  if (IsRun(step)) {
    const Run& run = *run_;
    if (step.parts.empty()) return blocked;
    const Part& part = step.parts[run.part];
    const bool rest = run.phase == Phase::kRest;
    if (step.check == "input") {
      const uint32_t bit = EmergencyBit(step.Word("emergency"));
      // After a press the emergency shows as well: a board may hold the input until the release.
      const bool pressed = run.phase == Phase::kWatch && part.mode == "press";
      if (rest && Signal(part.role)) {
        add("INPUT_ACTIVE", "The input " + part.role + " is switched already");
      } else if ((rest || pressed) && bit != 0 && (hardware_.emergency_reason & bit) != 0) {
        add("EMERGENCY", "The emergency " + step.Word("emergency") + " is still latched");
      }
      return blocked;
    }
    if (!rest) return blocked;
    if (part.move == "tilt_stop") {
      if ((hardware_.emergency_reason & TiltBit()) != 0) add("EMERGENCY", "The emergency TILT is still latched");
    } else if (now - run.began > kRestTime) {
      add("INPUT_ACTIVE", "The mower does not stand level yet");
    }
    return blocked;
  }
  if (!step.start || step.kind != "measure") return blocked;
  if (run_) add("BUSY", StepOf(*run_).id + " is running");
  if (!world_.state.empty() && world_.state != "IDLE") add("BUSY", "The mower is in " + world_.state);
  if ((step.check == "board" || step.check == "input") && !Fresh(hardware_at_, now, kFresh)) {
    add("DRIVER_MISSING", "The driver of the platform reports no status");
  }
  if (step.check == "imu" && !Fresh(imu_at_, now, kFresh)) add("NO_DATA", "No imu/data");
  if (step.check == "position" && (fixes_.empty() || !Fresh(fixes_.back(), now, kNoData))) {
    add("NO_DATA", "No gps/absolute_pose");
  }
  return blocked;
}

std::string Program::BlockedNow(double now) const {
  std::string text;
  for (const Step& step : list_.steps) {
    for (const auto& entry : Blocked(step, now)) text += step.id + ":" + entry["reason"].get<std::string>() + ";";
  }
  return text;
}

json Program::State(double now, const std::map<std::string, std::pair<std::string, int>>& images) const {
  const auto named = [](const std::vector<Named>& list) {
    json out = json::array();
    for (const Named& item : list) out.push_back({{"id", item.id}, {"texts", item.texts}});
    return out;
  };
  const auto image = [&images](const std::string& name, bool accent) -> json {
    const auto found = images.find(name);
    if (name.empty() || found == images.end()) return nullptr;
    json out = {{"name", name}, {"sha256", found->second.first}, {"bytes", found->second.second}};
    if (!accent) out["type"] = "image/svg+xml";
    return out;
  };
  json steps = json::array();
  json stages = json::array();
  json first_stage;
  json current = run_ ? json(StepOf(*run_).id) : json();
  for (const Step& step : list_.steps) {
    const Record& record = records_.at(step.id);
    const bool is_run = IsRun(step);
    const std::string shown = Shown(step);
    if (!Done(record.state)) {
      if (first_stage.is_null()) first_stage = step.stage;
      if (current.is_null() && shown != "locked") current = step.id;
    }
    json inputs = json::array();
    if (step.check == "choice" || step.check == "number") {
      const std::string& name = step.Word("name");
      json input = {{"name", name},
                    {"type", step.check},
                    {"ask", false},
                    {"value", record.answers.contains(name) ? record.answers[name] : json()},
                    {"texts", step.texts}};
      if (step.check == "choice") {
        input["values"] = named(step.choices);
      } else {
        input["unit"] = step.Word("unit");
        input["min"] = step.Number("min");
        input["max"] = step.Number("max");
      }
      inputs.push_back(std::move(input));
    }
    json parts = json::array();
    for (size_t i = 0; i < step.parts.size(); ++i) {
      std::string state = "open";
      if (is_run) {
        state = run_->part_states[i];
      } else if (i < record.parts.size() && record.parts[i].is_string()) {
        state = record.parts[i].get<std::string>();
      }
      parts.push_back({{"id", step.parts[i].id}, {"state", state}, {"texts", step.parts[i].texts}});
    }
    json progress;
    json ends_at;
    if (is_run) {
      if (!step.parts.empty()) {
        progress = {{"n", run_->part + 1}, {"of", step.parts.size()}};
        ends_at = std::floor(run_->part_wall + step.Number("time"));
      } else {
        ends_at = std::floor(run_->wall_began + step.Number(step.check == "position" ? "timeout" : "time"));
      }
    }
    json picture = image(step.image, false);
    if (!picture.is_null()) picture["accent"] = image(step.accent, true);
    steps.push_back({{"id", step.id},
                     {"stage", step.stage},
                     {"check", step.check},
                     {"kind", step.kind},
                     {"state", shown},
                     {"attempt", record.attempt},
                     {"needs", step.needs},
                     {"blocked", Blocked(step, now)},
                     {"motion", nullptr},
                     {"confirm", named(step.confirm)},
                     {"start", step.start},
                     {"auto", step.automatic},
                     {"skippable", step.skippable},
                     {"skip_reasons", named(step.skip_reasons)},
                     {"inputs", std::move(inputs)},
                     {"values", Values(step, now, false)},
                     {"parts", std::move(parts)},
                     {"progress", std::move(progress)},
                     {"ends_at", std::move(ends_at)},
                     {"open", step.open.empty() ? json() : json(step.open)},
                     {"image", std::move(picture)},
                     {"symbol", step.symbol.empty() ? json() : json(step.symbol)},
                     {"mark", step.mark},
                     {"texts", step.texts},
                     {"result", record.result},
                     {"restart", false}});
    if (stages.empty() || stages.back()["id"] != step.stage) {
      const mowbite_api::SetupName* stage = mowbite_api::FindSetupName(mowbite_api::SetupStages(), step.stage);
      stages.push_back({{"id", step.stage}, {"state", ""}, {"texts", mowbite_api::Texts(stage->texts)}});
    }
  }
  for (auto& stage : stages) {
    int all = 0, ok = 0, done = 0, failed = 0, running = 0;
    for (const Step& step : list_.steps) {
      if (step.stage != stage["id"]) continue;
      const std::string& state = records_.at(step.id).state;
      ++all;
      ok += state == "ok";
      done += Done(state);
      failed += state == "failed";
      running += state == "running";
    }
    stage["state"] = running ? "running" : ok == all ? "ok" : done == all ? "skipped" : failed ? "failed" : "open";
  }
  const std::string& date = capabilities_.safety_measurement_date;
  return {{"version", version_},
          {"platform", platform_},
          {"catalog", list_.catalog},
          {"stage", std::move(first_stage)},
          {"current", std::move(current)},
          {"complete", Complete()},
          {"safety",
           {{"open", !date.empty()},
            {"date", date.empty() ? json() : json(date)},
            {"firmware", nullptr},
            {"why", date.empty() ? json("NO_SAFETY_MEASUREMENT") : json()}}},
          {"restart_pending", false},
          {"stages", std::move(stages)},
          {"steps", std::move(steps)}};
}

}  // namespace mowbite_setup
