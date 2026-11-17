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
// The program's own release of an emergency a part brought: a try this often (s), and for this long after the step.
// A driver may take a release only every second.
constexpr double kReleaseEvery = 1.5;
constexpr double kReleaseAfter = 10.0;
// A step inputs_assign waits this long (s) after the first line changed for others that change with it.
constexpr double kAssignSettle = 0.3;
constexpr double kDegrees = 180.0 / M_PI;

constexpr const char* kStepEvent = "SETUP_STEP";

// The roles that stand for all inputs of their kind: stop, lift, bumper.
const std::vector<std::string>& Groups() {
  static const std::vector<std::string> groups = [] {
    std::vector<std::string> out;
    for (const auto& role : mowbite_api::SetupRoles()) {
      if (role.group == role.id) out.push_back(role.id);
    }
    return out;
  }();
  return groups;
}

// Bits of emergency_reason of hw/status the checks ask for, by their names in the catalog.
uint32_t FirmwareBit() {
  static const uint32_t bit = EmergencyBit("FIRMWARE_INCOMPATIBLE");
  return bit;
}

uint32_t TiltBit() {
  static const uint32_t bit = EmergencyBit("TILT");
  return bit;
}

uint32_t LatchBit() {
  static const uint32_t bit = EmergencyBit("LATCH");
  return bit;
}

uint32_t LearningBit() {
  static const uint32_t bit = EmergencyBit("INPUTS_LEARNING");
  return bit;
}

bool IsAssign(const Step& step) {
  return step.check == "inputs_assign";
}

json Keys(const std::map<std::string, bool>& lines) {
  json names = json::array();
  for (const auto& [name, level] : lines) names.push_back(name);
  return names;
}

bool IsTurn(const std::string& move) {
  return move.rfind("turn", 0) == 0;
}

// The group of a role, empty for a text that is no role.
std::string GroupOf(const std::string& role) {
  const auto* known = mowbite_api::FindSetupName(mowbite_api::SetupRoles(), role);
  return known ? known->group : "";
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
      if (saved.contains("assign")) record.assign = saved["assign"];
    }
    records[step.id] = std::move(record);
  }
  records_ = std::move(records);
  run_.reset();
  if (file.contains("version") && file["version"].is_number_integer()) {
    version_ = static_cast<int>(std::clamp<int64_t>(file["version"].get<int64_t>(), 0, 1000000000)) + 1;
  }
  for (const Step& step : list_.steps) {
    Record& record = RecordOf(step);
    const json& assign = record.assign;
    const bool learn = assign.is_object() && assign.value("phase", json()) == "learn";
    const bool apply = assign.is_object() && assign.value("phase", json()) == "apply" && assign.contains("parts") &&
                       assign["parts"].is_object();
    if (record.state != "running" || !IsAssign(step) || !(learn || apply) || run_) {
      record.assign = nullptr;
      if (record.state == "running") End(step, "open", "INTERRUPTED", "The OS restarted while it ran", {}, wall);
      continue;
    }
    // The restart the step asked for: it goes on with the driver, a learning one from its first part.
    if (learn) record.assign["parts"] = json::object();
    run_ = AssignRun(step, apply, wall);
  }
  return true;
}

Program::Run Program::AssignRun(const Step& step, bool apply, double wall) const {
  Run run;
  run.step = static_cast<size_t>(&step - list_.steps.data());
  run.began = run.stage_began = -1;
  run.wall_began = wall;
  run.part_states.assign(step.parts.size(), apply ? "ok" : "open");
  run.measured.assign(step.parts.size(), std::nullopt);
  run.part = apply ? step.parts.size() : 0;
  run.assign = Assign::kDriver;
  return run;
}

std::string Program::Text() const {
  json steps = json::object();
  for (const Step& step : list_.steps) {
    const Record& record = records_.at(step.id);
    steps[step.id] = {{"state", record.state},   {"attempt", record.attempt}, {"result", record.result},
                      {"values", record.values}, {"parts", record.parts},     {"answers", record.answers}};
    if (!record.assign.is_null()) steps[step.id]["assign"] = record.assign;
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

std::vector<const Line*> Program::LinesOf(const std::string& role) const {
  std::vector<const Line*> lines;
  for (const Line& line : lines_) {
    if (line.assigned && (line.role == role || GroupOf(line.role) == role)) lines.push_back(&line);
  }
  return lines;
}

// The lines of its role that are active count for the running part from the moment it waits for its press.
void Program::NoteLines() {
  if (!run_ || StepOf(*run_).check != "input" || run_->phase == Phase::kRest) return;
  for (const Line* line : LinesOf(StepOf(*run_).parts[run_->part].role)) {
    if (line->active) run_->part_lines.insert(line->name);
  }
}

// Every hw/status counts here, not only the last one before a tick: a bit other than the emergency of the part, for
// one message, and the program releases nothing itself. So does a driver that does not count its latches.
void Program::WatchOwnLatch() {
  if (!own_) return;
  const uint32_t reason = hardware_.emergency_reason;
  const bool other = (reason & ~(own_->bit | LatchBit())) != 0;
  const bool released = own_->confirmed && (reason & (own_->bit | LatchBit())) == 0;
  const bool uncounted = own_->confirmed && hardware_.latch_count == 0;
  if (other || released || uncounted) own_.reset();
}

void Program::OnHardware(const Hardware& hardware, double now) {
  hardware_ = hardware;
  hardware_at_ = now;
  WatchOwnLatch();
  if (!run_ || StepOf(*run_).check != "board") return;
  ++run_->status_count;
  for (const auto& group : Groups()) {
    if (Signal(group)) run_->inputs_seen.insert(group);
  }
  for (const auto& motor : hardware.motors_down) run_->motors_down.insert(motor);
}

void Program::OnInputs(const std::vector<Line>& lines, bool learning) {
  lines_ = lines;
  learning_ = learning;
  inputs_known_ = true;
  NoteLines();
  NoteLevels();
}

void Program::SetLayer(Layer layer, std::set<std::string> shadowed) {
  layer_ = std::move(layer);
  shadowed_ = std::move(shadowed);
  layer_set_ = true;
}

// Every hw/inputs counts for a step inputs_assign that watches: a line that left its level at rest, with the level
// it went to. A line a part took already is out of it.
void Program::NoteLevels() {
  if (!run_ || !IsAssign(StepOf(*run_))) return;
  Run& run = *run_;
  if (run.assign != Assign::kQuiet && run.assign != Assign::kPart) return;
  const json& taken = RecordOf(StepOf(run)).assign["parts"];
  for (const Line& line : lines_) {
    const auto rest = run.levels.find(line.name);
    if (rest == run.levels.end() || rest->second == line.active || run.changed.count(line.name)) continue;
    const bool is_taken = std::any_of(taken.begin(), taken.end(),
                                      [&line](const json& part) { return part["line"] == line.name; });
    if (!is_taken) run.changed[line.name] = line.active;
  }
}

bool Program::AtRest() const {
  for (const Line& line : lines_) {
    const auto rest = run_->levels.find(line.name);
    if (rest != run_->levels.end() && rest->second != line.active) return false;
  }
  return true;
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

std::optional<ReleaseRequest> Program::TakeRelease() {
  std::optional<ReleaseRequest> release;
  release.swap(release_);
  return release;
}

// The running step ended. An emergency its part brought and showed is released for a while yet.
void Program::EndRun(double now) {
  if (own_ && !own_->confirmed) own_.reset();
  if (own_ && own_->until < 0) own_->until = now + kReleaseAfter;
  // A step inputs_assign that ends without its table: the next start does not learn, and should the driver learn
  // now, TickLearning restarts it.
  const Step& step = StepOf(*run_);
  Record& record = RecordOf(step);
  if (!record.assign.is_null()) {
    record.assign = nullptr;
    leave_learning_ = true;
    restart_wanted_ = false;
    if (!layer_.Get(step.Word("node"), mowbite_api::kInputsLearn).is_null()) {
      layer_.Erase(step.Word("node"), mowbite_api::kInputsLearn);
      layer_dirty_ = true;
    }
  }
  run_.reset();
}

std::optional<std::string> Program::TakeLayer() {
  if (!layer_dirty_) return std::nullopt;
  layer_dirty_ = false;
  layer_pending_ = true;
  return layer_.Text();
}

void Program::LayerWritten(bool written, double now, double wall) {
  layer_pending_ = false;
  if (written) return;
  restart_wanted_ = false;
  if (run_ && IsAssign(StepOf(*run_))) {
    Fail("NOT_APPLIED", "The parameter file of the setup program cannot be written", json::object(), now, wall);
  }
}

// Only a mower that stands, and never to get rid of an emergency: none may be on but the one of learning itself.
bool Program::RestartAllowed(double now) const {
  const uint32_t reason = hardware_.emergency_reason;
  const bool learning = (reason & LearningBit()) != 0;
  const uint32_t allowed = learning ? LearningBit() | LatchBit() : 0;
  return Fresh(hardware_at_, now, kFresh) && world_.state == "IDLE" && (reason & ~allowed) == 0;
}

void Program::WantRestart() {
  layer_dirty_ = true;
  restart_wanted_ = true;
  restart_sent_ = -1;
}

bool Program::TakeRestart(double now) {
  if (!restart_wanted_ || layer_dirty_ || layer_pending_ || changed_ || !writable_ || !RestartAllowed(now)) {
    return false;
  }
  restart_wanted_ = false;
  restart_sent_ = now;
  return true;
}

// The driver learns its inputs and no step is at it (stopped, failed, reset, or its record is lost): the program takes
// the learning back and restarts, once. A learning the file kept by hand asks for is not its own.
void Program::TickLearning() {
  if (!learning_ || !layer_set_ || restart_wanted_ || restart_sent_ >= 0) return;
  if (run_ && IsAssign(StepOf(*run_))) return;
  const auto step = std::find_if(list_.steps.begin(), list_.steps.end(), IsAssign);
  if (step == list_.steps.end()) return;
  const std::string& node = step->Word("node");
  if (!leave_learning_ && layer_.Get(node, mowbite_api::kInputsLearn).is_null()) return;
  leave_learning_ = false;
  layer_.Erase(node, mowbite_api::kInputsLearn);
  WantRestart();
}

// Asks for the release of the emergency of own_ when its time has come; see OwnLatch for the conditions.
void Program::TickRelease(double now) {
  if (!own_) return;
  OwnLatch& own = *own_;
  if (own.until >= 0 && now > own.until) {
    own_.reset();
    return;
  }
  if (!own.confirmed || !Fresh(hardware_at_, now, kFresh) || world_.state != "IDLE") return;
  if (!own.held && Signal(own.role)) return;
  if (now - (own.last_try < 0 ? own.edge : own.last_try) < kReleaseEvery) return;
  own.last_try = now;
  release_ = ReleaseRequest{hardware_.latch_count, "setup: the input " + own.role + " was checked"};
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
    if (IsAssign(step)) record.assign = nullptr;
    EndRun(now);
  }
  End(step, "ok", "", "", {}, wall);
}

void Program::Fail(const std::string& reason, const std::string& message, json fields, double now, double wall) {
  const Step& step = StepOf(*run_);
  Record& record = RecordOf(step);
  const bool stopped = reason == "STOPPED";
  record.values = Values(step, now, true);
  // A step inputs_assign is in a part only while it asks for its line.
  const bool in_part = !IsAssign(step) || run_->assign == Assign::kPart || run_->assign == Assign::kAsk;
  if (!step.parts.empty() && in_part && run_->part < step.parts.size()) {
    run_->part_states[run_->part] = stopped ? "open" : "failed";
    if (!stopped) fields["part"] = step.parts[run_->part].id;
  }
  if (!step.parts.empty()) record.parts = run_->part_states;
  EndRun(now);
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
  if (step.kind == "measure" && learning_) return Refuse("BUSY", "The driver learns its inputs");
  if (IsAssign(step)) {
    if (auto refusal = AssignRefused(now)) return refusal;
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
  run.stage_began = now;
  run_ = std::move(run);
  if (IsAssign(step)) {
    // The driver starts again learning; the step goes on after that restart.
    record.assign = {{"phase", "learn"}, {"parts", json::object()}};
    layer_.Set(step.Word("node"), mowbite_api::kInputsLearn, true);
    leave_learning_ = false;
    WantRestart();
  } else if (!step.parts.empty()) {
    BeginPart(now, wall);
  }
  Changed();
  return std::nullopt;
}

std::optional<Refusal> Program::AssignRefused(double now) const {
  if (!layer_set_) {
    return Refuse("STORAGE_NOT_WRITABLE", "No data_dir for the table of the inputs", {{"file", "setup.yaml"}});
  }
  if (!Fresh(hardware_at_, now, kFresh) || !inputs_known_) {
    return Refuse("DRIVER_MISSING", "The driver of the platform reports no status or no inputs");
  }
  if (world_.state != "IDLE") return Refuse("BUSY", "The mower is not in IDLE", {{"state", world_.state}});
  if (hardware_.emergency_reason != 0) {
    return Refuse("EMERGENCY", "An emergency stands: " + mowbite_api::EmergencyNames(
                                   static_cast<int>(hardware_.emergency_reason)));
  }
  return std::nullopt;
}

std::optional<Refusal> Program::Answer(const json& params, double now, double wall) {
  const Step* found = nullptr;
  if (auto refusal = Common(params, &found)) return refusal;
  const Step& step = *found;
  // A step inputs_assign that runs takes the answer to its question, one that is ok the delays of its lines.
  if (IsAssign(step)) {
    return RecordOf(step).state == "ok" ? AnswerDelays(step, params, now, wall) : AnswerAssign(step, params);
  }
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

// accept yes takes the line ask names for its part, no leaves the part waiting for another one.
std::optional<Refusal> Program::AnswerAssign(const Step& step, const json& params) {
  if (!IsRun(step) || run_->assign != Assign::kAsk) return Refuse("WRONG_METHOD", step.id + " asks nothing right now");
  const std::string& name = mowbite_api::SetupAccept().id;
  const json none = json::object();
  const json& values = params.contains("values") && params["values"].is_object() ? params["values"] : none;
  const std::string accept = values.size() == 1 && values.contains(name) && values[name].is_string()
                                 ? values[name].get<std::string>()
                                 : "";
  if (!mowbite_api::FindSetupName(mowbite_api::SetupAcceptValues(), accept)) {
    return Refuse("INVALID_VALUE", step.id + " takes " + name + " alone, yes or no", {{"name", name}});
  }
  Run& run = *run_;
  if (accept == "yes") {
    RecordOf(step).assign["parts"][step.parts[run.part].id] = {{"line", run.proposed},
                                                               {"active_high", run.proposed_high}};
    run.part_states[run.part] = "ok";
    ++run.part;
  }
  run.assign = Assign::kRest;
  run.stage_began = -1;
  Changed();
  return std::nullopt;
}

int Program::DelayOf(const Step& step, const Part& part) const {
  const json& answers = records_.at(step.id).answers;
  const std::string name = mowbite_api::kSetupDelayPrefix + part.id;
  if (!answers.contains(name) || !answers[name].is_number_integer()) return part.delay_ms;
  return static_cast<int>(std::clamp<int64_t>(answers[name].get<int64_t>(), 0, part.delay_ms));
}

// The delays of lines the step found, each a whole number of ms up to the one of its part: never longer than the
// platform gives. The table goes into the parameter file again and the OS restarts with it, as at the end of the
// step; the steps of the inputs are open again after it.
std::optional<Refusal> Program::AnswerDelays(const Step& step, const json& params, double now, double wall) {
  if (run_) return Refuse("BUSY", StepOf(*run_).id + " is running", {{"step", StepOf(*run_).id}});
  if (!params.contains("values") || !params["values"].is_object()) return BadParam("values", "values is no object");
  const json& values = params["values"];
  if (values.empty()) return BadParam("values", "values names no delay");
  bool changes = false;
  for (const auto& item : values.items()) {
    const auto part = std::find_if(step.parts.begin(), step.parts.end(), [&item](const Part& part) {
      return mowbite_api::kSetupDelayPrefix + part.id == item.key();
    });
    const json& value = item.value();
    if (part == step.parts.end() || !value.is_number_integer() || value.get<int64_t>() < 0 ||
        value.get<int64_t>() > part->delay_ms) {
      return Refuse("INVALID_VALUE", item.key() + " is no delay " + step.id + " takes", {{"name", item.key()}});
    }
    changes = changes || value.get<int64_t>() != DelayOf(step, *part);
  }
  if (!changes) return std::nullopt;
  if (auto refusal = AssignRefused(now)) return refusal;
  const std::string& node = step.Word("node");
  const json lines = layer_.Get(node, mowbite_api::kInputsLines);
  const json levels = layer_.Get(node, mowbite_api::kInputsActiveHigh);
  if (!lines.is_array() || !levels.is_array() || lines.size() != step.parts.size() || levels.size() != lines.size()) {
    return Refuse("WRONG_METHOD", "The parameter file holds no table of " + step.id + ": do the step again");
  }
  Record& record = RecordOf(step);
  for (const auto& item : values.items()) record.answers[item.key()] = item.value();
  record.assign = {{"phase", "apply"}, {"parts", json::object()}};
  for (size_t i = 0; i < step.parts.size(); ++i) {
    record.assign["parts"][step.parts[i].id] = {{"line", lines[i]}, {"active_high", levels[i]}};
  }
  ++record.attempt;
  record.state = "running";
  run_ = AssignRun(step, true, wall);
  FinishAssign(now, wall);
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
  if (run_ && IsAssign(StepOf(*run_))) leave_learning_ = true;
  restart_wanted_ = false;
  run_.reset();
  own_.reset();
  release_.reset();
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
  run.part_lines.clear();
  run.repeated = false;
  run.yaw = 0;
  run.part_states[run.part] = "waiting";
}

void Program::PartDone(double now, double wall) {
  Run& run = *run_;
  const Step& step = StepOf(run);
  run.part_states[run.part] = "ok";
  run.lines.insert(run.part_lines.begin(), run.part_lines.end());
  if (run.part + 1 == step.parts.size()) {
    // With hw/inputs every line of the roles of the step has switched once by now: a dead contact of a pair shows.
    json silent = json::array();
    if (step.check == "input") {
      std::set<std::string> asked;
      for (const Part& part : step.parts) {
        for (const Line* line : LinesOf(part.role)) {
          if (!run.lines.count(line->name) && asked.insert(line->name).second) silent.push_back(line->name);
        }
      }
    }
    if (silent.empty()) {
      Pass(step, now, wall);
    } else {
      Fail("LINE_SILENT", "Lines of the step did not switch: " + silent.dump(), {{"lines", silent}}, now, wall);
    }
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
    if (check == "inputs_assign") TickAssign(now, wall);
  }
  TickLearning();
  TickRelease(now);
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
    if (run.repeated) {
      json silent = json::array();
      for (const Line* line : LinesOf(part.role)) {
        if (!run.lines.count(line->name)) silent.push_back(line->name);
      }
      Fail("LINE_SILENT", "Only lines of the parts before switched, not " + silent.dump(), {{"lines", silent}}, now,
           wall);
    } else if (run.other.empty()) {
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
        run.part_lines.clear();
        run.phase = Phase::kArmed;
      }
      break;
    case Phase::kArmed:
      if (signal) {
        run.edge = now;
        run.phase = Phase::kSwitched;
        NoteLines();
        if (part.mode == "press" && bit != 0) {
          const auto lines = LinesOf(part.role);
          OwnLatch own;
          own.role = part.role;
          own.bit = bit;
          own.held = std::any_of(lines.begin(), lines.end(), [](const Line* line) { return line->held; });
          own.edge = now;
          own_ = own;
          WatchOwnLatch();
        }
        break;
      }
      for (const auto& group : Groups()) {
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
          if (own_) own_->confirmed = true;
          WatchOwnLatch();
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
        if (signal) break;
        // With hw/inputs a part has to bring a line no part before had, as long as its role has one left.
        const auto lines = LinesOf(part.role);
        const auto fresh = [&run](const std::string& name) { return run.lines.count(name) == 0; };
        const bool left = std::any_of(lines.begin(), lines.end(), [&](const Line* line) { return fresh(line->name); });
        if (!left || std::any_of(run.part_lines.begin(), run.part_lines.end(), fresh)) {
          PartDone(now, wall);
        } else {
          run.repeated = true;
          run.phase = Phase::kRest;
        }
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

void Program::EnterAssign(Assign stage, double now) {
  run_->assign = stage;
  run_->stage_began = now;
}

void Program::BeginAssignPart(double now, double wall) {
  Run& run = *run_;
  EnterAssign(Assign::kPart, now);
  run.part_began = now;
  run.part_wall = wall;
  run.levels.clear();
  for (const Line& line : lines_) run.levels[line.name] = line.active;
  run.changed.clear();
  run.first_change = -1;
  run.part_states[run.part] = "waiting";
}

// The table hw/inputs shows is the one the parts found: every line assigned with its role, level and delay.
bool Program::TableApplied(const Step& step) const {
  const json& parts = records_.at(step.id).assign["parts"];
  for (const Part& part : step.parts) {
    if (!parts.contains(part.id)) return false;
    const json& found = parts[part.id];
    const auto line = std::find_if(lines_.begin(), lines_.end(),
                                   [&found](const Line& line) { return found["line"] == line.name; });
    if (line == lines_.end() || !line->assigned || line->role != part.role || line->delay_ms != DelayOf(step, part) ||
        found["active_high"] != line->active_high) {
      return false;
    }
  }
  return true;
}

// The table of the parts goes into the parameter file, without the learning, and the OS restarts with it.
void Program::FinishAssign(double now, double wall) {
  const Step& step = StepOf(*run_);
  Record& record = RecordOf(step);
  json lines = json::array(), levels = json::array(), reasons = json::array(), delays = json::array();
  json roles = json::array();
  for (const Part& part : step.parts) {
    const json& found = record.assign["parts"][part.id];
    lines.push_back(found["line"]);
    levels.push_back(found["active_high"]);
    // The reason of a line as hw/inputs names it: the emergency of its role in small letters.
    std::string reason = mowbite_api::FindSetupName(mowbite_api::SetupRoles(), part.role)->emergency;
    std::transform(reason.begin(), reason.end(), reason.begin(), [](unsigned char c) { return std::tolower(c); });
    reasons.push_back(reason);
    delays.push_back(DelayOf(step, part));
    roles.push_back(part.role);
  }
  const std::string& node = step.Word("node");
  const bool set = layer_.Set(node, mowbite_api::kInputsLines, lines) &&
                   layer_.Set(node, mowbite_api::kInputsActiveHigh, levels) &&
                   layer_.Set(node, mowbite_api::kInputsReasons, reasons) &&
                   layer_.Set(node, mowbite_api::kInputsDelays, delays) &&
                   layer_.Set(node, mowbite_api::kInputsRoles, roles);
  if (!set) {
    Fail("NOT_APPLIED", "The table of the inputs is none a parameter file holds", json::object(), now, wall);
    return;
  }
  layer_.Erase(node, mowbite_api::kInputsLearn);
  // The table is another one from the next start on: only a check of the inputs with it counts.
  for (const Step& other : list_.steps) {
    if (other.check == "input") RecordOf(other) = Record();
  }
  record.assign["phase"] = "apply";
  EnterAssign(Assign::kRestart, now);
  WantRestart();
  Changed();
}

void Program::TickAssign(double now, double wall) {
  Run& run = *run_;
  const Step& step = StepOf(run);
  if (run.stage_began < 0) run.stage_began = now;
  if (run.began < 0) run.began = now;
  const bool learn = RecordOf(step).assign["phase"] == "learn";
  const double restart_time = step.Number("restart_time");
  const bool ambiguous = !run.ambiguous.empty();
  switch (run.assign) {
    case Assign::kRestart:
      // Still here that long after the restart was asked for: it did not come.
      if (restart_sent_ >= 0 && now - restart_sent_ > restart_time) {
        Fail("RESTART_FAILED", "The OS did not restart", {{"limit", restart_time}}, now, wall);
      }
      break;
    case Assign::kDriver:
      if (inputs_known_ && Fresh(hardware_at_, now, kFresh) && learning_ == learn) {
        if (learn) {
          EnterAssign(Assign::kQuiet, now);
          run.levels.clear();
          for (const Line& line : lines_) run.levels[line.name] = line.active;
          run.changed.clear();
          break;
        }
        if (TableApplied(step)) {
          Pass(step, now, wall);
          break;
        }
      }
      if (now - run.stage_began > restart_time) {
        // The driver is not as the file of the program asks: the file kept by hand may overrule it.
        json fields = json::object();
        const std::string prefix = step.Word("node") + ".inputs.";
        const auto key = std::find_if(shadowed_.begin(), shadowed_.end(),
                                      [&prefix](const std::string& name) { return name.rfind(prefix, 0) == 0; });
        if (key != shadowed_.end()) fields["key"] = *key;
        Fail("NOT_APPLIED", learn ? "The driver does not learn its inputs" : "hw/inputs does not show the table",
             std::move(fields), now, wall);
      }
      break;
    case Assign::kQuiet:
      if (now - run.stage_began < step.Number("quiet")) break;
      if (!run.changed.empty()) {
        Fail("FLUTTER", "Lines changed while nothing was touched: " + Keys(run.changed).dump(),
             {{"lines", Keys(run.changed)}}, now, wall);
        break;
      }
      run.part = 0;
      BeginAssignPart(now, wall);
      break;
    case Assign::kPart:
      if (!run.changed.empty()) {
        if (run.first_change < 0) run.first_change = now;
        if (now - run.first_change < kAssignSettle) break;
        if (run.changed.size() == 1) {
          run.proposed = run.changed.begin()->first;
          run.proposed_high = run.changed.begin()->second;
          EnterAssign(Assign::kAsk, now);
          Changed();
        } else {
          // More than one line: once more, when all are back.
          run.ambiguous = Keys(run.changed);
          EnterAssign(Assign::kRest, now);
        }
      } else if (now - run.part_began > step.Number("time")) {
        Fail(ambiguous ? "AMBIGUOUS" : "NO_LINE",
             ambiguous ? "More than one line changed: " + run.ambiguous.dump() : "No line changed in time",
             ambiguous ? json{{"lines", run.ambiguous}} : json::object(), now, wall);
      }
      break;
    case Assign::kAsk:
      break;
    case Assign::kRest:
      if (AtRest()) {
        if (run.part == step.parts.size()) {
          FinishAssign(now, wall);
        } else {
          if (run.part_states[run.part] != "waiting") run.ambiguous = json::array();
          BeginAssignPart(now, wall);
        }
      } else if (now - run.stage_began > step.Number("time")) {
        Fail(ambiguous ? "AMBIGUOUS" : "NO_LINE",
             ambiguous ? "More than one line changed: " + run.ambiguous.dump() : "The lines did not come back to rest",
             ambiguous ? json{{"lines", run.ambiguous}} : json::object(), now, wall);
      }
      break;
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
  if (IsAssign(step)) {
    const json& parts = records_.at(step.id).assign["parts"];
    for (const Part& part : step.parts) {
      const bool taken = parts.contains(part.id);
      values.push_back(Value(part.id, taken ? parts[part.id]["line"] : json(), "", nullptr, nullptr,
                             taken ? json(true) : json(), part.texts));
    }
    return values;
  }
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
  if (IsRun(step) && IsAssign(step)) {
    if (restart_wanted_ && !RestartAllowed(now)) {
      if (world_.state != "IDLE") {
        add("BUSY", "The mower is not in IDLE");
      } else {
        add("EMERGENCY", "An emergency stands, the OS restarts once it is released");
      }
    }
    if (run_->assign == Assign::kRest && !AtRest()) add("INPUT_ACTIVE", "A line is still switched");
    return blocked;
  }
  if (IsRun(step)) {
    const Run& run = *run_;
    if (step.parts.empty()) return blocked;
    const Part& part = step.parts[run.part];
    const bool rest = run.phase == Phase::kRest;
    if (step.check == "input") {
      const uint32_t bit = EmergencyBit(step.Word("emergency"));
      const bool pressed = run.phase == Phase::kWatch && part.mode == "press";
      // The program releases the emergency of a press itself: until then only the input shows, while it is switched.
      const bool own = own_ && own_->confirmed;
      if ((rest || (pressed && own)) && Signal(part.role)) {
        add("INPUT_ACTIVE", "The input " + part.role + " is still switched");
      } else if ((rest || pressed) && !own && bit != 0 && (hardware_.emergency_reason & bit) != 0) {
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
  if (!step.start || (step.kind != "measure" && !IsAssign(step))) return blocked;
  if (run_) add("BUSY", StepOf(*run_).id + " is running");
  if (!world_.state.empty() && world_.state != "IDLE") add("BUSY", "The mower is in " + world_.state);
  if (learning_ && !run_) add("BUSY", "The driver learns its inputs");
  if ((step.check == "board" || step.check == "input" || IsAssign(step)) && !Fresh(hardware_at_, now, kFresh)) {
    add("DRIVER_MISSING", "The driver of the platform reports no status");
  }
  if (IsAssign(step) && !learning_ && hardware_.emergency_reason != 0) add("EMERGENCY", "An emergency stands");
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
                    {"advanced", false},
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
    const bool asks = is_run && IsAssign(step) && run_->assign == Assign::kAsk;
    if (IsAssign(step)) {
      const mowbite_api::SetupName& accept = mowbite_api::SetupAccept();
      json values = json::array();
      for (const auto& value : mowbite_api::SetupAcceptValues()) {
        values.push_back({{"id", value.id}, {"texts", mowbite_api::Texts(value.texts)}});
      }
      inputs.push_back({{"name", accept.id},
                        {"type", "choice"},
                        {"ask", asks},
                        {"advanced", false},
                        {"value", nullptr},
                        {"texts", mowbite_api::Texts(accept.texts)},
                        {"values", std::move(values)}});
      // The delay of each line, to be made shorter once the step is ok: the name of its part, help of the catalog.
      for (const Part& part : step.parts) {
        json texts = mowbite_api::Texts(mowbite_api::SetupDelay().texts);
        for (auto& language : texts.items()) language.value()["name"] = part.texts[language.key()]["name"];
        inputs.push_back({{"name", mowbite_api::kSetupDelayPrefix + part.id},
                          {"type", "number"},
                          {"ask", false},
                          {"advanced", true},
                          {"value", DelayOf(step, part)},
                          {"texts", std::move(texts)},
                          {"unit", "ms"},
                          {"min", 0},
                          {"max", part.delay_ms}});
      }
    }
    json ask;
    if (asks) {
      ask = {{"part", step.parts[run_->part].id}, {"line", run_->proposed}, {"active_high", run_->proposed_high}};
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
    json lines = json::array();
    if (step.check == "input" || step.check == "inputs_assign") {
      for (const Line& line : lines_) {
        lines.push_back({{"name", line.name},
                         {"role", line.role},
                         {"reason", line.reason},
                         {"active", line.active},
                         {"assigned", line.assigned},
                         {"held", line.held},
                         {"delay_ms", line.delay_ms}});
      }
    }
    json progress;
    json ends_at;
    if (is_run && IsAssign(step)) {
      if (run_->assign == Assign::kPart || asks) progress = {{"n", run_->part + 1}, {"of", step.parts.size()}};
      if (run_->assign == Assign::kPart) ends_at = std::floor(run_->part_wall + step.Number("time"));
    } else if (is_run) {
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
                     {"lines", std::move(lines)},
                     {"ask", std::move(ask)},
                     {"progress", std::move(progress)},
                     {"ends_at", std::move(ends_at)},
                     {"open", step.open.empty() ? json() : json(step.open)},
                     {"image", std::move(picture)},
                     {"symbol", step.symbol.empty() ? json() : json(step.symbol)},
                     {"mark", step.mark},
                     {"texts", step.texts},
                     {"result", record.result},
                     {"restart", IsAssign(step)}});
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
          {"restart_pending", restart_wanted_ || (run_ && IsAssign(StepOf(*run_)) && run_->assign == Assign::kRestart)},
          {"stages", std::move(stages)},
          {"steps", std::move(steps)}};
}

}  // namespace mowbite_setup
