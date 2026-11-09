#include "mowbite_logic/logic_core.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "mowbite_api/catalog.h"

namespace mowbite_logic {

namespace {

const std::string kRootPrefix = "mower_logic";
const std::string kIdlePrefix = "mower_logic:idle";
const std::string kRecordingPrefix = "mower_logic:area_recording";
const std::string kMowingPrefix = "mower_logic:mowing";
const std::string kUndockingPrefix = "mower_logic:undocking";
const std::string kDockingPrefix = "mower_logic:docking";

SensorDefinition Range(std::string id, std::string name, std::string unit, SensorMeaning meaning, double min,
                       double max) {
  SensorDefinition s;
  s.id = std::move(id);
  s.name = std::move(name);
  s.unit = std::move(unit);
  s.meaning = meaning;
  s.min = min;
  s.max = max;
  return s;
}

SensorDefinition Temperature(std::string id, std::string name, double min, double max, double critical_high) {
  SensorDefinition s = Range(std::move(id), std::move(name), "deg.C", SensorMeaning::kTemperature, min, max);
  s.critical_high = critical_high;
  return s;
}

}  // namespace

namespace {

EventObserver event_observer = nullptr;

}  // namespace

std::string EmergencyReasonNames(int reason) {
  return mowbite_api::EmergencyNames(reason);
}

void SetEventObserver(EventObserver observer) {
  event_observer = observer;
}

namespace {

// Why mowing pauses or a drive ends for an emergency, worded alike whether hw/status or a refused blade told it.
Reason EmergencyWhy(int reason) {
  return Reason("EMERGENCY", "Emergency: " + EmergencyReasonNames(reason));
}

RecorderConfig RecorderConfigOf(const LogicConfig& config) {
  RecorderConfig recorder = config.recorder;
  recorder.max_position_accuracy = config.max_position_accuracy;
  recorder.dock_backwards = config.dock_contact_x < 0;
  return recorder;
}

std::string Metres(double value) {
  char text[32];
  std::snprintf(text, sizeof(text), "%.2f", value);
  return text;
}

std::string Degrees(double radians) {
  char text[32];
  std::snprintf(text, sizeof(text), "%.0f", radians * 180.0 / M_PI);
  return text;
}

HomeSample HomeSampleOf(const HardwareSnapshot& hw, double now_s) {
  HomeSample sample;
  sample.time = now_s;
  sample.telemetry_ok = hw.telemetry_ok;
  sample.battery_ok = hw.battery_ok;
  sample.battery_voltage = hw.battery_voltage;
  sample.battery_temperature = hw.battery_temperature;
  sample.blade_ok = hw.blade.ok;
  sample.blade_temperature = hw.blade.temperature;
  sample.rain = hw.rain;
  return sample;
}

}  // namespace

std::string BatteryVoltagesProblem(const LogicConfig& config) {
  if (config.battery_critical_voltage > 0 && config.battery_critical_voltage < config.battery_empty_voltage &&
      config.battery_empty_voltage < config.battery_full_voltage) {
    return "";
  }
  return "battery voltages need 0 < battery_critical_voltage < battery_empty_voltage < battery_full_voltage";
}

LogicCore::LogicCore(const LogicConfig& config)
    : config_(config),
      ids_(config.id_seed),
      recorder_(RecorderConfigOf(config)),
      charge_watch_(config.charge),
      home_(HomeRules()) {
  // Without telemetry the mower counts as stopped until the driver reports.
  emergency_ = true;
}

void LogicCore::OnHardwareStatus(const HardwareSnapshot& hw) {
  if (hw.hold != hw_.hold) actions_changed_ = true;
  hw_ = hw;
  recorder_.SetDockContact(DockContact());
  UpdateEmergency();
  if (state_ == State::kDocking) OnDockingContact();
  WatchCharge();
  // From the rpm the MCU reports, not from what was asked for.
  const bool running = hw.telemetry_ok && hw.blade.ok && std::abs(hw.blade.rpm) >= config_.blade_running_rpm;
  if (running != blades_running_) {
    blades_running_ = running;
    AddEvent("BLADES", {{"enabled", running}});
  }
}

void LogicCore::OnHardwareTimeout() {
  hw_.telemetry_ok = false;
  hw_.battery_ok = false;
  recorder_.SetDockContact(false);
  charge_watch_.OnBusy(false);
  UpdateEmergency();
}

void LogicCore::OnPose(const PoseSnapshot& pose) {
  pose_ = pose;
  last_x_ = pose.x;
  last_y_ = pose.y;
  if (state_ != State::kAreaRecording) return;
  recorder_.OnPose(pose.x, pose.y, pose.position_accuracy, pose.orientation_valid);
  PullRecorder();
}

void LogicCore::OnGnssFix(double x, double y) {
  if (state_ != State::kUndocking || undock_stopping_) return;
  // The first fix of the undocking is where the course is measured from; the mower stands or drives straight.
  if (!undock_fix_start_) {
    undock_fix_start_ = Point2{x, y};
    return;
  }
  if (undock_step_ != UndockStep::kDrive) return;
  const auto error = CourseError(undock_fix_start_->x, undock_fix_start_->y, undock_heading_, x, y,
                                 config_.undock.course_distance);
  if (!error || std::abs(*error) <= config_.undock.course_limit) return;
  const std::string text = "Undocking stopped: the mower drove " + Degrees(std::abs(*error)) +
                           " deg off the line of the docking station, at most " + Degrees(config_.undock.course_limit) +
                           " deg allowed: the station may be turned, or its heading was recorded wrong";
  StopUndocking(Reason("path_failed", text), Reason("STOPPED", text));
}

void LogicCore::OnPoseTimeout() {
  pose_.reset();
  if (state_ == State::kDriving) CancelDrive(Reason("POSE_LOST", "No position from the localization"));
  if (state_ == State::kMowing) Pause(kPauseGps, Reason("NO_POSITION", "No position from the localization"));
  if (state_ == State::kUndocking) {
    StopUndocking(Reason("no_gps", "Undocking stopped: no position from the localization"),
                  Reason("POSE_LOST", "No position from the localization"));
  }
  if (state_ == State::kDocking && (docking_leg_ || docking_leg_due_)) {
    CancelDockingLeg(Reason("POSE_LOST", "No position from the localization"));
    WaitForDockingGps();
  }
  if (state_ == State::kCalibrating) EndCalibration(Reason("POSE_LOST", "No position from the localization"), true);
  recorder_.OnPoseLost();
  PullRecorder();
}

void LogicCore::OnMapStatus(MapDatum datum, const std::string& reason) {
  const bool writable = datum == MapDatum::kMatch || datum == MapDatum::kEmpty;
  if (writable != map_writable_) actions_changed_ = true;
  map_writable_ = writable;
  map_reason_ = writable ? Reason() : MapReason(datum, reason);
  recorder_.SetMapWritable(writable, map_reason_);
}

std::optional<MapEdit> LogicCore::TakeMapEdit() {
  return recorder_.TakeMapEdit();
}

void LogicCore::OnMapEditResult(bool ok, const std::string& id, const std::string& code,
                                const std::string& message) {
  recorder_.OnMapEditResult(ok, id, code, message);
  PullRecorder();
}

void LogicCore::PullRecorder() {
  for (auto& event : recorder_.TakeEvents()) AddEvent(event.type, std::move(event.details));
  actions_changed_ |= recorder_.TakeActionsChanged();
}

json LogicCore::MapOverlay() const {
  // The recorder is reset when the state changes, so outside of AREA_RECORDING it has no lines.
  return recorder_.Overlay();
}

bool LogicCore::TakeOverlayChanged() {
  return recorder_.TakeOverlayChanged();
}

std::vector<std::string> LogicCore::TakeNotices() {
  std::vector<std::string> out = recorder_.TakeNotices();
  out.insert(out.end(), notices_.begin(), notices_.end());
  notices_.clear();
  return out;
}

void LogicCore::OnPositionJump(double to_x, double to_y, double distance_m, bool after_outliers) {
  last_x_ = to_x;
  last_y_ = to_y;
  json details = json::object();
  PutReason(&details, "POSITION_JUMP",
            Reason(after_outliers ? "OUTLIERS" : "GAP",
                   after_outliers ? "Back on GPS after outliers" : "Back on GPS after a gap")
                .With("distance", distance_m));
  AddEvent("POSITION_JUMP", std::move(details));
}

// Every event carries the position where it happened, and the client of the command that caused it.
std::string LogicCore::AddEvent(const std::string& type, json details) {
  details["x"] = last_x_;
  details["y"] = last_y_;
  if (!job_.progress().job_id.empty()) details["job_id"] = job_.progress().job_id;
  if (!session_id_.empty()) details["session_id"] = session_id_;
  if (!client_.empty() && !details.contains("client")) details["client"] = client_;
  if (event_observer) event_observer(type, details);
  std::string id = NewId();
  events_.push_back({type, std::move(details), id});
  return id;
}

std::string LogicCore::NewId() {
  static const char kAlphabet[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
  std::uniform_int_distribution<size_t> pick(0, sizeof(kAlphabet) - 2);
  std::string id(32, '0');
  for (auto& c : id) c = kAlphabet[pick(ids_)];
  return id;
}

bool LogicCore::Emergency() const {
  return emergency_;
}

int LogicCore::EmergencyReason() const {
  return hw_.emergency_reason | (hw_.telemetry_ok ? 0 : kEmergencyTimeoutInputs) |
         (restart_hold_ ? kEmergencyUnexpectedRestart : 0);
}

void LogicCore::UpdateEmergency(bool report) {
  const bool emergency = hw_.emergency || !hw_.telemetry_ok || restart_hold_;
  // The driver sends no reason without an emergency; one that does not block is not shown either.
  reason_ = emergency ? EmergencyReason() : 0;
  if (reason_ != logged_reason_) {
    reason_changes_.push_back({logged_reason_, reason_});
    logged_reason_ = reason_;
  }
  // The first report after the start only sets the state, it is no change worth an event.
  if (!hw_seen_) {
    hw_seen_ = report;
    actions_changed_ |= emergency != emergency_;
    emergency_ = emergency;
    return;
  }
  if (emergency != emergency_) {
    emergency_ = emergency;
    if (emergency) teleop_armed_ = false;
    const Reason why = EmergencyWhy(reason_);
    if (emergency && state_ == State::kDriving) CancelDrive(why);
    if (emergency && state_ == State::kMowing) Pause(kPauseEmergency, why);
    // Like OpenMower: the emergency tells, no UNDOCKING_FAILED or DOCKING_FAILED.
    if (emergency && state_ == State::kUndocking) StopUndocking(std::nullopt, why);
    if (emergency && state_ == State::kDocking) EndDocking(why);
    if (emergency && state_ == State::kCalibrating) EndCalibration(why, true);
    // reason stays the bits of OpenMower; resetting is up to the user.
    json details = {{"emergency", emergency},
                    {"reason", reason_},
                    {"severity", emergency ? "error" : "info"},
                    {"resume", emergency ? "user" : "auto"}};
    // The end after emergency.reset or STOP tells who released it (masterplan 3.1a); a new emergency drops the request.
    if (!emergency && release_pending_) {
      details["source"] = release_source_;
      if (!release_client_.empty()) details["client"] = release_client_;
    }
    release_pending_ = false;
    release_answered_s_.reset();
    AddEvent("EMERGENCY", std::move(details));
    actions_changed_ = true;
  }
}

ActionResult LogicCore::OnAction(const std::string& action_id) {
  json params;
  if (const mowbite_api::Command* command = mowbite_api::CommandOfAction(action_id, &params)) {
    const CommandOutcome outcome = Command(command->name, params);
    if (outcome.release) return ActionResult::kReleaseEmergency;
    return outcome.accepted ? ActionResult::kDone : ActionResult::kRejected;
  }
  // start_manual_mowing and stop_manual_mowing have no command: the blade comes after phase 7.
  const bool known_prefix = action_id.rfind(kIdlePrefix + "/", 0) == 0 ||
                            action_id.rfind(kRecordingPrefix + "/", 0) == 0 ||
                            action_id.rfind(kUndockingPrefix + "/", 0) == 0 ||
                            action_id.rfind(kDockingPrefix + "/", 0) == 0;
  return known_prefix ? ActionResult::kRejected : ActionResult::kUnknown;
}

CommandOutcome LogicCore::Command(const std::string& name, const json& params, const Reason& node_problem) {
  CommandOutcome out;
  const mowbite_api::Command* command = mowbite_api::FindCommand(name);
  if (!command) {
    out.invalid = "No command " + name;
    return out;
  }
  out.invalid = mowbite_api::CheckParams(*command, params, &out.invalid_field);
  // open_mower_ros #359: the areas of the job in order; the OS keeps no such list yet.
  if (out.invalid.empty() && name == "mowing.start" && params.contains("areas")) {
    out.invalid = "areas: this OS keeps no list of areas yet, start without it";
    out.invalid_field = "areas";
  }
  if (!out.invalid.empty()) return out;
  client_ = params.is_object() ? params.value("client", "") : "";
  const size_t first_event = events_.size();
  const Reason why = RunCommand(*command, params, node_problem, &out);
  client_.clear();
  if (why.empty()) {
    out.accepted = true;
    out.result = CommandAnswer(out.result);
    return out;
  }
  out.why = why;
  // The refusal that went out as an event of its own names the group; else the group whose reason it is.
  std::string group;
  std::string event_id;
  json action;
  for (size_t i = first_event; i < events_.size(); i++) {
    const Event& event = events_[i];
    const bool refused = event.type.size() > 8 && event.type.compare(event.type.size() - 8, 8, "_REFUSED") == 0;
    if (!refused || event.details.value("reason", "") != why.code) continue;
    group = event.type;
    event_id = event.id;
    if (event.details.contains("action")) action = event.details["action"];
  }
  if (group.empty()) {
    const mowbite_api::EventEntry* answers = mowbite_api::FindReasonGroup("COMMAND_REFUSED");
    for (const auto& reason : answers->reasons) {
      if (reason.code == why.code) group = "COMMAND_REFUSED";
    }
  }
  if (group.empty()) group = command->refused.empty() ? "COMMAND_REFUSED" : command->refused;
  out.data = RefusalData(group, why, event_id);
  if (!action.is_null()) out.data["action"] = action;
  return out;
}

json LogicCore::CommandAnswer(const json& fields) const {
  const RobotStateData state = RobotState();
  json answer = {{"state", state.state}, {"sub_state", state.sub_state}};
  for (const auto& [key, value] : fields.items()) answer[key] = value;
  return answer;
}

json LogicCore::RefusalData(const std::string& group, const Reason& why, const std::string& event_id) const {
  json data = {{"event", group}};
  PutReason(&data, group, why);
  // BUSY brings its own state, what the mower is busy with.
  const RobotStateData state = RobotState();
  if (!data.contains("state")) data["state"] = state.state;
  if (!data.contains("sub_state")) data["sub_state"] = state.sub_state;
  if (!event_id.empty()) data["event_id"] = event_id;
  return data;
}

Reason LogicCore::WrongState() const {
  const RobotStateData state = RobotState();
  const std::string now = state.state + (state.sub_state.empty() ? "" : " " + state.sub_state);
  return Reason("WRONG_STATE", "Not now: the mower is " + now)
      .With("state", state.state)
      .With("sub_state", state.sub_state);
}

Reason LogicCore::RunCommand(const mowbite_api::Command& command, const json& params, const Reason& node_problem,
                             CommandOutcome* out) {
  const std::string& name = command.name;
  if (name == "emergency.reset") {
    release_pending_ = true;
    release_source_ = "rpc";
    release_client_ = client_;
    release_answered_s_.reset();
    out->release = true;
    return {};
  }
  if (name == "mowing.start") {
    Reason why = StartProblem();
    if (why.empty()) why = node_problem;
    if (!why.empty()) {
      RefuseStart(why);
      return why;
    }
    // On the docking station the reference check comes first; its refusal is an event of its own.
    out->result["check"] = DockContact();
    if (DockContact()) {
      BeginDockCheck(DockCheck::kMowing);
    } else {
      StartMowing();
    }
    return {};
  }
  if (name == "mowing.reset_job") {
    if (state_ != State::kIdle) return WrongState();
    if (!job_.Unfinished()) return Reason("NO_JOB", "There is no unfinished job");
    out->result["job_id"] = job_.progress().job_id;
    // The event carries the id of the dropped job, the checkpoint is saved without it.
    AddEvent("JOB_RESET", json::object());
    job_.Reset();
    CheckpointNow();
    actions_changed_ = true;
    return {};
  }
  if (name.rfind("mowing.", 0) == 0) {
    if (state_ != State::kMowing) return WrongState();
    const int waiting = kPauseManual | kPauseEmergency | kPauseFault;
    if (name == "mowing.pause") {
      if (pause_ & kPauseManual) return WrongState();
      Pause(kPauseManual, Reason("MANUAL", "Paused by the app"));
      return {};
    }
    if (name == "mowing.continue") {
      if ((pause_ & waiting) == 0) return WrongState();
      // The EMERGENCY event told already: no MOWING_REFUSED.
      if (emergency_) return Reason("EMERGENCY", "Emergency: reset it first");
      if (Held()) {
        const Reason why = HoldProblem();
        RefuseStart(why);
        return why;
      }
      // Without GPS it still waits for a good position.
      Unpause(waiting);
      return {};
    }
    if (name == "mowing.abort") {
      EndJob(false, true, Reason("MANUAL", "Going home: mowing stopped by the app"));
      return {};
    }
    // skip_area, skip_path: only the area or path the client saw.
    const bool area = name == "mowing.skip_area";
    const char* key = area ? "area" : "path";
    const int now = area ? job_.progress().area : job_.progress().path;
    if (params.contains(key) && params[key].get<double>() != now) {
      return Reason("CHANGED", std::string("The ") + key + " changed meanwhile: nothing was skipped").With(key, now);
    }
    out->result[key] = now;
    (area ? skip_area_ : skip_path_) = true;
    skip_client_ = client_;
    // A leg on its way ends first, its index counts.
    if (leg_ == Leg::kApproach || leg_ == Leg::kFollow) {
      CancelDrive(Reason("STOPPED", "Skipped by the app"));
      stop_requested_ = true;
    } else {
      ApplySkips();
    }
    return {};
  }
  if (name == "undocking.abort") {
    if (state_ != State::kUndocking) return WrongState();
    AbortUndocking(Reason("STOPPED", "Aborted by the app"));
    return {};
  }
  if (name == "docking.abort") {
    if (state_ != State::kDocking) return WrongState();
    notices_.push_back("Docking stopped: aborted by the app");
    EndDocking(Reason("STOPPED", "Aborted by the app"));
    return {};
  }
  if (name == "docking.start") {
    Reason why = DockingProblem();
    if (why.empty()) why = node_problem;
    if (!why.empty()) {
      RefuseDrive(why);
      return why;
    }
    return StartDocking();
  }
  if (name == "recording.start") {
    if (state_ != State::kIdle) return WrongState();
    // The EMERGENCY event told already: no RECORDING_REFUSED.
    if (emergency_) return Reason("EMERGENCY", "Emergency: reset it first");
    const Reason why = Held() ? HoldProblem() : map_writable_ ? Reason() : map_reason_;
    if (!why.empty()) {
      json details = {{"action", "start_area_recording"}};
      PutReason(&details, "RECORDING_REFUSED", why);
      AddEvent("RECORDING_REFUSED", std::move(details));
      return why;
    }
    SetState(State::kAreaRecording);
    return {};
  }
  if (name == "recording.exit") {
    if (state_ != State::kAreaRecording) return WrongState();
    SetState(State::kIdle);
    return {};
  }
  return RecordingCommand(command, params);
}

// A step of the recording: the recorder decides by the action the command stands for.
Reason LogicCore::RecordingCommand(const mowbite_api::Command& command, const json& params) {
  if (state_ != State::kAreaRecording) return WrongState();
  std::string action;
  for (const auto& entry : command.actions) {
    bool fits = true;
    for (const auto& [key, value] : entry.params.items()) fits = fits && params.value(key, json()) == value;
    if (fits) {
      action = entry.action;
      break;
    }
  }
  if (action.rfind(kRecordingPrefix + "/", 0) != 0) return WrongState();
  Reason why;
  const AreaRecorder::Result result = recorder_.OnAction(action.substr(kRecordingPrefix.size() + 1), &why);
  PullRecorder();
  if (result == AreaRecorder::Result::kDone) return {};
  return why.empty() ? WrongState() : why;
}

std::optional<std::pair<double, double>> LogicCore::OnTeleop(double vx, double vz, double now_s) {
  const double quiet_s = now_s - last_teleop_s_;
  last_teleop_s_ = now_s;
  if (state_ != State::kAreaRecording || emergency_) {
    teleop_armed_ = false;
    return std::nullopt;
  }
  if (!teleop_armed_) {
    const bool zero = vx == 0.0 && vz == 0.0;
    if (!zero && quiet_s < kTeleopQuietS) return std::nullopt;
    teleop_armed_ = true;
  }
  return std::make_pair(vx, vz);
}

void LogicCore::OnCapabilities(bool safety_measured, bool charge_stop) {
  safety_measured_ = safety_measured;
  charge_stop_ = charge_stop;
  if (safety_measured) return;
  if (state_ == State::kDriving) {
    CancelDrive(Reason("NOT_ALLOWED", "No safety measurement of this platform (hw/capabilities)"));
  }
  if (state_ == State::kUndocking) {
    const std::string text = "Undocking stopped: no safety measurement of this platform (hw/capabilities)";
    StopUndocking(Reason("path_failed", text), Reason("NOT_ALLOWED", text));
  }
  if (state_ == State::kCalibrating) {
    EndCalibration(Reason("NO_SAFETY_MEASUREMENT", "No safety measurement of this platform (hw/capabilities)"), true);
  }
  if (state_ == State::kDocking) {
    const bool approach = docking_last_kind_ == DockingLeg::Kind::kApproach;
    CountDockingFailure(Reason(approach ? "approach_failed" : "dock_failed",
                               "Docking stopped: no safety measurement of this platform (hw/capabilities)"),
                        true);
  }
  Pause(kPauseFault, Reason("NO_SAFETY_MEASUREMENT", "No safety measurement of this platform (hw/capabilities)"));
}

Reason LogicCore::DriveProblem() const {
  Reason why = Busy();
  if (why.empty()) why = HoldProblem();
  return why.empty() ? MoveProblem() : why;
}

Reason LogicCore::MoveProblem() const {
  Reason why = Unsafe();
  if (why.empty()) why = PositionProblem();
  if (why.empty()) why = DockProblem();
  return why;
}

Reason LogicCore::StartDrive(double x, double y) {
  const Reason why = DriveProblem();
  if (!why.empty()) {
    RefuseDrive(why);
    return why;
  }
  BeginDrive(x, y);
  return {};
}

void LogicCore::BeginDrive(double x, double y) {
  drive_cancel_ = false;
  cancel_reason_.reset();
  SetState(State::kDriving);
  AddEvent("DRIVE", {{"phase", "start"}, {"goal_x", x}, {"goal_y", y}});
}

void LogicCore::RefuseDrive(const Reason& why) {
  json details = json::object();
  PutReason(&details, "DRIVE_REFUSED", why);
  AddEvent("DRIVE_REFUSED", std::move(details));
}

void LogicCore::OnDriveEnded(bool arrived, const Reason& reason) {
  if (state_ != State::kDriving) return;
  json details = {{"phase", "end"}, {"arrived", arrived}};
  PutReason(&details, "DRIVE", reason);
  AddEvent("DRIVE", std::move(details));
  drive_cancel_ = false;
  cancel_reason_.reset();
  SetState(State::kIdle);
}

bool LogicCore::Driving() const {
  return state_ == State::kDriving;
}

bool LogicCore::FollowerActive() const {
  if (emergency_) return false;
  if (state_ == State::kDriving) return true;
  if (state_ == State::kUndocking) return undock_step_ == UndockStep::kDrive && !undock_stopping_;
  if (state_ == State::kDocking) return docking_leg_.has_value() && !docking_hold_;
  return state_ == State::kMowing && pause_ == 0 && (leg_ == Leg::kApproach || leg_ == Leg::kFollow);
}

std::optional<Reason> LogicCore::TakeDriveCancel() {
  if (!drive_cancel_) return std::nullopt;
  drive_cancel_ = false;
  Reason reason = cancel_reason_.value_or(Reason("STOPPED", "Stopped by the logic"));
  cancel_reason_.reset();
  return reason;
}

void LogicCore::CancelDrive(const Reason& reason) {
  drive_cancel_ = true;
  if (!cancel_reason_) cancel_reason_ = reason;
}

void LogicCore::SetState(State state) {
  if (state == state_) return;
  if (state_ == State::kAreaRecording || state_ == State::kDriving || state_ == State::kMowing ||
      state_ == State::kUndocking || state_ == State::kDocking || state_ == State::kCalibrating) {
    stop_requested_ = true;
  }
  // A session is one run of mowing; the job lasts until it is finished or reset.
  if (state == State::kMowing) {
    if (job_.progress().job_id.empty()) job_.SetJobId(NewId());
    session_id_ = NewId();
  }
  if (state_ == State::kMowing) {
    session_id_.clear();
    if (leg_ == Leg::kApproach || leg_ == Leg::kFollow) CancelDrive(Reason("STOPPED", "Mowing ended"));
    leg_ = Leg::kNone;
    pause_ = 0;
    pause_reason_ = Reason();
    skip_path_ = skip_area_ = false;
    spinup_since_.reset();
    gps_good_since_.reset();
  }
  if (state_ == State::kMowing || state == State::kMowing) {
    home_.Reset();
    home_told_ = 0;
  }
  // Entering and leaving both start from nothing; what was not saved is gone, like with OpenMower.
  recorder_.Reset();
  recorder_.TakeActionsChanged();
  state_ = state;
  teleop_armed_ = false;
  json details = {{"state", CurrentStateName()}};
  if (state == State::kDriving) details["sub_state"] = "DRIVING";
  if (state == State::kCalibrating) details["sub_state"] = "CALIBRATING";
  AddEvent("STATE", std::move(details));
  actions_changed_ = true;
}

std::string LogicCore::CurrentStateName() const {
  if (state_ == State::kMowing && pause_ != 0) return "PAUSED";
  return StateName(state_);
}

const char* LogicCore::StateName(State state) {
  switch (state) {
    case State::kAreaRecording: return "AREA_RECORDING";
    case State::kMowing: return "MOWING";
    case State::kUndocking: return "UNDOCKING";
    case State::kDocking: return "DOCKING";
    // OpenMower has no state for driving or calibrating without mowing; the sub state tells.
    case State::kDriving:
    case State::kCalibrating:
    case State::kIdle:
    default: return "IDLE";
  }
}

RobotStateData LogicCore::RobotState() const {
  RobotStateData s;
  s.state = CurrentStateName();
  if (state_ == State::kMowing) {
    s.current_area = job_.progress().area;
    s.current_path = job_.progress().path;
    s.current_path_index = job_.progress().index;
  }
  if (state_ == State::kAreaRecording) s.sub_state = recorder_.SubState();
  if (state_ == State::kDriving) s.sub_state = "DRIVING";
  if (state_ == State::kCalibrating) s.sub_state = "CALIBRATING";
  s.emergency = emergency_;
  s.emergency_reason = reason_;
  s.is_charging = hw_.battery_ok && hw_.dock_contact;
  s.rain_detected = hw_.telemetry_ok && hw_.rain;
  if (hw_.battery_ok) {
    s.battery_percentage = std::clamp(hw_.battery_percentage / 100.0f, 0.0f, 1.0f);
  }
  if (pose_) {
    s.pose_x = pose_->x;
    s.pose_y = pose_->y;
    s.pose_heading = pose_->heading;
    s.pos_accuracy = pose_->position_accuracy;
    s.heading_accuracy = pose_->orientation_accuracy;
    s.heading_valid = pose_->orientation_valid;
    if (pose_->position_accuracy <= config_.max_position_accuracy && pose_->orientation_valid) {
      s.gps_percentage = static_cast<float>(
          1.0 - std::min(1.0, pose_->position_accuracy / config_.max_position_accuracy));
    }
  }
  return s;
}

std::vector<ActionGroup> LogicCore::Actions() const {
  const bool idle = state_ == State::kIdle;
  const bool recording = state_ == State::kAreaRecording;
  const bool mowing = state_ == State::kMowing;
  std::vector<ActionGroup> groups;
  groups.push_back({kRootPrefix, {{"reset_emergency", "Reset Emergency", true}}});
  const Reason start = StartProblem();
  groups.push_back({kIdlePrefix,
                    {{"start_mowing", "Start Mowing", start.empty(), start.code},
                     {"start_area_recording", "Start Area Recording", idle && !emergency_ && map_writable_ && !Held()},
                     {"reset_job", "Reset Job", idle && job_.Unfinished()}}});
  // Pause and continue take turns, like in OpenMower; continue waits for the emergency to be reset.
  const bool waits = (pause_ & (kPauseManual | kPauseEmergency | kPauseFault)) != 0;
  groups.push_back({kMowingPrefix,
                    {{"pause", "Pause Mowing", mowing && !waits},
                     {"continue", "Continue Mowing", mowing && waits && !emergency_ && !Held()},
                     {"abort_mowing", "Stop Mowing", mowing},
                     {"skip_area", "Skip Area", mowing},
                     {"skip_path", "Skip Path", mowing}}});
  groups.push_back({kUndockingPrefix, {{"abort_undocking", "Stop Undocking", state_ == State::kUndocking}}});
  groups.push_back({kDockingPrefix, {{"abort_docking", "Stop Docking", state_ == State::kDocking}}});
  // The recorder decides while recording; outside of it all are off.
  const std::vector<bool> enabled = recorder_.Enabled();
  const std::vector<std::string> names = {"Start Recording",
                                          "Stop Recording",
                                          "Save Navigation Area",
                                          "Save Mowing Area",
                                          "Exit",
                                          "Discard Area",
                                          "Record Docking point",
                                          "Enable automatic point collecting",
                                          "Disable automatic point collecting",
                                          "Collect point",
                                          "Start manual mowing",
                                          "Stop manual mowing"};
  ActionGroup recording_group{kRecordingPrefix, {}};
  for (size_t i = 0; i < names.size(); i++) {
    recording_group.actions.push_back({AreaRecorder::ActionNames()[i], names[i], recording && enabled[i]});
  }
  groups.push_back(std::move(recording_group));
  return groups;
}

std::vector<SensorDefinition> LogicCore::Sensors() const {
  std::vector<SensorDefinition> sensors;
  SensorDefinition battery =
      Range("om_v_battery", "V Battery", "V", SensorMeaning::kVoltage, config_.battery_empty_voltage,
            config_.battery_full_voltage);
  battery.critical_low = config_.battery_empty_voltage;
  sensors.push_back(battery);
  // The MCU reports neither a charge voltage nor a charger state: both are derived, the names say so.
  sensors.push_back(
      Range("om_v_charge", "V Charge (dock contact)", "V", SensorMeaning::kVoltage, 0, config_.dock_charge_voltage + 2));
  sensors.push_back(Range("om_charge_current", "Charge Current", "A", SensorMeaning::kCurrent, 0, 7));
  SensorDefinition charge_state;
  charge_state.id = "om_charge_state";
  charge_state.name = "Charge State (derived)";
  charge_state.text = true;
  sensors.push_back(charge_state);
  sensors.push_back(Range("om_mow_motor_rpm", "Mow Motor Revolutions", "rpm", SensorMeaning::kRevolutions, 0, 3500));
  sensors.push_back(Range("om_mow_motor_current", "Mow Motor Current", "A", SensorMeaning::kCurrent, 0, 5));
  // One temperature per motor board, so no separate mow ESC temperature. Own limits: the drive motors hold position
  // up to 68 degC, warn above 80.
  SensorDefinition gps_accuracy;
  gps_accuracy.id = "om_gps_accuracy";
  gps_accuracy.name = "GPS Accuracy";
  gps_accuracy.unit = "m";
  gps_accuracy.meaning = SensorMeaning::kDistance;
  sensors.push_back(gps_accuracy);
  sensors.push_back(Temperature("om_mow_motor_temp", "Mow Motor Temp", 0, 100, 80));
  sensors.push_back(Temperature("om_left_esc_temp", "Left ESC Temp", 0, 100, 80));
  sensors.push_back(Temperature("om_right_esc_temp", "Right ESC Temp", 0, 100, 80));
  // From the BMS. Own limits for Li-ion: warn above 55 degC.
  sensors.push_back(Temperature("battery_temp", "Battery Temp", -10, 70, 55));
  return sensors;
}

std::vector<SensorReading> LogicCore::Readings() const {
  std::vector<SensorReading> readings;
  if (hw_.battery_ok) {
    const double charge_current = hw_.battery_current < 0 ? -hw_.battery_current : 0.0;
    readings.push_back({"om_v_battery", hw_.battery_voltage, ""});
    readings.push_back({"om_v_charge", hw_.dock_contact ? config_.dock_charge_voltage : 0.0, ""});
    readings.push_back({"om_charge_current", charge_current, ""});
    // Texts of the charger of OpenMower v2, the app's battery script reads them.
    std::string state = "Not Charging";
    if (hw_.dock_contact && charge_current > 0.1) {
      state = "Fast Charge";
    } else if (hw_.dock_contact && hw_.battery_percentage >= 99) {
      state = "Done";
    }
    readings.push_back({"om_charge_state", 0, state});
    readings.push_back({"battery_temp", static_cast<double>(hw_.battery_temperature), ""});
  }
  if (pose_) readings.push_back({"om_gps_accuracy", pose_->position_accuracy, ""});
  if (hw_.blade.ok) {
    readings.push_back({"om_mow_motor_rpm", static_cast<double>(hw_.blade.rpm), ""});
    readings.push_back({"om_mow_motor_current", hw_.blade.current, ""});
    readings.push_back({"om_mow_motor_temp", static_cast<double>(hw_.blade.temperature), ""});
  }
  if (hw_.left.ok) readings.push_back({"om_left_esc_temp", static_cast<double>(hw_.left.temperature), ""});
  if (hw_.right.ok) readings.push_back({"om_right_esc_temp", static_cast<double>(hw_.right.temperature), ""});
  return readings;
}

std::vector<Event> LogicCore::TakeEvents() {
  std::vector<Event> out;
  out.swap(events_);
  return out;
}

bool LogicCore::TakeActionsChanged() {
  bool changed = actions_changed_;
  actions_changed_ = false;
  // Start Mowing and its reason follow the position, the battery and more, which change without an event.
  const std::string start_reason = StartProblem().code;
  if (start_reason != last_start_reason_) {
    last_start_reason_ = start_reason;
    changed = true;
  }
  return changed;
}

std::vector<ReasonChange> LogicCore::TakeReasonChanges() {
  std::vector<ReasonChange> out;
  out.swap(reason_changes_);
  return out;
}

bool LogicCore::TakeStopRequest() {
  bool stop = stop_requested_;
  stop_requested_ = false;
  return stop;
}

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
  const std::vector<Reason> problems = CalibrationProblems(now_s);
  if (!problems.empty()) {
    RefuseCalibration(problems.front());
    return problems.front();
  }
  const CalibrationStatus status = *CalibrationStatusNow(now_s);
  calibration_goal_due_ = CalibrationGoal{status.heading, status.lane_length};
  calibration_cancel_ = false;
  calibration_sign_s_ = now_s;
  calibration_phase_ = "start";
  calibration_index_ = 0;
  home_.Reset();
  SetState(State::kCalibrating);
  AddEvent("CALIBRATION", {{"phase", "start"}, {"index", 0}});
  notices_.push_back("Calibration: lanes of " + Metres(status.lane_length) + " m, heading " +
                     Degrees(status.heading) + " deg");
  return {};
}

void LogicCore::RefuseCalibration(const Reason& why) {
  json details = json::object();
  PutReason(&details, "CALIBRATION_REFUSED", why);
  AddEvent("CALIBRATION_REFUSED", std::move(details));
}

std::optional<CalibrationGoal> LogicCore::TakeCalibrationGoal() {
  std::optional<CalibrationGoal> goal = calibration_goal_due_;
  calibration_goal_due_.reset();
  return goal;
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
    notices_.push_back("Calibration done: " + outcome.summary);
    EndCalibration(std::nullopt, false);
    return;
  }
  // A program that ends without saying why counts as gone.
  EndCalibration(Reason(outcome.reason.empty() ? "CALIBRATION_MISSING" : outcome.reason, outcome.message), false);
}

bool LogicCore::AbortCalibration(const Reason& why) {
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

bool LogicCore::TakeCalibrationCancel() {
  const bool cancel = calibration_cancel_;
  calibration_cancel_ = false;
  return cancel;
}

void LogicCore::EndCalibration(const std::optional<Reason>& failure, bool cancel) {
  if (failure) {
    json details = json::object();
    PutReason(&details, "CALIBRATION_FAILED", *failure);
    AddEvent("CALIBRATION_FAILED", std::move(details));
    notices_.push_back("Calibration stopped: " + failure->message);
  }
  AddEvent("CALIBRATION", {{"phase", "end"}, {"index", calibration_index_}});
  calibration_cancel_ = calibration_cancel_ || cancel;
  calibration_goal_due_.reset();
  home_.Reset();
  // One zero follows from leaving the state; the mower stays where it stopped.
  SetState(State::kIdle);
}

void LogicCore::OnMowMap(std::vector<JobArea> areas, const Reason& problem) {
  job_.SetAreas(std::move(areas));
  mow_map_problem_ = problem;
  if (state_ == State::kMowing && !problem.empty()) Pause(kPauseFault, problem);
}

void LogicCore::RestoreJob(const JobProgress& progress) {
  job_.Restore(progress);
  actions_changed_ = true;
}

bool LogicCore::PoseGood() const {
  return pose_ && pose_->position_accuracy <= config_.max_position_accuracy && pose_->orientation_valid;
}

Reason LogicCore::Busy() const {
  if (state_ == State::kIdle) {
    if (dock_check_ == DockCheck::kNone) return {};
    return Reason("BUSY", "The mower is busy: checking its position on the docking station").With("state", "checking");
  }
  const std::string state = state_ == State::kDriving       ? "driving"
                            : state_ == State::kCalibrating ? "calibrating"
                                                            : CurrentStateName();
  return Reason("BUSY", "The mower is busy: " + state).With("state", state);
}

Reason LogicCore::Unsafe() const {
  if (emergency_) return Reason("EMERGENCY", "Emergency: reset it first");
  if (!safety_measured_) {
    return Reason("NO_SAFETY_MEASUREMENT",
                  "No safety measurement of this platform (hw/capabilities): mowing and driving are blocked");
  }
  return {};
}

Reason LogicCore::AccuracyReason(float accuracy, double limit) const {
  char text[160];
  std::snprintf(text, sizeof(text), "Position accuracy %.3f m is worse than %.3f m", accuracy, limit);
  return Reason("POSITION_ACCURACY", text).With("accuracy", accuracy).With("limit", limit);
}

Reason LogicCore::PositionProblem() const {
  if (!pose_) return Reason("NO_POSITION", "No position from the localization");
  // On the docking station the reference check sets the heading, and it wants a better position.
  const double limit = DockContact() ? DockAccuracyLimit() : config_.max_position_accuracy;
  if (!(pose_->position_accuracy <= limit)) {
    Reason why = AccuracyReason(pose_->position_accuracy, limit);
    if (DockContact()) why.message += " on the docking station";
    return why;
  }
  if (!pose_->orientation_valid && !DockContact()) {
    return Reason("NO_HEADING", "The heading is not aligned yet");
  }
  return {};
}

Reason LogicCore::DockProblem() const {
  if (!DockContact()) return {};
  if (!dock_) return Reason("NO_DOCK", "The mower is on a docking station, but the map has none: record it first");
  return UndockEndProblem(dock_->x, dock_->y);
}

Reason LogicCore::UndockEndProblem(double x, double y) const {
  // Contacts in front: undocking backwards is not built, the reference check ends in ON_DOCK.
  if (config_.dock_contact_x >= 0) return {};
  const double heading = DockedHeading();
  const double distance = config_.undock.distance;
  if (Reachable(drivable_, obstacles_, x + distance * std::cos(heading), y + distance * std::sin(heading))) return {};
  return Reason("DOCK_UNREACHABLE", "Leaving the docking station " + Metres(distance) +
                                        " m straight ahead ends outside of the mowing and navigation areas: let an "
                                        "area reach there, or place the station closer to one")
      .With("distance", distance);
}

void LogicCore::OnDrivableAreas(std::vector<Ring> drivable, std::vector<Ring> obstacles, const Reason& problem) {
  drivable_ = std::move(drivable);
  obstacles_ = std::move(obstacles);
  drivable_problem_ = problem;
}

void LogicCore::SetUndockDistance(double distance) {
  config_.undock.distance = distance;
}

double LogicCore::DockAccuracyLimit() const {
  return std::min(config_.dock_max_position_accuracy, config_.max_position_accuracy);
}

bool LogicCore::DockContact() const {
  return hw_.telemetry_ok && hw_.dock_contact;
}

void LogicCore::OnMapDock(const std::optional<DockPose>& dock) {
  dock_ = dock;
}

double LogicCore::DockedHeading() const {
  const double heading = dock_ ? dock_->heading : 0.0;
  const double facing = config_.dock_contact_x < 0 ? heading + M_PI : heading;
  return std::atan2(std::sin(facing), std::cos(facing));
}

void LogicCore::BeginDockCheck(DockCheck purpose) {
  dock_check_ = purpose;
  dock_check_due_ = true;
  actions_changed_ = true;
  notices_.push_back("On the docking station: checking the position against the recorded one first");
}

void LogicCore::CheckDockBeforeDrive(double x, double y) {
  drive_x_ = x;
  drive_y_ = y;
  BeginDockCheck(DockCheck::kDrive);
}

std::optional<double> LogicCore::TakeDockCheck() {
  if (!dock_check_due_) return std::nullopt;
  dock_check_due_ = false;
  return DockedHeading();
}

bool LogicCore::DockCheckPending() const {
  return dock_check_ != DockCheck::kNone;
}

Reason LogicCore::OnDockCheck(bool ok, double x, double y, float accuracy, const std::string& message) {
  const DockCheck purpose = dock_check_;
  if (purpose == DockCheck::kNone) return Reason("STOPPED", "No reference check was on its way");
  dock_check_ = DockCheck::kNone;
  dock_check_due_ = false;
  actions_changed_ = true;
  // What changed meanwhile comes first: an emergency, the contact, then the rest of a start or a drive.
  Reason why = Unsafe();
  if (why.empty() && !DockContact()) {
    why = Reason("NO_DOCK_CONTACT", "The contact with the docking station ended during the reference check");
  }
  if (why.empty()) why = purpose == DockCheck::kMowing ? StartProblem() : DriveProblem();
  if (why.empty() && !ok) why = Reason("NO_POSITION", message.empty() ? "No answer from the localization" : message);
  if (why.empty() && !(accuracy <= DockAccuracyLimit())) {
    why = AccuracyReason(accuracy, DockAccuracyLimit());
    why.message += " on the docking station";
  }
  if (why.empty()) {
    const double distance = std::hypot(x - dock_->x, y - dock_->y);
    const double limit = config_.dock_reference_limit;
    if (distance > limit) {
      why = Reason("REFERENCE_MISMATCH", "On the docking station the position is " + Metres(distance) +
                                             " m from the recorded one, at most " + Metres(limit) +
                                             " m allowed: the corrections may have another reference, or the "
                                             "station was moved")
                .With("distance", distance)
                .With("limit", limit);
    } else {
      notices_.push_back("Reference check on the docking station passed: " + Metres(distance) +
                         " m from the recorded position");
      if (config_.dock_contact_x >= 0) {
        why = Reason("ON_DOCK", "The position on the docking station fits, but undocking backwards is not built yet: "
                                "drive the mower off by remote control first")
                  .With("distance", distance)
                  .With("limit", limit);
      } else {
        // The way out from where the mower stands.
        why = UndockEndProblem(x, y);
      }
    }
  }
  if (why.empty()) {
    StartUndocking(purpose, x, y);
    return why;
  }
  if (purpose == DockCheck::kMowing) {
    RefuseStart(why);
  } else {
    RefuseDrive(why);
  }
  return why;
}

void LogicCore::StartUndocking(DockCheck purpose, double x, double y) {
  undock_for_ = purpose;
  undock_step_ = UndockStep::kChargeOff;
  undock_x_ = x;
  undock_y_ = y;
  undock_heading_ = DockedHeading();
  undock_fix_start_.reset();
  charge_off_due_ = charge_stop_;
  undock_path_due_.reset();
  undock_stopping_ = false;
  undock_failure_.reset();
  undock_stop_why_ = Reason();
  if (charge_stop_) {
    notices_.push_back("Undocking: charging off, then " + Metres(config_.undock.distance) + " m straight ahead");
  } else {
    // The hardware charges by itself and cannot stop it: straight to the path.
    notices_.push_back("Undocking: " + Metres(config_.undock.distance) + " m straight ahead");
    undock_step_ = UndockStep::kDrive;
    undock_path_due_ = PlanUndock(undock_x_, undock_y_, undock_heading_, config_.undock);
  }
  SetState(State::kUndocking);
}

bool LogicCore::Undocking() const {
  return state_ == State::kUndocking;
}

bool LogicCore::UndockingForDrive() const {
  return state_ == State::kUndocking && undock_for_ == DockCheck::kDrive;
}

bool LogicCore::TakeChargeOff() {
  const bool due = charge_off_due_;
  charge_off_due_ = false;
  return due;
}

void LogicCore::OnChargeOff(bool ok, const std::string& message) {
  if (state_ == State::kDocking && docking_out_after_charge_off_) {
    DockingChargeOff(ok, message);
    return;
  }
  if (state_ != State::kUndocking || undock_step_ != UndockStep::kChargeOff) return;
  if (!ok) {
    const std::string text = "Undocking failed: charging could not be switched off (" + message + ")";
    EndUndocking(Reason("path_failed", text), Reason("STOPPED", text));
    return;
  }
  undock_step_ = UndockStep::kDrive;
  undock_path_due_ = PlanUndock(undock_x_, undock_y_, undock_heading_, config_.undock);
}

std::optional<UndockPath> LogicCore::TakeUndockPath() {
  std::optional<UndockPath> path;
  path.swap(undock_path_due_);
  return path;
}

void LogicCore::StopUndocking(const std::optional<Reason>& failure, const Reason& why) {
  if (state_ != State::kUndocking || undock_stopping_) return;
  // Before the path went to the follower nothing moves: it ends at once.
  if (undock_step_ == UndockStep::kChargeOff || undock_path_due_) {
    EndUndocking(failure, why);
    return;
  }
  undock_stopping_ = true;
  undock_failure_ = failure;
  undock_stop_why_ = why;
  undock_path_due_.reset();
  CancelDrive(why);
  stop_requested_ = true;
  actions_changed_ = true;
}

void LogicCore::AbortUndocking(const Reason& why) {
  StopUndocking(std::nullopt, why);
}

void LogicCore::OnUndockEnded(LegEnd end, const Reason& reason) {
  if (state_ != State::kUndocking || undock_step_ != UndockStep::kDrive) return;
  if (end == LegEnd::kDone && !undock_stopping_) {
    Undocked();
    return;
  }
  // Stopped from here: the cause decided already. Otherwise the follower's: an emergency tells by itself.
  if (undock_stopping_) {
    EndUndocking(undock_failure_, undock_stop_why_);
  } else if (end == LegEnd::kEmergency) {
    EndUndocking(std::nullopt, reason);
  } else if (end == LegEnd::kGps) {
    const Reason failure("no_gps", "Undocking stopped: " + reason.message);
    EndUndocking(failure, failure);
  } else {
    const Reason failure("path_failed", "Undocking failed: " + reason.message);
    EndUndocking(failure, failure);
  }
}

void LogicCore::EndUndocking(const std::optional<Reason>& failure, const Reason& why) {
  if (failure) {
    json details = json::object();
    PutReason(&details, "UNDOCKING_FAILED", *failure);
    AddEvent("UNDOCKING_FAILED", std::move(details));
  }
  if (!failure) notices_.push_back("Undocking stopped: " + why.message);
  UndockOutcome outcome;
  outcome.for_drive = undock_for_ == DockCheck::kDrive;
  outcome.why = failure.value_or(why);
  undock_outcome_ = outcome;
  // The mower may still stand on the contacts.
  charge_on_due_ = charge_stop_;
  charge_off_due_ = false;
  undock_path_due_.reset();
  undock_stopping_ = false;
  undock_for_ = DockCheck::kNone;
  SetState(State::kIdle);
}

void LogicCore::Undocked() {
  AddEvent("UNDOCKED", json::object());
  notices_.push_back("Undocked");
  UndockOutcome outcome;
  outcome.undocked = true;
  outcome.for_drive = undock_for_ == DockCheck::kDrive;
  undock_for_ = DockCheck::kNone;
  // On like a start away from the station; what changed meanwhile comes first.
  outcome.why = outcome.for_drive ? MoveProblem() : MowProblem();
  if (!outcome.why.empty()) {
    if (outcome.for_drive) {
      RefuseDrive(outcome.why);
    } else {
      RefuseStart(outcome.why);
    }
    SetState(State::kIdle);
  } else if (outcome.for_drive) {
    BeginDrive(drive_x_, drive_y_);
  } else {
    StartMowing();
  }
  undock_outcome_ = outcome;
}

bool LogicCore::TakeChargeOn() {
  const bool due = charge_on_due_;
  charge_on_due_ = false;
  return due;
}

std::optional<UndockOutcome> LogicCore::TakeUndockOutcome() {
  std::optional<UndockOutcome> outcome;
  outcome.swap(undock_outcome_);
  return outcome;
}

bool LogicCore::CanDock() const {
  return config_.dock_contact_x < 0;
}

bool LogicCore::Docking() const {
  return state_ == State::kDocking;
}

void LogicCore::SetDockingSettings(double approach_distance, int retry_count) {
  config_.dock.approach_distance = approach_distance;
  config_.dock.retry_count = retry_count;
}

DockingSequence LogicCore::NewDocking() const {
  return DockingSequence(config_.dock, config_.undock, dock_.value_or(DockPose()), DockedHeading());
}

Reason LogicCore::ApproachProblem() const {
  const Point2 p = NewDocking().ApproachPoint();
  if (Reachable(drivable_, obstacles_, p.x, p.y)) return {};
  const double distance = config_.dock.approach_distance;
  return Reason("DOCK_UNREACHABLE", "The way onto the docking station begins " + Metres(distance) +
                                        " m in front of it, outside of the mowing and navigation areas: let an area "
                                        "reach there, or place the station closer to one")
      .With("distance", distance);
}

Reason LogicCore::DockingProblem() const {
  Reason why = Busy();
  if (why.empty()) why = HoldProblem();
  if (!why.empty()) return why;
  if (DockContact()) return Reason("ALREADY_THERE", "The mower is on the docking station already");
  why = Unsafe();
  if (why.empty()) why = PositionProblem();
  if (why.empty()) why = drivable_problem_;
  if (why.empty() && !dock_) why = Reason("NO_DOCK", "The map has no docking station: record it first");
  if (why.empty() && !CanDock()) why = Reason("NO_DOCK", "Docking onto contacts in front is not built");
  if (why.empty()) why = ApproachProblem();
  return why;
}

Reason LogicCore::StartDocking() {
  const Reason why = DockingProblem();
  if (!why.empty()) {
    RefuseDrive(why);
    return why;
  }
  json details = json::object();
  PutReason(&details, "DOCKING", Reason("MANUAL", "Going home: sent by the app"));
  AddEvent("DOCKING", std::move(details));
  BeginDocking();
  return {};
}

void LogicCore::GoHome(const Reason& home_why) {
  // Never after an emergency or without the safety measurement: the mower does not drive off by itself then.
  if (!CanDock() || !dock_ || !Unsafe().empty()) {
    SetState(State::kIdle);
    return;
  }
  // On the contacts already: DOCKED once the watch confirms the charge there, as for any stay on the station.
  if (DockContact()) {
    charge_watch_.Unconfirm(false);
    SetState(State::kIdle);
    return;
  }
  Reason why = drivable_problem_;
  if (why.empty()) why = ApproachProblem();
  if (!why.empty()) {
    // An approach that cannot succeed fails like OpenMower's: DOCKING_FAILED approach_failed, not DOCK_UNREACHABLE.
    json details = json::object();
    PutReason(&details, "DOCKING_FAILED", Reason("approach_failed", why.message).With("attempts", 1));
    AddEvent("DOCKING_FAILED", std::move(details));
    notices_.push_back("Docking failed: " + why.message);
    SetState(State::kIdle);
    return;
  }
  if (!home_why.empty()) {
    json details = json::object();
    PutReason(&details, "DOCKING", home_why);
    AddEvent("DOCKING", std::move(details));
    notices_.push_back(home_why.message);
  }
  BeginDocking();
}

bool LogicCore::HomeReachable() const {
  return CanDock() && dock_ && Unsafe().empty() && !DockContact() && drivable_problem_.empty() &&
         ApproachProblem().empty();
}

HomeConfig LogicCore::HomeRules() const {
  HomeConfig rules;
  rules.critical_voltage = config_.battery_critical_voltage;
  rules.critical_time = config_.battery_critical_time;
  rules.empty_voltage = config_.battery_empty_voltage;
  rules.low_window = config_.battery_low_window;
  rules.battery_hot_temperature = config_.battery_hot_temperature;
  rules.motor_hot_temperature = config_.motor_hot_temperature;
  rules.rain_mode = config_.rain_mode;
  rules.rain_check_seconds = config_.rain_check_seconds;
  return rules;
}

void LogicCore::SetHomeSettings(int rain_mode, double rain_check_seconds, double motor_hot_temperature,
                                double motor_cold_temperature) {
  config_.rain_mode = rain_mode;
  config_.rain_check_seconds = rain_check_seconds;
  config_.motor_hot_temperature = motor_hot_temperature;
  config_.motor_cold_temperature = motor_cold_temperature;
  home_.SetConfig(HomeRules());
}

Reason LogicCore::HomeWhy(HomeReason reason) const {
  char text[160];
  switch (reason) {
    case HomeReason::kBatteryCritical:
      std::snprintf(text, sizeof(text), "the battery at %.1f V stayed below %.1f V for %.0f s",
                    hw_.battery_voltage, config_.battery_critical_voltage, config_.battery_critical_time);
      return Reason("BATTERY_CRITICAL", text)
          .With("battery_voltage", hw_.battery_voltage)
          .With("limit", config_.battery_critical_voltage);
    case HomeReason::kBatteryHot:
      std::snprintf(text, sizeof(text), "the battery at %d °C, %.0f °C or more", hw_.battery_temperature,
                    config_.battery_hot_temperature);
      return Reason("BATTERY_HOT", text)
          .With("temperature", static_cast<int>(hw_.battery_temperature))
          .With("limit", config_.battery_hot_temperature);
    case HomeReason::kBatteryLow:
      std::snprintf(text, sizeof(text), "the battery stayed below %.1f V for %.0f s (at most %.1f V)",
                    config_.battery_empty_voltage, config_.battery_low_window, home_.WindowVoltage());
      return Reason("BATTERY_LOW", text)
          .With("battery_voltage", home_.WindowVoltage())
          .With("limit", config_.battery_empty_voltage);
    case HomeReason::kMotorHot:
      std::snprintf(text, sizeof(text), "the blade motor at %d °C, %.0f °C or more",
                    hw_.blade.temperature, config_.motor_hot_temperature);
      return Reason("MOTOR_HOT", text)
          .With("temperature", static_cast<int>(hw_.blade.temperature))
          .With("limit", config_.motor_hot_temperature);
    case HomeReason::kRain:
    default:
      std::snprintf(text, sizeof(text), "rain for %.0f s", config_.rain_check_seconds);
      return Reason("RAIN", text);
  }
}

void LogicCore::BatteryEndsMowing(HomeReason reason) {
  const bool critical = reason == HomeReason::kBatteryCritical;
  const double voltage = critical ? hw_.battery_voltage : home_.WindowVoltage();
  const double limit = critical ? config_.battery_critical_voltage : config_.battery_empty_voltage;
  AddEvent("BATTERY_LOW", {{"battery_voltage", voltage}, {"limit", limit}});
  notices_.push_back("Mowing ends: the battery is low and the mower cannot go home");
  EndJob(false, false);
}

void LogicCore::WatchHome(double now_s) {
  home_.OnSample(HomeSampleOf(hw_, now_s));
  const std::vector<HomeReason> due = home_.Due();
  if (due.empty()) return;
  const auto battery = [](HomeReason r) { return r == HomeReason::kBatteryCritical || r == HomeReason::kBatteryLow; };
  // A pause that waits for the user never goes home: the user may stand at the mower (G11).
  const bool waits_for_user = (pause_ & (kPauseManual | kPauseEmergency | kPauseFault)) != 0;
  if (!waits_for_user && HomeReachable()) {
    Reason why = HomeWhy(due.front());
    why.message = "Going home: " + why.message;
    EndJob(false, true, why);
    return;
  }
  for (HomeReason reason : due) {
    if (battery(reason)) {
      BatteryEndsMowing(reason);
      return;
    }
  }
  // Without a way home heat and rain change nothing yet (O11, question 47 iii); told once per run.
  const int bit = 1 << static_cast<int>(due.front());
  if (waits_for_user || (home_told_ & bit) != 0) return;
  home_told_ |= bit;
  notices_.push_back("Mowing goes on, it cannot go home: " + HomeWhy(due.front()).message);
}

void LogicCore::BeginDocking() {
  docking_ = NewDocking();
  docking_leg_due_.reset();
  docking_leg_.reset();
  docking_last_kind_ = DockingLeg::Kind::kApproach;
  docking_hold_ = false;
  docking_failure_.reset();
  docking_gps_wait_ = false;
  docking_gps_since_.reset();
  docking_gps_good_since_.reset();
  docking_charge_.reset();
  docking_checked_ = false;
  docking_out_after_charge_off_.reset();
  docking_charge_touched_ = false;
  docking_charge_stopped_ = false;
  SetState(State::kDocking);
  const Point2 p = docking_.ApproachPoint();
  notices_.push_back("Docking: to (" + Metres(p.x) + ", " + Metres(p.y) +
                     ") in front of the docking station, then backwards onto it");
  NextDockingTry();
}

void LogicCore::NextDockingTry() {
  if (!PoseGood()) {
    WaitForDockingGps();
    return;
  }
  // A failed try is counted at once where the mower does not drive out first.
  if (docking_failure_ && !docking_.OnLine(pose_->x, pose_->y)) {
    CountDockingFailure(*docking_failure_);
    return;
  }
  docking_checked_ = false;
  SetDockingLeg(docking_.FirstLeg(pose_->x, pose_->y, pose_->heading));
}

std::optional<DockingLeg> LogicCore::TakeDockingLeg() {
  if (state_ != State::kDocking || !docking_leg_due_) return std::nullopt;
  // A contact that came after the way out was set: charging off first all the same.
  if (docking_leg_due_->kind == DockingLeg::Kind::kOut) {
    DockingLeg out = std::move(*docking_leg_due_);
    docking_leg_due_.reset();
    SetDockingLeg(std::move(out));
    if (!docking_leg_due_) return std::nullopt;
  }
  std::optional<DockingLeg> leg;
  leg.swap(docking_leg_due_);
  docking_leg_ = leg->kind;
  docking_last_kind_ = leg->kind;
  docking_hold_ = false;
  return leg;
}

void LogicCore::OnDockingContact() {
  if (docking_charge_) {
    OnChargeResult(docking_charge_->OnSample(ChargeSampleNow()));
    return;
  }
  // A leg onto the station failed, and before the way out the contact is there after all: the charge decides, once
  // per try. Charging that went off for the way out is allowed again.
  const bool failed_onto =
      docking_last_kind_ == DockingLeg::Kind::kLine || docking_last_kind_ == DockingLeg::Kind::kContact;
  if (DockContact() && docking_failure_ && !docking_leg_ && failed_onto && !docking_checked_) {
    docking_leg_due_.reset();
    docking_out_after_charge_off_.reset();
    docking_failure_.reset();
    if (charge_off_due_) {
      charge_off_due_ = false;
      docking_charge_touched_ = false;
    }
    AllowChargeAgain();
    BeginChargeCheck();
    return;
  }
  // On the way onto the station, or waiting there for a good position; driving out starts on the contacts.
  const bool onto =
      docking_leg_ && (*docking_leg_ == DockingLeg::Kind::kLine || *docking_leg_ == DockingLeg::Kind::kContact);
  if (!onto && !docking_gps_wait_) return;
  if (DockContact()) {
    BeginChargeCheck();
    return;
  }
  if (!onto) return;
  const bool raw = hw_.telemetry_ok && hw_.dock_contact_raw;
  if (raw && !docking_hold_) {
    // G18: one zero command, the driver sends three zero frames and then nothing while the contact settles.
    docking_hold_ = true;
    stop_requested_ = true;
    notices_.push_back("Docking: contact with the docking station, stopped until it is confirmed");
  } else if (!raw && docking_hold_) {
    docking_hold_ = false;
    notices_.push_back("Docking: the contact went away before it was confirmed, on slowly");
  }
}

void LogicCore::OnDockingLegEnded(LegEnd end, const Reason& reason) {
  if (state_ != State::kDocking || !docking_leg_) return;
  const DockingLeg::Kind kind = *docking_leg_;
  docking_leg_.reset();
  docking_hold_ = false;
  if (end == LegEnd::kEmergency) {
    notices_.push_back("Docking stopped: " + reason.message);
    EndDocking(reason);
    return;
  }
  if (kind == DockingLeg::Kind::kOut) AllowChargeAgain();
  if (end == LegEnd::kGps) {
    WaitForDockingGps();
    return;
  }
  if (end != LegEnd::kDone) {
    if (kind == DockingLeg::Kind::kApproach) {
      DockingTryFailed(Reason("approach_failed", "The way to the docking station failed: " + reason.message));
    } else if (kind == DockingLeg::Kind::kOut && docking_failure_) {
      Reason why = *docking_failure_;
      why.message += "; then driving out failed: " + reason.message;
      CountDockingFailure(why);
    } else {
      DockingTryFailed(Reason("dock_failed", std::string("Docking failed on the ") + DockingLegName(kind) + ": " +
                                                 reason.message));
    }
    return;
  }
  switch (kind) {
    case DockingLeg::Kind::kApproach: docking_leg_due_ = docking_.Line(); break;
    case DockingLeg::Kind::kOut:
      if (docking_failure_) {
        CountDockingFailure(*docking_failure_);
      } else {
        NextDockingTry();
      }
      break;
    case DockingLeg::Kind::kLine: {
      const std::string gate = pose_ ? docking_.CheckGate(pose_->x, pose_->y, pose_->heading,
                                                          pose_->position_accuracy, DockAccuracyLimit())
                                     : "at the gate there was no position";
      if (gate.empty()) {
        notices_.push_back("Docking: through the gate, onto the contacts");
        docking_leg_due_ = docking_.Contact();
      } else {
        DockingTryFailed(Reason("dock_failed", "Docking failed: " + gate));
      }
      break;
    }
    case DockingLeg::Kind::kContact:
      DockingTryFailed(Reason("dock_failed", "Docking failed: no contact with the docking station up to " +
                                                 Metres(config_.dock.overshoot) + " m past it"));
      break;
  }
}

void LogicCore::DockingTryFailed(const Reason& why) {
  notices_.push_back(why.message);
  // Out to the approach point first; the try counts once the mower got there.
  if (pose_ && docking_last_kind_ != DockingLeg::Kind::kOut && docking_.OnLine(pose_->x, pose_->y)) {
    docking_failure_ = why;
    SetDockingLeg(docking_.Out(pose_->x, pose_->y, pose_->heading));
    return;
  }
  CountDockingFailure(why);
}

void LogicCore::SetDockingLeg(DockingLeg leg) {
  // Off the contacts only with charging off where the driver can stop it, like undocking (G19).
  const bool on_contacts = hw_.telemetry_ok && (hw_.dock_contact || hw_.dock_contact_raw);
  if (leg.kind == DockingLeg::Kind::kOut && charge_stop_ && on_contacts && !docking_charge_stopped_) {
    docking_out_after_charge_off_ = std::move(leg);
    docking_charge_touched_ = true;
    charge_off_due_ = true;
    notices_.push_back("Docking: charging off, then out of the docking station");
    return;
  }
  docking_leg_due_ = std::move(leg);
}

void LogicCore::DockingChargeOff(bool ok, const std::string& message) {
  DockingLeg out = std::move(*docking_out_after_charge_off_);
  docking_out_after_charge_off_.reset();
  if (ok) {
    docking_charge_stopped_ = true;
    docking_leg_due_ = std::move(out);
    return;
  }
  // Not off the contacts while they may still charge: docking ends where the mower stands.
  Reason why = docking_failure_.value_or(Reason("dock_failed", "Docking failed"));
  why.message += "; charging could not be switched off (" + message + "), the mower stays on the docking station";
  notices_.push_back(why.message);
  CountDockingFailure(why, true);
  charge_watch_.Unconfirm(why.code == "NO_CHARGE_CURRENT");
}

void LogicCore::AllowChargeAgain() {
  if (!docking_charge_touched_) return;
  docking_charge_touched_ = false;
  docking_charge_stopped_ = false;
  charge_on_due_ = true;
}

ChargeSample LogicCore::ChargeSampleNow() const {
  ChargeSample sample;
  sample.time = hw_.time_s;
  sample.contact = DockContact();
  sample.battery_ok = hw_.battery_ok;
  sample.charge_current = hw_.battery_current < 0 ? -hw_.battery_current : 0.0;
  sample.percentage = hw_.battery_percentage;
  return sample;
}

void LogicCore::BeginChargeCheck() {
  // The mower stands since the first raw contact: the leg ends, nothing drives while the charge is checked.
  CancelDockingLeg(Reason("STOPPED", "Contact with the docking station"));
  docking_gps_wait_ = false;
  docking_checked_ = true;
  docking_charge_.emplace(config_.charge, hw_.time_s);
  notices_.push_back("Docking: contact with the docking station, checking the charge");
  OnChargeResult(docking_charge_->OnSample(ChargeSampleNow()));
}

void LogicCore::OnChargeResult(ChargeCheck::Result result) {
  if (result == ChargeCheck::Result::kWaiting) return;
  const ChargeCheck check = *docking_charge_;
  docking_charge_.reset();
  char text[240];
  switch (result) {
    case ChargeCheck::Result::kCharging:
      std::snprintf(text, sizeof(text), "Docking: charging at %.2f A", check.max_current());
      notices_.push_back(text);
      Docked();
      return;
    case ChargeCheck::Result::kFull:
      std::snprintf(text, sizeof(text), "Docking: the contact held, the battery at %d %% needs no charge current",
                    check.percentage());
      notices_.push_back(text);
      Docked();
      return;
    case ChargeCheck::Result::kContactLost:
      DockingTryFailed(
          Reason("dock_failed", "Docking failed: the contact with the docking station went away while the charge "
                                "was checked"));
      return;
    case ChargeCheck::Result::kNoCharge:
    default: {
      std::snprintf(text, sizeof(text),
                    "Docking failed: no charge current on the contacts within %.0f s (at most %.2f A, %.2f A "
                    "needed below %.0f %%, the battery at %d %%)",
                    config_.charge.timeout, check.max_current(), config_.charge.current,
                    config_.charge.expected_below, check.percentage());
      const Reason why =
          Reason("NO_CHARGE_CURRENT", text).With("current", check.max_current()).With("limit", config_.charge.current);
      if (!docking_.LastTry()) {
        DockingTryFailed(why);
        return;
      }
      // The last try leaves the mower on the contacts: a person has to look anyway, and charging may still begin.
      notices_.push_back(why.message);
      CountDockingFailure(why, true);
      charge_watch_.Unconfirm(true);
      return;
    }
  }
}

void LogicCore::CountDockingFailure(Reason why, bool last) {
  docking_failure_.reset();
  const bool again = docking_.Failed() && !last;
  const char* type = again ? "DOCKING_RETRY" : "DOCKING_FAILED";
  json details = json::object();
  PutReason(&details, type, Reason(why).With("attempts", docking_.failures()));
  AddEvent(type, std::move(details));
  if (!again) {
    notices_.push_back("Docking failed, tries: " + std::to_string(docking_.failures()));
    EndDocking(Reason("STOPPED", why.message));
    return;
  }
  notices_.push_back("Docking: try " + std::to_string(docking_.failures() + 1) + " of " +
                     std::to_string(docking_.config().retry_count + 1));
  NextDockingTry();
}

void LogicCore::WaitForDockingGps() {
  docking_leg_due_.reset();
  docking_gps_wait_ = true;
  docking_gps_since_.reset();
  docking_gps_good_since_.reset();
  notices_.push_back("Docking waits for a good position");
}

void LogicCore::TickDocking(double now_s) {
  if (docking_charge_) {
    OnChargeResult(docking_charge_->OnTick(now_s));
    return;
  }
  if (!docking_gps_wait_) return;
  if (!docking_gps_since_) docking_gps_since_ = now_s;
  if (PoseGood()) {
    if (!docking_gps_good_since_) docking_gps_good_since_ = now_s;
    if (now_s - *docking_gps_good_since_ < config_.gps_wait_time) return;
    docking_gps_wait_ = false;
    notices_.push_back("Docking goes on: the position is good again");
    NextDockingTry();
    return;
  }
  docking_gps_good_since_.reset();
  if (now_s - *docking_gps_since_ < config_.gps_timeout) return;
  // Why it waited in vain, with the names of a refused start.
  char text[128];
  std::snprintf(text, sizeof(text), "Docking failed: no good position for %.0f s", config_.gps_timeout);
  Reason why;
  if (!pose_) {
    why = Reason("NO_POSITION", std::string(text) + ": no position from the localization");
  } else if (!pose_->orientation_valid) {
    why = Reason("NO_HEADING", std::string(text) + ": the heading is not known");
  } else {
    why = AccuracyReason(pose_->position_accuracy, config_.max_position_accuracy);
    why.message = std::string(text) + ": " + why.message;
  }
  CountDockingFailure(why, true);
}

void LogicCore::CancelDockingLeg(const Reason& why) {
  docking_leg_due_.reset();
  if (!docking_leg_) return;
  docking_leg_.reset();
  docking_hold_ = false;
  CancelDrive(why);
  stop_requested_ = true;
}

void LogicCore::Docked() {
  charge_watch_.Confirm();
  AddEvent("DOCKED", json::object());
  notices_.push_back("Docked");
  EndDocking(Reason("STOPPED", "Docked"));
}

void LogicCore::EndDocking(const Reason& why) {
  CancelDockingLeg(why);
  docking_failure_.reset();
  docking_gps_wait_ = false;
  docking_charge_.reset();
  docking_out_after_charge_off_.reset();
  AllowChargeAgain();
  SetState(State::kIdle);
}

void LogicCore::WatchCharge() {
  if (state_ != State::kIdle || !hw_.telemetry_ok) {
    charge_watch_.OnBusy(hw_.telemetry_ok && !hw_.dock_contact);
    return;
  }
  const ChargeSample now = ChargeSampleNow();
  const ChargeConfig& c = config_.charge;
  char text[200];
  switch (charge_watch_.OnSample(now, emergency_)) {
    case ChargeWatch::Change::kConfirmed:
      AddEvent("DOCKED", json::object());
      notices_.push_back("Docked: charging confirmed on the docking station");
      return;
    case ChargeWatch::Change::kContactLost:
      ChargingLost(Reason("NO_DOCK_CONTACT", "The mower lost the contact with the docking station"));
      return;
    case ChargeWatch::Change::kCurrentLost:
      std::snprintf(text, sizeof(text),
                    "The charge current on the docking station stopped for %.0f s (below %.2f A, the battery at "
                    "%d %%)",
                    c.lost_time, c.current, now.percentage);
      ChargingLost(Reason("NO_CHARGE_CURRENT", text).With("current", now.charge_current).With("limit", c.current));
      return;
    case ChargeWatch::Change::kNoCurrent:
      std::snprintf(text, sizeof(text),
                    "On the docking station, but no charge current for %.0f s (at least %.2f A, the battery at %d %%)",
                    c.timeout, c.current, now.percentage);
      ChargingLost(Reason("NO_CHARGE_CURRENT", text).With("current", now.charge_current).With("limit", c.current));
      return;
    case ChargeWatch::Change::kNone:
    default: return;
  }
}

void LogicCore::ChargingLost(const Reason& why) {
  json details = json::object();
  PutReason(&details, "CHARGING_LOST", why);
  if (hw_.battery_ok) details["battery_percentage"] = hw_.battery_percentage;
  AddEvent("CHARGING_LOST", std::move(details));
  notices_.push_back("Charging lost: " + why.message);
}

Reason LogicCore::StartProblem() const {
  Reason why = Busy();
  if (why.empty()) why = HoldProblem();
  return why.empty() ? MowProblem() : why;
}

Reason LogicCore::MowProblem() const {
  Reason why = Unsafe();
  if (!why.empty()) return why;
  if (hw_.rain) return Reason("RAIN", "Rain detected");
  char text[160];
  // Below empty mowing would go home at once.
  if (hw_.battery_ok && hw_.battery_voltage < config_.battery_empty_voltage) {
    std::snprintf(text, sizeof(text), "Battery %.1f V is below %.1f V", hw_.battery_voltage,
                  config_.battery_empty_voltage);
    return Reason("BATTERY_LOW", text)
        .With("battery_voltage", hw_.battery_voltage)
        .With("limit", config_.battery_empty_voltage);
  }
  if (hw_.battery_ok && config_.battery_hot_temperature > 0 &&
      hw_.battery_temperature >= config_.battery_hot_temperature) {
    std::snprintf(text, sizeof(text), "The battery at %d °C is too hot to start (%.0f °C or more)",
                  hw_.battery_temperature, config_.battery_hot_temperature);
    return Reason("BATTERY_HOT", text)
        .With("temperature", static_cast<int>(hw_.battery_temperature))
        .With("limit", config_.battery_hot_temperature);
  }
  // Like OpenMower: hot at motor_hot_temperature, on again below motor_cold_temperature.
  if (hw_.blade.ok && hw_.blade.temperature >= config_.motor_cold_temperature) {
    std::snprintf(text, sizeof(text), "The blade motor at %d °C has to cool below %.0f °C first",
                  hw_.blade.temperature, config_.motor_cold_temperature);
    return Reason("MOTOR_HOT", text)
        .With("temperature", static_cast<int>(hw_.blade.temperature))
        .With("limit", config_.motor_cold_temperature);
  }
  why = PositionProblem();
  if (!why.empty()) return why;
  if (!mow_map_problem_.empty()) return mow_map_problem_;
  if (!job_.HasMowableArea()) return Reason("NO_MOWING_AREA", "The map has no active mowing area");
  return DockProblem();
}

void LogicCore::RefuseStart(const Reason& why) {
  json details = json::object();
  PutReason(&details, "MOWING_REFUSED", why);
  AddEvent("MOWING_REFUSED", std::move(details));
}

std::string LogicCore::RefuseReset(const Reason& why) {
  json details = json::object();
  PutReason(&details, "EMERGENCY_RESET_REFUSED", why);
  if (release_pending_) details["source"] = release_source_;
  // The refusal carries the client of the request it answers.
  const std::string client = client_;
  if (release_pending_) client_ = release_client_;
  release_pending_ = false;
  release_answered_s_.reset();
  const std::string id = AddEvent("EMERGENCY_RESET_REFUSED", std::move(details));
  client_ = client;
  return id;
}

void LogicCore::SetRestartHold(bool held) {
  if (held == restart_hold_) return;
  restart_hold_ = held;
  actions_changed_ = true;
  UpdateEmergency(false);
}

void LogicCore::ReportRestart(const RestartReport& report) {
  json details = {{"cause", report.cause},
                  {"detail", report.detail},
                  {"crashed_at", report.crashed_at > 0 ? json(report.crashed_at) : json()},
                  {"state_before", report.state_before},
                  {"job_id_before", report.job_id_before.empty() ? json() : json(report.job_id_before)},
                  {"hold", report.hold}};
  PutReason(&details, "UNEXPECTED_RESTART", Reason(report.cause, report.detail));
  AddEvent("UNEXPECTED_RESTART", std::move(details));
}

const std::string& LogicCore::JobId() const {
  return job_.progress().job_id;
}

void LogicCore::OnReleaseAnswered(double now_s) {
  if (release_pending_) release_answered_s_ = now_s;
}

bool LogicCore::OnStopRelease() {
  if (!emergency_) return false;
  release_pending_ = true;
  release_source_ = "button";
  release_client_.clear();
  release_answered_s_.reset();
  return true;
}

void LogicCore::GeometryChanged(const std::vector<std::string>& changed, const std::vector<std::string>& below_default,
                                int version) {
  const auto list = [](const std::vector<std::string>& names) {
    std::string text;
    for (const auto& name : names) text += (text.empty() ? "" : ", ") + name;
    return text;
  };
  json details = json::object();
  details["changed"] = changed;
  details["below_default"] = below_default;
  details["version"] = version;
  std::string message = "The dimensions of the mower changed: " + list(changed);
  if (below_default.empty()) {
    details["severity"] = "info";
  } else {
    details["reason"] = "BELOW_DEFAULT";
    details["severity"] = "warning";
    message += ". " + list(below_default) + " below the platform's value: check the measurement";
  }
  details["message"] = message;
  AddEvent("GEOMETRY_CHANGED", std::move(details));
}

bool LogicCore::Mowing() const {
  return state_ == State::kMowing;
}

bool LogicCore::Idle() const {
  return state_ == State::kIdle;
}

void LogicCore::SetHold(int64_t until) {
  if (until == hold_until_) return;
  hold_until_ = until;
  actions_changed_ = true;
}

bool LogicCore::Held() const {
  return hold_until_ > 0 || hw_.hold;
}

Reason LogicCore::HoldProblem() const {
  if (hold_until_ > 0) {
    return Reason("HOLD", "An update holds the mower: nothing starts until it is done").With("until", hold_until_);
  }
  if (hw_.hold) return Reason("HOLD", "The driver holds the mower for an update: nothing starts until it is done");
  return {};
}

bool LogicCore::HoldAllowed() const {
  return state_ == State::kIdle && dock_check_ == DockCheck::kNone;
}

void LogicCore::SetGpsSettings(double max_position_accuracy, double gps_wait_time, double gps_timeout) {
  config_.max_position_accuracy = max_position_accuracy;
  config_.gps_wait_time = gps_wait_time;
  config_.gps_timeout = gps_timeout;
  recorder_.SetMaxPositionAccuracy(max_position_accuracy);
}

void LogicCore::StartMowing() {
  // Planned anew on every start, the map may have changed; the fingerprint keeps the place if the plan is the same.
  job_.Restore(job_.progress());
  leg_ = Leg::kNone;
  approached_ = false;
  announced_area_.clear();
  SetState(State::kMowing);
  CheckpointNow();
}

void LogicCore::ApplySkips() {
  // Its events carry the client of the skip, also after the leg stopped.
  const std::string client = client_;
  if (client_.empty()) client_ = skip_client_;
  if (skip_area_) {
    if (const JobArea* area = job_.CurrentArea()) {
      json details = {{"area_id", area->id}, {"area_name", area->name}};
      PutReason(&details, "AREA_SKIPPED", Reason("MANUAL", "Skipped by the app"));
      AddEvent("AREA_SKIPPED", std::move(details));
    }
    job_.SkipArea();
    // A plan on its way is for the skipped area.
    if (leg_ == Leg::kPlanning) leg_ = Leg::kNone;
  } else if (skip_path_ && job_.HasPlan()) {
    job_.SkipPath();
  }
  if (skip_area_ || skip_path_) {
    approached_ = false;
    spinup_since_.reset();
    CheckpointNow();
  }
  skip_area_ = skip_path_ = false;
  skip_client_.clear();
  client_ = client;
}

void LogicCore::Pause(int why, const Reason& reason) {
  if (state_ != State::kMowing) return;
  const bool was_paused = pause_ != 0;
  pause_ |= why;
  if (why == kPauseGps) gps_good_since_.reset();
  if (leg_ == Leg::kApproach || leg_ == Leg::kFollow) {
    CancelDrive(reason);
    stop_requested_ = true;
  }
  spinup_since_.reset();
  if (was_paused) return;
  pause_reason_ = reason;
  notices_.push_back("Mowing paused: " + reason.message);
  json details = {{"state", "PAUSED"}};
  PutReason(&details, "STATE", reason);
  AddEvent("STATE", std::move(details));
  actions_changed_ = true;
  CheckpointNow();
}

void LogicCore::Unpause(int why) {
  if (pause_ == 0) return;
  pause_ &= ~why;
  actions_changed_ = true;
  if (pause_ != 0) return;
  pause_reason_ = Reason();
  // From where it stopped: the start of the path may be behind it now.
  approached_ = false;
  AddEvent("STATE", {{"state", "MOWING"}});
}

void LogicCore::EndJob(bool finished, bool home, const Reason& why) {
  if (state_ != State::kMowing) return;
  if (finished) {
    AddEvent("JOB_COMPLETE", json::object());
    job_.Complete();
  }
  if (home) {
    GoHome(why);
  } else {
    SetState(State::kIdle);
  }
  CheckpointNow();
}

JobStep LogicCore::NextJobStep(double now_s) {
  JobStep step;
  if (state_ != State::kMowing || pause_ != 0 || emergency_ || leg_ != Leg::kNone) return step;
  if (!job_.HasPlan()) {
    std::string notice;
    if (!job_.SeekArea(&notice)) {
      if (!notice.empty()) notices_.push_back(notice);
      EndJob(true, true);
      return step;
    }
    if (!notice.empty()) notices_.push_back(notice);
    const JobArea* area = job_.CurrentArea();
    if (area->id != announced_area_) {
      announced_area_ = area->id;
      AddEvent("AREA", {{"area_id", area->id}, {"area_name", area->name}});
    }
    leg_ = Leg::kPlanning;
    step.kind = JobStep::Kind::kPlan;
    step.area_id = area->id;
    return step;
  }
  const JobPose* goal = job_.CurrentPose();
  if (!goal) return step;
  if (!pose_) {
    Pause(kPauseGps, Reason("NO_POSITION", "No position from the localization"));
    return step;
  }
  if (!approached_ && std::hypot(goal->x - pose_->x, goal->y - pose_->y) > config_.approach_distance) {
    leg_ = Leg::kApproach;
    step.kind = JobStep::Kind::kApproach;
    step.goal = *goal;
    return step;
  }
  // The blade gets up to speed before a path that starts mowing, like in OpenMower.
  if (config_.mow_with_blade && IsMowKind(goal->kind) && config_.mower_spinup_rpm > 0 &&
      std::abs(hw_.blade.rpm) < config_.mower_spinup_rpm) {
    if (!spinup_since_) spinup_since_ = now_s;
    if (now_s - *spinup_since_ > config_.mower_spinup_timeout) {
      AddEvent("MOW_MOTOR_SPINUP_FAILED", json::object());
      Pause(kPauseFault, Reason("BLADE_SPINUP_FAILED", "The blade did not reach its speed"));
    }
    return step;
  }
  spinup_since_.reset();
  leg_ = Leg::kFollow;
  step.kind = JobStep::Kind::kFollow;
  step.path = *job_.CurrentPath();
  step.start_index = static_cast<size_t>(job_.progress().index);
  return step;
}

void LogicCore::OnJobPlanned(std::vector<JobPath> paths, const std::string& fingerprint) {
  if (state_ != State::kMowing || leg_ != Leg::kPlanning) return;
  leg_ = Leg::kNone;
  job_.SetPlan(std::move(paths), fingerprint);
  approached_ = false;
  CheckpointNow();
}

void LogicCore::OnJobPlanFailed(const std::string& message) {
  if (state_ != State::kMowing || leg_ != Leg::kPlanning) return;
  leg_ = Leg::kNone;
  Pause(kPauseFault, Reason("PLANNING_FAILED", "Planning failed: " + message));
}

void LogicCore::OnJobProgress(size_t index) {
  if (state_ != State::kMowing || leg_ != Leg::kFollow) return;
  job_.OnIndex(index);
}

void LogicCore::OnLegEnded(LegEnd end, size_t index, const Reason& reason) {
  if (state_ != State::kMowing || (leg_ != Leg::kApproach && leg_ != Leg::kFollow)) return;
  const bool follow = leg_ == Leg::kFollow;
  leg_ = Leg::kNone;
  if (follow) job_.OnIndex(index);
  if (skip_area_ || skip_path_) {
    ApplySkips();
    return;
  }
  switch (end) {
    case LegEnd::kDone:
      if (follow) {
        job_.PathDone();
        approached_ = false;
        CheckpointNow();
      } else {
        approached_ = true;
      }
      break;
    case LegEnd::kCanceled: approached_ = false; break;
    case LegEnd::kGps:
      // The follower's text carries no numbers: the event gets the accuracy and the limit like a refused start.
      if (reason.code == "POSITION_ACCURACY" && pose_ && !(pose_->position_accuracy <= config_.max_position_accuracy)) {
        Pause(kPauseGps, AccuracyReason(pose_->position_accuracy, config_.max_position_accuracy));
      } else {
        Pause(kPauseGps, reason);
      }
      break;
    case LegEnd::kEmergency: Pause(kPauseEmergency, reason); break;
    case LegEnd::kFailed:
      approached_ = false;
      Pause(kPauseFault, reason);
      break;
  }
  if (follow) CheckpointNow();
}

void LogicCore::OnTick(double now_s) {
  // Any end of the emergency drops the request, so one still pending is an emergency that stayed.
  if (release_answered_s_ && now_s - *release_answered_s_ > kReleaseSourceS) {
    RefuseReset(Reason("STILL_ACTIVE", "The emergency stays: " + EmergencyReasonNames(reason_))
                    .With("bits", mowbite_api::EmergencyNameList(reason_)));
  }
  if (state_ == State::kDocking) TickDocking(now_s);
  if (state_ != State::kMowing) return;
  if (pause_ & kPauseGps) {
    if (!PoseGood()) {
      gps_good_since_.reset();
    } else if (!gps_good_since_) {
      gps_good_since_ = now_s;
    } else if (now_s - *gps_good_since_ >= config_.gps_wait_time) {
      gps_good_since_.reset();
      Unpause(kPauseGps);
    }
  }
  WatchHome(now_s);
  if (state_ != State::kMowing) return;
  if (leg_ == Leg::kFollow && now_s - last_checkpoint_s_ >= config_.checkpoint_period) {
    last_checkpoint_s_ = now_s;
    CheckpointNow();
  }
}

bool LogicCore::BladeWanted() const {
  if (state_ != State::kMowing || pause_ != 0 || emergency_ || !config_.mow_with_blade || !safety_measured_ ||
      !hw_.telemetry_ok || hw_.dock_contact) {
    return false;
  }
  if (spinup_since_) return true;
  const JobPose* pose = job_.CurrentPose();
  return leg_ == Leg::kFollow && pose && IsMowKind(pose->kind);
}

void LogicCore::OnBladeRefused(const std::string& message, int emergency_reason) {
  if (!BladeWanted()) return;
  // The driver blocked before hw/status showed it: the pause of that emergency, its flag and event follow hw/status.
  if (emergency_reason != 0) {
    Pause(kPauseEmergency, EmergencyWhy(emergency_reason));
    return;
  }
  Pause(kPauseFault, Reason("BLADE_REFUSED", "The blade was refused: " + message));
}

void LogicCore::CheckpointNow() {
  checkpoint_due_ = true;
}

std::optional<std::string> LogicCore::TakeCheckpoint() {
  if (!checkpoint_due_) return std::nullopt;
  checkpoint_due_ = false;
  return CheckpointText(job_.progress());
}

}  // namespace mowbite_logic
