#include "mowbite_logic/logic_core.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iterator>

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

bool PlannerSilent(const Reason& why) {
  return why.code == "PLANNER_TIMEOUT" || why.code == "PLANNER_MISSING";
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
  effects_.push_back(ActionsChanged{});
}

void LogicCore::Notify(std::string line) {
  effects_.push_back(Notice{std::move(line)});
}

namespace {

template <typename T, typename... Types>
constexpr bool kOneOf = (std::is_same_v<T, Types> || ...);

}  // namespace

const char* EffectName(const Effect& effect) {
  static constexpr const char* kNames[] = {"DriveCancel",     "StopRequest",    "CalibrationCancel", "ChargeOff",
                                           "ChargeOn",        "ReferenceCheck", "UndockPath",        "UndockOutcome",
                                           "CalibrationGoal", "MapEdit",        "Event",             "ReasonChange",
                                           "ActionsChanged",  "Notice",         "Checkpoint",        "RunChanged"};
  static_assert(std::size(kNames) == std::variant_size_v<Effect>, "a name for every effect");
  return kNames[effect.index()];
}

EffectPhase PhaseOf(const Effect& effect) {
  return std::visit(
      [](const auto& value) {
        using T = std::decay_t<decltype(value)>;
        if constexpr (kOneOf<T, DriveCancel, StopRequest, CalibrationCancel>) {
          return EffectPhase::kHalt;
        } else if constexpr (kOneOf<T, Event, ReasonChange, ActionsChanged, Notice, Checkpoint, RunChanged>) {
          return EffectPhase::kRecord;
        } else {
          static_assert(
              kOneOf<T, ChargeOff, ChargeOn, ReferenceCheck, UndockPath, UndockOutcome, CalibrationGoal, MapEdit>,
              "every effect has its phase");
          return EffectPhase::kAct;
        }
      },
      effect);
}

void LogicCore::OnHardwareStatus(const HardwareSnapshot& hw) {
  if (hw.hold != hw_.hold) SetDue<ActionsChanged>(true);
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
  WatchAnnouncement();
}

void LogicCore::OnHardwareTimeout() {
  hw_.telemetry_ok = false;
  hw_.battery_ok = false;
  recorder_.SetDockContact(false);
  charge_watch_.OnBusy(false);
  UpdateEmergency();
  WatchAnnouncement();
}

void LogicCore::OnPose(const PoseSnapshot& pose) {
  pose_ = pose;
  last_x_ = pose.x;
  last_y_ = pose.y;
  if (state_ == State::kDocking) WatchMarkerNear();
  if (state_ != State::kAreaRecording) return;
  recorder_.OnPose(pose.x, pose.y, pose.heading, pose.position_accuracy, pose.orientation_valid);
  PullRecorder();
}

void LogicCore::OnPoseTimeout() {
  pose_.reset();
  if (state_ == State::kDriving) CancelDrive(Reason("POSE_LOST", "No position from the localization"));
  if (state_ == State::kMowing) Pause(kPauseGps, Reason("NO_POSITION", "No position from the localization"));
  if (state_ == State::kUndocking) {
    StopUndocking(Reason("no_gps", "Undocking stopped: no position from the localization"),
                  Reason("POSE_LOST", "No position from the localization"));
  }
  const bool map_leg = (docking_leg_ && docking_leg_frame_ == LegFrame::kMap) ||
                       (docking_leg_due_ && docking_leg_due_->frame == LegFrame::kMap);
  if (state_ == State::kDocking && map_leg) {
    CancelDockingLeg(Reason("POSE_LOST", "No position from the localization"));
    WaitForDockingGps();
  }
  if (state_ == State::kCalibrating) {
    calibration_.EndCalibration(CoreAccess(*this), Reason("POSE_LOST", "No position from the localization"), true);
  }
  recorder_.OnPoseLost();
  PullRecorder();
}

std::vector<Effect> LogicCore::TakeEffects() {
  if (auto edit = recorder_.TakeMapEdit()) effects_.push_back(std::move(*edit));
  for (auto& line : recorder_.TakeNotices()) Notify(std::move(line));
  // start_mowing and its reason follow the position, the battery and more, which change without an event.
  const std::string start_reason = StartProblem().code;
  if (start_reason != last_start_reason_) {
    last_start_reason_ = start_reason;
    SetDue<ActionsChanged>(true);
  }
  for (auto& effect : effects_) {
    if (auto* checkpoint = std::get_if<Checkpoint>(&effect)) checkpoint->text = CheckpointText(job_.progress());
  }
  return effects_.Take();
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
  if (state_ == State::kAreaRecording) {
    recorder_.OnPositionJump();
    PullRecorder();
  }
}

// Every event carries the position where it happened, and the client of the command that caused it.
std::string LogicCore::AddEvent(const std::string& type, json details) {
  details["x"] = last_x_;
  details["y"] = last_y_;
  if (!job_.progress().job_id.empty()) details["job_id"] = job_.progress().job_id;
  if (!session_id_.empty()) details["session_id"] = session_id_;
  if (!client_.empty() && !details.contains("client")) details["client"] = client_;
  // Who released or asked names the source there.
  if (run_ && !details.contains("source") && type != "EMERGENCY" && type != "EMERGENCY_RESET_REFUSED") {
    details["source"] = "schedule";
  }
  if (event_observer) event_observer(type, details);
  std::string id = NewId();
  effects_.push_back(Event{type, std::move(details), id});
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
    effects_.push_back(ReasonChange{logged_reason_, reason_});
    logged_reason_ = reason_;
  }
  // The first report after the start only sets the state, it is no change worth an event.
  if (!hw_seen_) {
    hw_seen_ = report;
    if (emergency != emergency_) SetDue<ActionsChanged>(true);
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
    if (emergency && state_ == State::kCalibrating) calibration_.EndCalibration(CoreAccess(*this), why, true);
    // reason stays the bits of OpenMower; resetting is up to the user.
    json details = {{"emergency", emergency},
                    {"reason", reason_},
                    {"severity", emergency ? "error" : "info"},
                    {"resume", emergency ? "user" : "auto"}};
    release_.OnEmergencyChanged(emergency, &details);
    AddEvent("EMERGENCY", std::move(details));
    SetDue<ActionsChanged>(true);
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
  if (known_prefix) CommandCancels();
  return known_prefix ? ActionResult::kRejected : ActionResult::kUnknown;
}

CommandOutcome LogicCore::Command(const std::string& name, const json& params, const Reason& node_problem) {
  CommandCancels();
  CommandOutcome out;
  const mowbite_api::Command* command = mowbite_api::FindCommand(name);
  if (!command) {
    out.invalid = "No command " + name;
    return out;
  }
  out.invalid = mowbite_api::CheckParams(*command, params, &out.invalid_field);
  StartRequest start;
  if (out.invalid.empty() && name == "mowing.start") {
    out.invalid = ReadStartRequest(params, &start, &out.invalid_field);
  }
  if (out.invalid.empty() && name == "docking.test" && params.contains("runs")) {
    const json& runs = params["runs"];
    if (!runs.is_number_integer() || runs.get<int64_t>() < 1 || runs.get<int64_t>() > 10) {
      out.invalid = "runs must be a whole number from 1 to 10";
      out.invalid_field = "runs";
    }
  }
  if (!out.invalid.empty()) return out;
  client_ = params.is_object() ? params.value("client", "") : "";
  // Effects may go away while the command runs, events never: its events are those after the ones before it.
  const auto is_event = [](const Effect& effect) { return std::holds_alternative<Event>(effect); };
  const auto events_before = std::count_if(effects_.begin(), effects_.end(), is_event);
  const Reason why = name == "mowing.start" ? RequestStart(std::move(start), node_problem, &out)
                                            : RunCommand(*command, params, node_problem, &out);
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
  auto seen = decltype(events_before){0};
  for (const Effect& effect : effects_) {
    const Event* found = std::get_if<Event>(&effect);
    if (!found || seen++ < events_before) continue;
    const Event& event = *found;
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
    release_.OnResetRequested(client_);
    out->release = true;
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
    SetDue<ActionsChanged>(true);
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
      Reason why = Held() ? HoldProblem() : ContinueProblem();
      if (!why.empty()) {
        RefuseStart(why);
        return why;
      }
      // Without GPS it still waits for a good position.
      Unpause(waiting);
      ResumedByUser();
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
    if (area) skip_why_ = Reason("MANUAL", "Skipped by the app");
    skip_client_ = client_;
    // A leg on its way ends first, its index counts.
    if (LegDrives()) {
      CancelDrive(Reason("STOPPED", "Skipped by the app"));
      SetDue<StopRequest>(true);
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
    Notify("Docking stopped: aborted by the app");
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
  if (name == "docking.test") return StartDockingTest(node_problem, params.is_object() ? params.value("runs", 1) : 1);
  if (name == "recording.start") {
    if (state_ != State::kIdle) return WrongState();
    // The EMERGENCY event told already: no RECORDING_REFUSED.
    if (emergency_) return Reason("EMERGENCY", "Emergency: reset it first");
    Reason why = HoldProblem();
    if (why.empty() && !map_writable_) why = map_reason_;
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
  return RecordingCommand(command, params, out);
}

// A step of the recording: the recorder decides by the action the command stands for.
Reason LogicCore::RecordingCommand(const mowbite_api::Command& command, const json& params, CommandOutcome* out) {
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
  const AreaRecorder::Result result = recorder_.OnAction(action.substr(kRecordingPrefix.size() + 1), &why,
                                                        params.value("meters", kUndoDefaultM));
  PullRecorder();
  if (result != AreaRecorder::Result::kDone) return why.empty() ? WrongState() : why;
  if (command.name == "recording.undo") out->result["points"] = recorder_.current().size();
  if (command.name == "recording.ring_stop" || command.name == "recording.save") {
    out->result["trimmed_points"] = recorder_.trimmed_points();
  }
  return {};
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
  const Reason why("NO_SAFETY_MEASUREMENT", "No safety measurement of this platform (hw/capabilities)");
  if (state_ == State::kDriving) CancelDrive(why);
  if (state_ == State::kUndocking) StopUndocking(why, why);
  if (state_ == State::kCalibrating) calibration_.EndCalibration(CoreAccess(*this), why, true);
  if (state_ == State::kDocking) {
    const bool approach = docking_last_kind_ == DockingLeg::Kind::kApproach;
    CountDockingFailure(Reason(approach ? "approach_failed" : "dock_failed",
                               "Docking stopped: no safety measurement of this platform (hw/capabilities)"),
                        true);
  }
  Pause(kPauseFault, why);
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
  CommandCancels();
  Reason why = DriveProblem();
  if (!why.empty()) {
    RefuseDrive(why);
    return why;
  }
  BeginDrive(x, y);
  return {};
}

void LogicCore::BeginDrive(double x, double y) {
  Drop<DriveCancel>();
  SetState(State::kDriving);
  AddEvent("DRIVE", {{"phase", "start"}, {"goal_x", x}, {"goal_y", y}});
}

void LogicCore::RefuseDrive(const Reason& why) {
  CommandCancels();
  json details = json::object();
  PutReason(&details, "DRIVE_REFUSED", why);
  AddEvent("DRIVE_REFUSED", std::move(details));
}

void LogicCore::OnDriveEnded(bool arrived, const Reason& reason) {
  if (state_ != State::kDriving) return;
  json details = {{"phase", "end"}, {"arrived", arrived}};
  PutReason(&details, "DRIVE", reason);
  AddEvent("DRIVE", std::move(details));
  Drop<DriveCancel>();
  SetState(State::kIdle);
}

bool LogicCore::Driving() const {
  return state_ == State::kDriving;
}

bool LogicCore::FollowerActive() const {
  if (emergency_) return false;
  if (state_ == State::kDriving) return true;
  if (state_ == State::kUndocking) return undock_step_ == UndockStep::kDrive && !undock_stopping_;
  if (state_ == State::kDocking) return docking_leg_ && docking_leg_frame_ == LegFrame::kMap && !docking_hold_;
  return state_ == State::kMowing && pause_ == 0 && LegDrives();
}

void LogicCore::CancelDrive(const Reason& reason) {
  // The first reason stays until the node takes it.
  if (!Pending<DriveCancel>()) effects_.push_back(DriveCancel{reason});
}

void LogicCore::SetState(State state) {
  if (state == state_) return;
  if (state_ == State::kAreaRecording || state_ == State::kDriving || state_ == State::kMowing ||
      state_ == State::kUndocking || state_ == State::kDocking || state_ == State::kCalibrating ||
      state_ == State::kSetup) {
    SetDue<StopRequest>(true);
  }
  // A session is one run of mowing; the job lasts until it is finished or reset.
  if (state == State::kMowing) {
    if (job_.progress().job_id.empty()) job_.SetJobId(NewId());
    session_id_ = NewId();
  }
  if (state_ == State::kMowing) {
    session_id_.clear();
    if (LegDrives()) CancelDrive(Reason("STOPPED", "Mowing ended"));
    leg_ = Leg::kNone;
    pause_ = 0;
    pause_reason_ = Reason();
    skip_path_ = skip_area_ = drive_on_ = false;
    skipped_.reset();
    spinup_since_.reset();
    gps_good_since_.reset();
  }
  if (state_ == State::kMowing || state == State::kMowing) {
    home_.Reset();
    sunset_due_.reset();
    sunset_try_ = sunset_tried_ = sunset_told_ = false;
  }
  // Entering and leaving both start from nothing; what was not saved is gone, like with OpenMower.
  recorder_.Reset();
  recorder_.TakeActionsChanged();
  state_ = state;
  teleop_armed_ = false;
  json details = {{"state", CurrentStateName()}};
  if (state == State::kDriving) details["sub_state"] = "DRIVING";
  if (state == State::kCalibrating) details["sub_state"] = "CALIBRATING";
  if (state == State::kSetup) details["sub_state"] = "SETUP";
  AddEvent("STATE", std::move(details));
  SetDue<ActionsChanged>(true);
  // The run of the schedule ends once the mower is idle again after it left the station.
  if (state == State::kIdle && run_ && run_->phase >= RunPhase::kUndocking) FinishRun();
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
    case State::kSetup:
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
    s.task_index = job_.TaskIndex();
    s.task_count = job_.TaskCount();
  }
  if (state_ == State::kAreaRecording) s.sub_state = recorder_.SubState();
  if (state_ == State::kDriving) s.sub_state = "DRIVING";
  if (state_ == State::kCalibrating) s.sub_state = "CALIBRATING";
  if (state_ == State::kSetup) s.sub_state = "SETUP";
  s.emergency = emergency_;
  s.emergency_reason = reason_;
  s.is_charging = hw_.battery_ok && hw_.dock_contact;
  s.rain_detected = hw_.telemetry_ok && hw_.rain;
  if (hw_.battery_ok) {
    s.battery_percentage = std::clamp(static_cast<float>(hw_.battery_percentage) / 100.0f, 0.0f, 1.0f);
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
  groups.push_back({kRootPrefix, {{"reset_emergency", "Unlatch emergency", true}}});
  const Reason start = StartProblem();
  groups.push_back({kIdlePrefix,
                    {{"start_mowing", "Begin mowing", start.empty(), start.code},
                     {"start_area_recording", "Record an area",
                      idle && !emergency_ && map_writable_ && HoldProblem().empty()},
                     {"reset_job", "Discard unfinished job", idle && job_.Unfinished()}}});
  // Pause and continue take turns, like in OpenMower; continue waits for the emergency to be reset.
  const bool waits = (pause_ & (kPauseManual | kPauseEmergency | kPauseFault)) != 0;
  groups.push_back({kMowingPrefix,
                    {{"pause", "Pause", mowing && !waits},
                     {"continue", "Resume", mowing && waits && !emergency_ && !Held()},
                     {"abort_mowing", "End mowing", mowing},
                     {"skip_area", "Next area", mowing},
                     {"skip_path", "Next path", mowing}}});
  groups.push_back({kUndockingPrefix, {{"abort_undocking", "Cancel undocking", state_ == State::kUndocking}}});
  groups.push_back({kDockingPrefix, {{"abort_docking", "Cancel docking", state_ == State::kDocking}}});
  // The recorder decides while recording; outside of it all are off.
  const std::vector<bool> enabled = recorder_.Enabled();
  const std::vector<std::string>& names = AreaRecorder::ActionLabels();
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

void LogicCore::OnMowMap(std::vector<JobArea> areas, const Reason& problem) {
  job_.SetAreas(std::move(areas));
  mow_map_problem_ = problem;
  if (state_ == State::kMowing && !problem.empty()) Pause(kPauseFault, problem);
}

void LogicCore::RestoreJob(const JobProgress& progress) {
  job_.Restore(progress);
  SetDue<ActionsChanged>(true);
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
                            : state_ == State::kSetup       ? "setup"
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

Reason LogicCore::StartProblem(const std::vector<std::string>& tasks) const {
  Reason why = Busy();
  if (why.empty()) why = HoldProblem();
  return why.empty() ? MowProblem(tasks) : why;
}

Reason LogicCore::MowProblem(const std::vector<std::string>& tasks) const {
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
  if (!job_.HasMowableArea(tasks)) {
    return Reason("NO_MOWING_AREA", tasks.empty() ? "The map has no active mowing area" : "No area of tasks is active");
  }
  return DockProblem();
}

void LogicCore::RefuseStart(const Reason& why) {
  json details = json::object();
  PutReason(&details, "MOWING_REFUSED", why);
  AddEvent("MOWING_REFUSED", std::move(details));
}

const std::string& LogicCore::JobId() const {
  return job_.progress().job_id;
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
  if (!shape_outside_.empty()) {
    std::vector<std::string> parts;
    for (const auto& part : shape_outside_) parts.push_back(part.value("part", ""));
    details["outside_shape"] = shape_outside_;
    if (below_default.empty()) details["reason"] = "SHAPE_OUTSIDE";
    details["severity"] = "warning";
    message += ". The body leaves out " + list(parts) + " of the shape of the mower: check the outline";
  }
  details["message"] = message;
  AddEvent("GEOMETRY_CHANGED", std::move(details));
}

void LogicCore::SetShapeOutside(const std::string& parts) {
  const json parsed = json::parse(parts, nullptr, false);
  shape_outside_ = parsed.is_array() ? parsed : json::array();
}

bool LogicCore::Idle() const {
  return state_ == State::kIdle;
}

void LogicCore::SetHold(int64_t until) {
  if (until == hold_until_) return;
  hold_until_ = until;
  SetDue<ActionsChanged>(true);
  WatchAnnouncement();
}

bool LogicCore::Held() const {
  return hold_until_ > 0 || hw_.hold;
}

Reason LogicCore::HoldProblem() const {
  if (hold_until_ > 0) {
    return Reason("HOLD", "An update holds the mower: nothing starts until it is done").With("until", hold_until_);
  }
  if (hw_.hold) return Reason("HOLD", "The driver holds the mower for an update: nothing starts until it is done");
  return StorageProblem();
}

bool LogicCore::HoldAllowed() const {
  return state_ == State::kIdle && dock_check_ == DockCheck::kNone && !run_;
}

void LogicCore::SetGpsSettings(double max_position_accuracy, double gps_wait_time, double gps_timeout) {
  config_.max_position_accuracy = max_position_accuracy;
  config_.gps_wait_time = gps_wait_time;
  config_.gps_timeout = gps_timeout;
  recorder_.SetMaxPositionAccuracy(max_position_accuracy);
}

void LogicCore::OnTick(double now_s) {
  // Any end of the emergency drops the request, so one still pending is an emergency that stayed.
  if (release_.release_answered_s() && now_s - *release_.release_answered_s() > kReleaseSourceS) {
    RefuseReset(Reason("STILL_ACTIVE", "The emergency stays: " + EmergencyReasonNames(reason_))
                    .With("bits", mowbite_api::EmergencyNameList(reason_)));
  }
  WatchAnnouncement();
  if (run_ && run_->phase == RunPhase::kAnnouncing && now_s >= run_->announce_until) EndAnnouncement(now_s);
  if (state_ == State::kDocking) TickDocking(now_s);
  if (state_ == State::kAreaRecording) TickRecording(now_s);
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
  if (state_ == State::kSetup) {
    return setup_.setup_blade() != 0 && !setup_.setup_blade_refused() && !emergency_ && hw_.telemetry_ok &&
           !hw_.dock_contact;
  }
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
  if (state_ == State::kSetup) {
    setup_.BladeRefused(CoreAccess(*this), message);
    return;
  }
  // The driver blocked before hw/status showed it: the pause of that emergency, its flag and event follow hw/status.
  if (emergency_reason != 0) {
    Pause(kPauseEmergency, EmergencyWhy(emergency_reason));
    return;
  }
  Pause(kPauseFault, Reason("BLADE_REFUSED", "The blade was refused: " + message));
}

void LogicCore::CheckpointNow() {
  SetDue<Checkpoint>(true);
}

}  // namespace mowbite_logic
