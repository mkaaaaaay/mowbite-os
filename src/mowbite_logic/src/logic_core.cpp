#include "mowbite_logic/logic_core.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <iterator>
#include <utility>

#include "mowbite_api/catalog.h"

namespace mowbite_logic {

namespace {

const std::string kRootPrefix = "mower_logic";
const std::string kIdlePrefix = "mower_logic:idle";
const std::string kRecordingPrefix = "mower_logic:area_recording";
const std::string kMowingPrefix = "mower_logic:mowing";
const std::string kUndockingPrefix = "mower_logic:undocking";
const std::string kDockingPrefix = "mower_logic:docking";

EventObserver event_observer = nullptr;

}  // namespace

bool PlannerSilent(const Reason& why) {
  return why.code == "PLANNER_TIMEOUT" || why.code == "PLANNER_MISSING";
}

bool PlannerSkipsArea(const std::string& code) {
  return code == "BODY_OUT" || code == "NO_PLAN" || code == "START_UNREACHABLE";
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

std::string ConfigRangeProblem(const LogicConfig& config) {
  const ObstacleConfig& o = config.obstacle;
  const auto finite = [](std::initializer_list<double> values) {
    return std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); });
  };
  if (!finite({o.free_wait, o.near_wait, o.near_ahead, o.stale, o.avoid_wait, o.avoid_still, o.avoid_sigma,
               o.avoid_clearance, o.avoid_pass_clearance, o.avoid_keep, o.avoid_speed})) {
    return "the obstacle_* values need to be numbers";
  }
  if (!(o.free_wait > 0) || !(o.near_wait >= o.free_wait) || !(o.near_ahead >= 0) || !(o.stale > 0)) {
    return "obstacle_free_wait and obstacle_stale need to be above 0, obstacle_near_wait at least obstacle_free_wait, "
           "obstacle_near_ahead 0 or more";
  }
  if (!(o.avoid_clearance >= 0.30) || !(o.avoid_sigma >= 2.0) || !(o.avoid_pass_clearance >= 0.10) ||
      !(o.avoid_pass_clearance <= o.avoid_clearance) || !(o.avoid_speed > 0 && o.avoid_speed <= 0.3) ||
      !(o.avoid_wait >= 0) || !(o.avoid_still > 0) || !(o.avoid_keep > 0)) {
    return "obstacle_avoid_clearance needs to be at least 0.30, obstacle_avoid_sigma at least 2, "
           "obstacle_avoid_pass_clearance at least 0.10 and at most obstacle_avoid_clearance, obstacle_avoid_speed "
           "above 0 and at most 0.3, obstacle_avoid_still and obstacle_avoid_keep above 0, obstacle_avoid_wait 0 or more";
  }
  const UndockConfig& u = config.undock;
  const DockConfig& d = config.dock;
  if (!finite({u.speed, u.stall_time, d.speed, d.contact_speed, d.contact_timeout, config.gps_wait_time}) ||
      !(u.speed > 0) || !(u.stall_time > 0) || !(d.speed > 0) || !(d.contact_speed > 0) ||
      !(d.contact_timeout > 0) || !(config.gps_wait_time >= 0)) {
    return "undock_speed, undock_stall_time, docking_speed, contact_speed and contact_timeout need to be above 0, "
           "gps_wait_time 0 or more";
  }
  const CalibrationConfig& c = config.calibration;
  if (!finite({c.status_timeout, c.timeout, c.max_position_accuracy}) || !(c.status_timeout > 0) ||
      !(c.timeout > 0) || !(c.max_position_accuracy > 0)) {
    return "calibration_status_timeout, calibration_timeout and calibration_max_position_accuracy need to be above 0";
  }
  return "";
}

LogicCore::LogicCore(const LogicConfig& config)
    : config_(config),
      ids_(config.id_seed),
      recording_(RecorderConfigOf(config)),
      docking_part_(config.charge, config.marker),
      home_(HomeRules()),
      obstacle_part_(config.obstacle) {
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
                                           "ActionsChanged",  "Notice",         "Checkpoint",        "RunChanged",
                                           "ObstacleNote"};
  static_assert(std::size(kNames) == std::variant_size_v<Effect>, "a name for every effect");
  return kNames[effect.index()];
}

EffectPhase PhaseOf(const Effect& effect) {
  return std::visit(
      [](const auto& value) {
        using T = std::decay_t<decltype(value)>;
        if constexpr (kOneOf<T, DriveCancel, StopRequest, CalibrationCancel>) {
          return EffectPhase::kHalt;
        } else if constexpr (kOneOf<T, Event, ReasonChange, ActionsChanged, Notice, Checkpoint, RunChanged,
                                     ObstacleNote>) {
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
  recording_.recorder().SetDockContact(DockContact());
  UpdateEmergency();
  if (state_ == State::kDocking) OnDockingContact();
  // Mowing never touches the station: a contact stops it for the user, the blade stays off until continue.
  if (state_ == State::kMowing && DockContact()) {
    Pause(kPauseFault, Reason("DOCK_CONTACT", "Contact with the docking station while mowing"));
  }
  // The blade of a setup step stays off after the station as after an emergency, until the step asks anew.
  if (state_ == State::kSetup && hw_.dock_contact) setup_.BladeStopped();
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
  recording_.recorder().SetDockContact(false);
  docking_part_.charge_watch().OnBusy(false);
  UpdateEmergency();
  WatchAnnouncement();
}

void LogicCore::OnPose(const PoseSnapshot& pose) {
  pose_ = pose;
  last_x_ = pose.x;
  last_y_ = pose.y;
  if (state_ == State::kDocking) WatchMarkerNear();
  if (state_ != State::kAreaRecording) return;
  recording_.recorder().OnPose(pose.x, pose.y, pose.heading, pose.position_accuracy, pose.orientation_valid);
  recording_.PullRecorder(CoreAccess(*this));
}

void LogicCore::OnPoseTimeout() {
  pose_.reset();
  if (state_ == State::kDriving) CancelDrive(Reason("POSE_LOST", "No position from the localization"));
  if (state_ == State::kMowing) Pause(kPauseGps, Reason("NO_POSITION", "No position from the localization"));
  if (state_ == State::kUndocking) {
    StopUndocking(Reason("no_gps", "Undocking stopped: no position from the localization"),
                  Reason("POSE_LOST", "No position from the localization"));
  }
  const bool map_leg = docking_part_.MapLeg();
  if (state_ == State::kDocking && map_leg) {
    CancelDockingLeg(Reason("POSE_LOST", "No position from the localization"));
    WaitForDockingGps();
  }
  if (state_ == State::kCalibrating) {
    calibration_.EndCalibration(CoreAccess(*this), Reason("POSE_LOST", "No position from the localization"), true);
  }
  recording_.recorder().OnPoseLost();
  recording_.PullRecorder(CoreAccess(*this));
}

std::vector<Effect> LogicCore::TakeEffects() {
  if (auto edit = recording_.recorder().TakeMapEdit()) effects_.push_back(std::move(*edit));
  for (auto& line : recording_.recorder().TakeNotices()) Notify(std::move(line));
  // start_mowing and its reason follow the position, the battery and more, which change without an event.
  const std::string start_reason = StartProblem().code;
  if (start_reason != last_start_reason_) {
    last_start_reason_ = start_reason;
    SetDue<ActionsChanged>(true);
  }
  for (auto& effect : effects_) {
    if (auto* checkpoint = std::get_if<Checkpoint>(&effect)) {
      checkpoint->text = CheckpointText(mowing_.job().progress());
    }
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
    recording_.recorder().OnPositionJump();
    recording_.PullRecorder(CoreAccess(*this));
  }
}

// Every event carries the position where it happened, and the client of the command that caused it.
std::string LogicCore::AddEvent(const std::string& type, json details) {
  details["x"] = last_x_;
  details["y"] = last_y_;
  if (!mowing_.job().progress().job_id.empty()) details["job_id"] = mowing_.job().progress().job_id;
  if (!mowing_.session_id().empty()) details["session_id"] = mowing_.session_id();
  if (!client_.empty() && !details.contains("client")) details["client"] = client_;
  // Who released or asked names the source there.
  if (schedule_.run() && !details.contains("source") && type != "EMERGENCY" && type != "EMERGENCY_RESET_REFUSED") {
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
    if (emergency) recording_.DisarmTeleop();
    const Reason why = EmergencyWhy(reason_);
    if (emergency && state_ == State::kDriving) CancelDrive(why);
    if (emergency && state_ == State::kMowing) Pause(kPauseEmergency, why);
    // Like OpenMower: the emergency tells, no UNDOCKING_FAILED or DOCKING_FAILED.
    if (emergency && state_ == State::kUndocking) StopUndocking(std::nullopt, why);
    if (emergency && state_ == State::kDocking) EndDocking(why);
    if (emergency && state_ == State::kCalibrating) calibration_.EndCalibration(CoreAccess(*this), why, true);
    if (emergency && state_ == State::kSetup) setup_.BladeStopped();
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
    out.invalid = mowing_.ReadStartRequest(CoreFacts(*this), params, &start, &out.invalid_field);
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
  const Reason why = name == "mowing.start"
                         ? mowing_.RequestStart(CoreAccess(*this), std::move(start), node_problem, &out)
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
    if (!mowing_.job().Unfinished()) return Reason("NO_JOB", "There is no unfinished job");
    out->result["job_id"] = mowing_.job().progress().job_id;
    // The event carries the id of the dropped job, the checkpoint is saved without it.
    AddEvent("JOB_RESET", json::object());
    mowing_.job().Reset();
    CheckpointNow();
    SetDue<ActionsChanged>(true);
    return {};
  }
  if (name.rfind("mowing.", 0) == 0) {
    if (state_ != State::kMowing) return WrongState();
    const int waiting = kPauseWaitsForUser;
    if (name == "mowing.pause") {
      if (mowing_.pause() & kPauseManual) return WrongState();
      Pause(kPauseManual, Reason("MANUAL", "Paused by the app"));
      return {};
    }
    if (name == "mowing.continue") {
      if ((mowing_.pause() & (waiting | kPauseObstacle)) == 0) return WrongState();
      // The EMERGENCY event told already: no MOWING_REFUSED.
      if (emergency_) return Reason("EMERGENCY", "Emergency: reset it first");
      // Without a safety measurement or a usable mowing map the pause stays.
      Reason why = Unsafe();
      if (why.empty()) why = mow_map_problem_;
      if (why.empty()) why = Held() ? HoldProblem() : ContinueProblem();
      // The user lets the mower go past what it cannot see; what a sensor sees in the stop zone keeps it.
      if (why.empty() && (mowing_.pause() & kPauseObstacle) && !obstacle_part_.Waive()) {
        why = obstacle_part_.Why().WithMessage("An obstacle in view, or one the sensors report no more, is in the way");
      }
      if (!why.empty()) {
        RefuseStart(why);
        return why;
      }
      // Without GPS it still waits for a good position.
      Unpause(waiting | kPauseObstacle);
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
    const int now = area ? mowing_.job().progress().area : mowing_.job().progress().path;
    if (params.contains(key) && params[key].get<double>() != now) {
      return Reason("CHANGED", std::string("The ") + key + " changed meanwhile: nothing was skipped").With(key, now);
    }
    out->result[key] = now;
    (area ? mowing_.skip_area() : mowing_.skip_path()) = true;
    if (area) mowing_.skip_why() = Reason("MANUAL", "Skipped by the app");
    mowing_.skip_client() = client_;
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
    if (why.empty() && !recording_.map_writable()) why = recording_.map_reason();
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
  const AreaRecorder::Result result = recording_.recorder().OnAction(
      action.substr(kRecordingPrefix.size() + 1), &why, params.value("meters", kUndoDefaultM));
  recording_.PullRecorder(CoreAccess(*this));
  if (result != AreaRecorder::Result::kDone) return why.empty() ? WrongState() : why;
  if (command.name == "recording.undo") out->result["points"] = recording_.recorder().current().size();
  if (command.name == "recording.ring_stop" || command.name == "recording.save") {
    out->result["trimmed_points"] = recording_.recorder().trimmed_points();
  }
  return {};
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
    const bool approach = docking_part_.last_kind() == DockingLeg::Kind::kApproach;
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
  if (state_ == State::kUndocking) return undocking_.Leaving();
  if (state_ == State::kDocking) return docking_part_.MapLegActive();
  return state_ == State::kMowing && mowing_.pause() == 0 && !mowing_.stopping() && LegDrives();
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
    mowing_.Enter(CoreAccess(*this));
  }
  if (state_ == State::kMowing) {
    mowing_.Leave(CoreAccess(*this));
  }
  if (state_ == State::kMowing || state == State::kMowing) {
    home_.StartOver();
  }
  // Entering and leaving both start from nothing; what was not saved is gone, like with OpenMower.
  recording_.recorder().Reset();
  recording_.recorder().TakeActionsChanged();
  state_ = state;
  recording_.DisarmTeleop();
  json details = {{"state", CurrentStateName()}};
  if (state == State::kDriving) details["sub_state"] = "DRIVING";
  if (state == State::kCalibrating) details["sub_state"] = "CALIBRATING";
  if (state == State::kSetup) details["sub_state"] = "SETUP";
  AddEvent("STATE", std::move(details));
  SetDue<ActionsChanged>(true);
  // The run of the schedule ends once the mower is idle again after it left the station.
  if (state == State::kIdle && schedule_.run() && schedule_.run()->phase >= RunPhase::kUndocking) FinishRun();
}

std::string LogicCore::CurrentStateName() const {
  if (state_ == State::kMowing && mowing_.pause() != 0) return "PAUSED";
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
    s.current_area = mowing_.job().progress().area;
    s.current_path = mowing_.job().progress().path;
    s.current_path_index = mowing_.job().progress().index;
    s.task_index = mowing_.job().TaskIndex();
    s.task_count = mowing_.job().TaskCount();
    const Reason& paused = mowing_.PauseReason();
    s.pause_reason = paused.code;
    if (paused.fields.contains("kind")) s.pause_kind = paused.fields["kind"].get<std::string>();
  }
  if (state_ == State::kAreaRecording) s.sub_state = recording_.recorder().SubState();
  if (state_ == State::kDriving) s.sub_state = "DRIVING";
  if (state_ == State::kCalibrating) s.sub_state = "CALIBRATING";
  if (state_ == State::kSetup) s.sub_state = "SETUP";
  s.emergency = emergency_;
  s.emergency_reason = reason_;
  s.is_charging = hw_.battery_ok && hw_.dock_contact;
  s.rain_detected = hw_.telemetry_ok && hw_.rain;
  s.hardware = hw_.telemetry_ok;
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
                      idle && !emergency_ && recording_.map_writable() && HoldProblem().empty()},
                     {"reset_job", "Discard unfinished job", idle && mowing_.job().Unfinished()}}});
  // Pause and continue take turns, like in OpenMower; continue waits for the emergency to be reset.
  const bool waits = (mowing_.pause() & kPauseWaitsForUser) != 0;
  const bool obstacle = (mowing_.pause() & kPauseObstacle) != 0;
  groups.push_back({kMowingPrefix,
                    {{"pause", "Pause", mowing && !waits},
                     {"continue", "Resume", mowing && (waits || obstacle) && !emergency_ && !Held()},
                     {"abort_mowing", "End mowing", mowing},
                     {"skip_area", "Next area", mowing},
                     {"skip_path", "Next path", mowing}}});
  groups.push_back({kUndockingPrefix, {{"abort_undocking", "Cancel undocking", state_ == State::kUndocking}}});
  groups.push_back({kDockingPrefix, {{"abort_docking", "Cancel docking", state_ == State::kDocking}}});
  // The recorder decides while recording; outside of it all are off.
  const std::vector<bool> enabled = recording_.recorder().Enabled();
  const std::vector<std::string>& names = AreaRecorder::ActionLabels();
  ActionGroup recording_group{kRecordingPrefix, {}};
  for (size_t i = 0; i < names.size(); i++) {
    recording_group.actions.push_back({AreaRecorder::ActionNames()[i], names[i], recording && enabled[i]});
  }
  groups.push_back(std::move(recording_group));
  return groups;
}

bool LogicCore::PoseGood() const {
  return pose_ && pose_->position_accuracy <= config_.max_position_accuracy && pose_->orientation_valid;
}

Reason LogicCore::Busy() const {
  if (state_ == State::kIdle) {
    if (undocking_.dock_check() == DockCheck::kNone) return {};
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
  // On the docking station the reference check sets the heading, and it wants a better position; until then the
  // spread of the heading counts not.
  const bool docked = DockContact();
  const double limit = docked ? DockAccuracyLimit() : config_.max_position_accuracy;
  const float accuracy =
      docked ? std::min(pose_->position_accuracy, pose_->position_accuracy_at_heading) : pose_->position_accuracy;
  if (!(accuracy <= limit)) {
    Reason why = AccuracyReason(accuracy, limit);
    if (docked) why.message += " on the docking station";
    return why;
  }
  if (!pose_->orientation_valid && !docked) {
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
  if (DataRollbackOpen()) {
    const json& backup_of = data_rollback_.contains("backup_of") ? data_rollback_["backup_of"] : json();
    return Reason("DATA_ROLLBACK", "The data went back to an older backup: check the map and confirm it in the app")
        .With("backup_of", backup_of.is_number_integer() ? backup_of.get<int64_t>() : 0);
  }
  if (hw_.rain) return Reason("RAIN", "Rain detected");
  if (!hw_.battery_ok) return Reason("BATTERY_LOW", "No battery level from the driver");
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
  if (!mowing_.job().HasMowableArea(tasks)) {
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
  return mowing_.job().progress().job_id;
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

Reason LogicCore::HoldProblem() const {
  if (hold_until_ > 0) {
    return Reason("HOLD", "An update holds the mower: nothing starts until it is done").With("until", hold_until_);
  }
  if (hw_.hold) return Reason("HOLD", "The driver holds the mower for an update: nothing starts until it is done");
  return StorageProblem();
}

bool LogicCore::HoldAllowed() const {
  return state_ == State::kIdle && undocking_.dock_check() == DockCheck::kNone && !schedule_.run();
}

void LogicCore::SetGpsSettings(double max_position_accuracy, double gps_wait_time, double gps_timeout) {
  config_.max_position_accuracy = max_position_accuracy;
  config_.gps_wait_time = gps_wait_time;
  config_.gps_timeout = gps_timeout;
  recording_.recorder().SetMaxPositionAccuracy(max_position_accuracy);
}

void LogicCore::OnTick(double now_s) {
  auto& active = schedule_.run();
  const auto& answered = release_.release_answered_s();
  // Any end of the emergency drops the request, so one still pending is an emergency that stayed.
  if (answered && now_s - *answered > kReleaseSourceS) {
    RefuseReset(Reason("STILL_ACTIVE", "The emergency stays: " + EmergencyReasonNames(reason_))
                    .With("bits", mowbite_api::EmergencyNameList(reason_)));
  }
  WatchAnnouncement();
  obstacle_part_.Tick(now_s);
  ApplyObstacleHold();
  if (active && active->phase == RunPhase::kAnnouncing &&
      now_s >= active->announce_until) {
    EndAnnouncement(now_s);
  }
  if (state_ == State::kDocking) TickDocking(now_s);
  if (state_ == State::kAreaRecording) recording_.TickRecording(CoreAccess(*this), now_s);
  if (state_ != State::kMowing) return;
  mowing_.TickGpsWait(CoreAccess(*this), now_s);
  WatchHome(now_s);
  if (state_ != State::kMowing) return;
  mowing_.TickCheckpoint(CoreAccess(*this), now_s);
}

bool LogicCore::BladeWanted() const {
  if (state_ == State::kSetup) {
    return setup_.setup_blade() != 0 && !setup_.setup_blade_refused() && !emergency_ && hw_.telemetry_ok &&
           !hw_.dock_contact;
  }
  if (state_ != State::kMowing || mowing_.pause() != 0 || mowing_.stopping() || emergency_ || !config_.mow_with_blade ||
      !safety_measured_ || !hw_.telemetry_ok || hw_.dock_contact) {
    return false;
  }
  if (mowing_.spinup_since()) return true;
  // Around an object the blade runs (G46).
  if (mowing_.leg() == Leg::kDetour) return true;
  const JobPose* pose = mowing_.job().CurrentPose();
  return mowing_.leg() == Leg::kFollow && pose && IsMowKind(pose->kind);
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
