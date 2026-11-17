// The setup program: the state of every step of the list, what setup.* does to it, and the checks that watch the
// hardware without moving anything. Without ROS; the node feeds what it hears and sends what comes out.
#ifndef MOWBITE_SETUP_PROGRAM_H
#define MOWBITE_SETUP_PROGRAM_H

#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "mowbite_hold/lock.h"
#include "mowbite_setup/layer.h"
#include "mowbite_setup/steps.h"

namespace mowbite_setup {

// hw/status as the checks need it.
struct Hardware {
  uint32_t emergency_reason = 0;
  // Goes up with every reason that latches, 0 from a driver that does not count.
  uint32_t latch_count = 0;
  // The driver hears its board.
  bool telemetry_ok = false;
  bool stop = false;
  bool lift = false;
  bool bumper = false;
  bool bumper_left = false;
  bool bumper_right = false;
  bool dock_contact = false;
  // An update holds the mower.
  bool hold = false;
  // A reading of the battery is there, and its charge (%).
  bool battery = false;
  double battery_percentage = 0;
  // Names of the motor controllers that are not ok.
  std::vector<std::string> motors_down;
  uint64_t link_errors = 0;
};

// One input of hw/inputs.
struct Line {
  std::string name;
  std::string role;
  std::string reason;
  bool active = false;
  bool assigned = false;
  // The board keeps it switched after the sensor is free, until the emergency is released.
  bool held = false;
  bool active_high = true;
  int delay_ms = 0;
};

// A release of the emergency the node asks safety/release_emergency for.
struct ReleaseRequest {
  uint32_t latch_count = 0;
  std::string reason;
};

// hw/capabilities as the checks need it.
struct Capabilities {
  // Board and version, as the driver names them.
  std::vector<std::pair<std::string, std::string>> firmware;
  std::string safety_measurement_date;
  // What the driver offers the steps that move the wheels: the longest hw/silence (s, 0 for none) and hw/end.
  double silence_max_s = 0;
  bool driver_end = false;
};

// setup/motion while a step that moves the wheels runs: the bounds mowbite_safety holds its commands to.
struct Motion {
  std::string step;
  bool raised = true;
  double max_linear = 0;
  double max_angular = 0;
};

// A command on setup/cmd_vel: m/s and rad/s.
struct Command {
  double linear = 0;
  double angular = 0;
};

// imu/data: acceleration (m/s²) and turn rate (rad/s) in base_link.
struct ImuSample {
  double ax = 0;
  double ay = 0;
  double az = 0;
  double gz = 0;
};

// What the other nodes report, for the steps that wait for it.
struct World {
  // current_state of the logic, with IDLE replaced by its sub state while it drives or calibrates
  // (mowbite_hold::SessionState): IDLE is a mower that stands. Empty before the first.
  std::string state;
  int areas = 0;
  int docking_stations = 0;
  bool mowing_allowed = false;
  // Correction data arrives.
  bool ntrip = false;
  // Keys of robot.geometry the user confirmed.
  std::set<std::string> confirmed;
};

// A method that was not done: -32602 with field for a param at fault, else -32600 with the reason of SETUP_REFUSED.
struct Refusal {
  std::string reason;
  std::string message;
  json data = json::object();
  std::string field;
};

struct Event {
  std::string type;
  json details;
};

class Program {
 public:
  Program(StepList list, std::string platform);

  // The record written from Text(). False with why for a file that is no record of this platform; then all is open.
  // A step that ran when the OS stopped is open again and ended with INTERRUPTED.
  bool Load(const std::string& text, double wall, std::string* why);
  // <data_dir>/setup.json: the record of the program, never a setting.
  std::string Text() const;

  // What the node hears; now is a monotonic clock (s), wall unix s.
  void OnHardware(const Hardware& hardware, double now);
  // hw/inputs: the lines of the platform, and whether the driver learns them (then active is the level of a line).
  void OnInputs(const std::vector<Line>& lines, bool learning = false);
  // The parameter file of the program as it is on disk, and <node>.<parameter> of what the file kept by hand
  // overrules of it. Without this call the program has no place for the file.
  void SetLayer(Layer layer, std::set<std::string> shadowed);
  void OnCapabilities(const Capabilities& capabilities);
  void OnImu(const ImuSample& sample, double now);
  // solution as GnssFix has it (3 float, 4 fixed), accuracy in m.
  void OnFix(int solution, double accuracy, double now);
  void OnWorld(const World& world);
  // hw/wheel_odom as the speeds of the wheels (m/s, forwards positive).
  void OnWheels(double left, double right, double now);
  // track of robot.geometry (m), 0 while unknown.
  void SetTrack(double track);
  // How often the start would start an ended driver again from now on.
  void SetDriverRestarts(int left);
  // Whether setup.json could be written the last time; without, nothing changes the state.
  void SetWritable(bool writable);
  bool writable() const { return writable_; }

  // The lock of autonomous driving and mowing (mowbite_hold/lock.h): <data_dir>/safety_lock.json as the start found
  // it, nothing when it is not there, an empty text when it cannot be read (then a reset closes it). A cause of a
  // step the list does not have as a safety step becomes a reset. A safety step that fails with a reason that closes
  // puts its cause in (failures + 1), passing it takes the cause out; skip and setup.reset leave them.
  void LoadLock(const std::optional<std::string>& text, double wall);
  // The file to write now, once, before setup.json and the events (empty: to remove); LockWritten tells how it went.
  std::optional<std::string> TakeLock();
  void LockWritten(bool written);
  bool lock_writable() const { return lock_writable_; }
  // Moves the running step on and ends the steps that wait for what the OS sees.
  void Tick(double now, double wall);

  // setup.*: the params as the call gave them. Nothing when done.
  std::optional<Refusal> Start(const json& params, double now, double wall);
  std::optional<Refusal> Answer(const json& params, double now, double wall);
  std::optional<Refusal> Skip(const json& params, double wall);
  std::optional<Refusal> Abort(const json& params, double now, double wall);
  std::optional<Refusal> Reset(const json& params);
  // setup.keep: keeps a step that moves the wheels going; result tells whether the step runs.
  std::optional<Refusal> Keep(const json& params, double now, json* result);

  // What a step that moves the wheels asks for now: setup/motion while it runs (nothing else), the command to send
  // (nothing: none), and once at its end the step to send setup/motion with active false for.
  std::optional<Motion> MotionNow() const;
  std::optional<Command> CommandNow() const;
  std::optional<std::string> TakeMotionEnd();
  // hw/silence (seconds) and hw/end to call now, once each, and their answers.
  std::optional<double> TakeSilence();
  bool TakeEnd();
  void OnSilenceAnswer(bool success, const std::string& code, double now, double wall);
  void OnEndAnswer(bool success, const std::string& code, double now, double wall);

  // setup.get and setup/json. images: name to {sha256, bytes} of the images that are there.
  json State(double now, const std::map<std::string, std::pair<std::string, int>>& images) const;

  // What blocks the steps now, as one text: it changes without a change of version.
  std::string BlockedNow(double now) const;

  int version() const {
    return version_;
  }
  bool running() const {
    return run_.has_value();
  }
  // The record changed since the last call: save Text() and send the state.
  bool TakeChanged();
  std::vector<Event> TakeEvents();
  // A release to ask for now, once.
  std::optional<ReleaseRequest> TakeRelease();
  // The parameter file to write now, once (empty: to remove); LayerWritten tells how it went.
  std::optional<std::string> TakeLayer();
  void LayerWritten(bool written, double now, double wall);
  // Whether to restart the OS now, once: a step asked for it, record and parameter file are on disk, the mower
  // stands in IDLE and no emergency is on but the one of learning.
  bool TakeRestart(double now);
  const std::string& restart_way() const {
    return list_.restart;
  }

 private:
  // What a step holds between restarts.
  struct Record {
    // open, running, ok, failed or skipped.
    std::string state = "open";
    int attempt = 0;
    json result;
    json values = json::array();
    // open, waiting, ok or failed by part, as the last attempt left them.
    json parts = json::array();
    json answers = json::object();
    // A step inputs_assign under way: phase learn (the driver learns, part by part) or apply (the table is written),
    // and the line of every part taken so far. It outlives the restarts of the step.
    json assign;
  };
  // Where a step inputs_assign is: waiting to be restarted, for the driver after it, the lines untouched, a part, its
  // question, the lines back at rest.
  enum class Assign { kRestart, kDriver, kQuiet, kPart, kAsk, kRest };
  // The part of the running step: what it waits for.
  enum class Phase { kRest, kArmed, kSwitched, kWatch };
  // A step that moves the wheels: the logic takes SETUP, the wheels speed up, the trigger, they stand, nothing starts,
  // the emergency it brought, its release, armed again, a new command turns them, they stand again. wheels: a part
  // drives and is measured, one_wheel waits for the answer. driver_end waits for the driver after the trigger.
  enum class Drive {
    kEnter,
    kRunUp,
    kTrigger,
    kStop,
    kReturn,
    kQuiet,
    kLatch,
    kRelease,
    kArm,
    kRestart,
    kPause,
    kMeasure,
    kAsk,
  };
  struct Run {
    size_t step = 0;
    double began = 0;
    double wall_began = 0;
    size_t part = 0;
    double part_began = 0;
    double part_wall = 0;
    Phase phase = Phase::kRest;
    // When the input switched and when it was let go again.
    double edge = 0;
    double released = 0;
    // Since when the emergency of the step is on without the input, below 0 while it is off.
    double latched_since = -1;
    // Another input that switched while this part waited.
    std::string other;
    // Lines of the role that were active since the part was armed, and those of the parts before.
    std::set<std::string> part_lines;
    std::set<std::string> lines;
    // The part saw a press, but only of lines an earlier part had already.
    bool repeated = false;
    std::vector<std::string> part_states;
    // What a part measured: the delay of the emergency (s) or the angle reached (deg).
    std::vector<std::optional<double>> measured;
    // board
    int status_count = 0;
    uint64_t link_errors_first = 0;
    std::set<std::string> inputs_seen;
    std::set<std::string> motors_down;
    double gravity_sum = 0;
    int gravity_count = 0;
    // imu: gravity at the start of the step, the filtered one of now, and the turn since the part began (rad).
    int rest_count = 0;
    double rest[3] = {0, 0, 0};
    // The first imu/data of the run, from which the rest is taken; below 0 before it.
    double rest_began = -1;
    double gravity[3] = {0, 0, 0};
    bool gravity_known = false;
    double yaw = 0;
    double last_imu = -1;
    // position
    double good_since = -1;
    // inputs_assign: the stage and since when (below 0: from the next tick), the level of every line at rest, the
    // lines that left it with the level they went to and when the first did, what a try of the part showed more than
    // one line of, and the line ask names.
    Assign assign = Assign::kRestart;
    double stage_began = 0;
    std::map<std::string, bool> levels;
    std::map<std::string, bool> changed;
    double first_change = -1;
    json ambiguous = json::array();
    std::string proposed;
    bool proposed_high = true;
    // A step that moves the wheels: where it is and since when, the trigger, standing wheels, the silence or the
    // return of the driver, the last setup.keep, the command, when the wheels began to move, a service asked for.
    Drive drive = Drive::kEnter;
    double drive_began = 0;
    double drive_wall = 0;
    double trigger = -1;
    double stood = -1;
    double silence_end = -1;
    double returned = -1;
    // driver_end: hw/status was away after hw/end; the new driver counts once it hears its board.
    bool gap = false;
    double keep = 0;
    std::optional<Command> command;
    double move_began = -1;
    bool asked = false;
    double asked_at = -1;
    bool armed = false;
    // The emergencies a stop or end of the driver may bring, LATCH aside; 0 before the trigger.
    uint32_t expected = 0;
    // The round of a step silence, the stand times so far, and what a part of wheels measured.
    int round = 0;
    std::vector<double> stands;
    double sum_left = 0;
    double sum_right = 0;
    int samples = 0;
    std::string answer;
    std::optional<double> returned_after;
    // A safety step that saw nothing of the action of a part asks pressed (below 0: it does not): since when, and the
    // reason it fails with on yes.
    double pressed_at = -1;
    double pressed_wall = 0;
    std::string pressed_reason;
    json pressed_fields;
  };

  // The emergency a part of mode press brought, which the program releases itself. It exists only while every
  // condition holds: gone, the emergency waits for emergency.reset.
  //  - the input switched and its emergency followed within the limit of the step (confirmed);
  //  - since the input switched hw/status never showed a bit other than this emergency and LATCH;
  //  - the driver counts its latches, and the release carries the count of the last hw/status;
  //  - the mower is in IDLE.
  // A try comes at the earliest kReleaseEvery after the input switched and then as often; where the board does not
  // hold the input, only once it is free. The tries end kReleaseAfter after the step. Only hw/status tells the
  // outcome.
  struct OwnLatch {
    std::string role;
    uint32_t bit = 0;
    bool held = false;
    bool confirmed = false;
    double edge = 0;
    double last_try = -1;
    // No try after this; below 0 while the step runs.
    double until = -1;
  };

  const Step& StepOf(const Run& run) const {
    return list_.steps[run.step];
  }
  Record& RecordOf(const Step& step) {
    return records_[step.id];
  }
  bool NeedsMet(const Step& step) const;
  std::string Shown(const Step& step) const;
  json Blocked(const Step& step, double now) const;
  json Values(const Step& step, double now, bool final) const;
  std::optional<Refusal> Common(const json& params, const Step** step) const;
  void Changed();
  void End(const Step& step, const std::string& state, const std::string& reason, const std::string& message,
           json fields, double wall);
  void Pass(const Step& step, double now, double wall);
  // The cause of a failed safety step into the lock, and out again when it passes.
  void CloseLock(const Step& step, const std::string& reason, const json& fields, double wall);
  void ReleaseLock(const Step& step);
  bool LockOpen() const;
  json SafetyState() const;
  // The part of the running safety step saw nothing within its time: it asks pressed. On yes it fails with reason and
  // fields, on no or without an answer within kSetupPressedTime it ends open with NOT_DONE.
  void AskPressed(const std::string& reason, json fields, double now, double wall);
  void TickPressed(double now, double wall);
  std::optional<Refusal> AnswerPressed(const Step& step, const json& params, double now, double wall);
  // Whether a step of this check can ask pressed.
  static bool CanAsk(const Step& step);
  void Fail(const std::string& reason, const std::string& message, json fields, double now, double wall);
  bool StageDone(const std::string& stage) const;
  bool Complete() const;
  std::string FirmwareText() const;
  static bool Fresh(double last, double now, double limit);
  bool Signal(const std::string& role) const;
  // The assigned lines a part of this role can switch.
  std::vector<const Line*> LinesOf(const std::string& role) const;
  void NoteLines();
  void WatchOwnLatch();
  void TickRelease(double now);
  void EndRun(double now);
  // Takes inputs.learn of the step out of the parameter file, if the program put it there.
  void ForgetLearning(const Step& step);
  bool RestartAllowed(double now) const;
  void WantRestart();
  void TickLearning(double now);
  void TickAssign(double now, double wall);
  void EnterAssign(Assign stage, double now);
  void BeginAssignPart(double now, double wall);
  void FinishAssign(double now, double wall);
  void NoteLevels();
  // Takes the levels of the lines as those at rest: a change is counted from here.
  void TakeLevels();
  // Ends a part of inputs_assign no single line came for: AMBIGUOUS after more than one changed, else NO_LINE.
  void FailNoLine(const std::string& message, double now, double wall);
  bool AtRest() const;
  bool TableApplied(const Step& step) const;
  std::optional<Refusal> AnswerAssign(const Step& step, const json& params);
  std::optional<Refusal> AnswerDelays(const Step& step, const json& params, double now, double wall);
  // What keeps a step inputs_assign from writing for the driver and restarting it; nothing when it can.
  std::optional<Refusal> AssignRefused(double now) const;
  // The run of a step inputs_assign that waits for the driver after a restart, with every part taken (apply) or none.
  Run AssignRun(const Step& step, bool apply, double wall) const;
  // The delay the line of a part gets (ms): what setup.answer set for it, at most the one of the step list.
  int DelayOf(const Step& step, const Part& part) const;
  bool IsRun(const Step& step) const;
  // motion.cpp: the steps that move the wheels.
  static bool IsDrive(const Step& step);
  // What keeps a step that moves the wheels from starting now; the first of them refuses setup.start.
  std::vector<Refusal> DriveProblems(const Step& step, double now) const;
  void BeginDrive(double now, double wall);
  void TickDrive(double now, double wall);
  void NoteDriverBack(const Hardware& hardware, double now);
  void TickWheels(double now, double wall);
  void EnterDrive(Drive drive, double now, double wall);
  void NextPass(double now, double wall);
  void DriveFail(const std::string& reason, const std::string& message, json fields, double now, double wall);
  // The emergencies that may stand now in the running step, LATCH with them; anything else ends it.
  uint32_t AllowedNow() const;
  bool Standing(double now) const;
  bool Started() const;
  bool Turning(double speed) const;
  double Speed(const Step& step) const;
  double TurnRate(const Step& step) const;
  std::optional<Refusal> AnswerWheel(const Step& step, const json& params);
  json DriveValues(const Run& run, const Step& step, bool final) const;
  json MotionOf(const Step& step) const;
  // The position has the solution the step asks for.
  bool SolutionOk(const Step& step) const;
  void BeginPart(double now, double wall);
  void PartDone(double now, double wall);
  void TickBoard(double now, double wall);
  void TickInput(double now, double wall);
  void TickImu(double now, double wall);
  void TickPosition(double now, double wall);
  void TickWaiting(double wall);
  void TrackWorld(double now);
  double FixRate(double now) const;
  json BoardValues(const Run& run, const Step& step, double elapsed, bool final) const;
  std::optional<double> ImuValue(const Run& run, const std::string& move) const;

  StepList list_;
  std::string platform_;
  std::map<std::string, Record> records_;
  int version_ = 1;
  bool writable_ = true;
  bool changed_ = false;
  std::vector<Event> events_;
  std::optional<Run> run_;

  Hardware hardware_;
  double hardware_at_ = -1;
  std::vector<Line> lines_;
  std::optional<OwnLatch> own_;
  std::optional<ReleaseRequest> release_;
  bool inputs_known_ = false;
  bool learning_ = false;
  Layer layer_;
  // As the file is on disk: what layer_ goes back to when a write fails; when one did last (below 0: none).
  Layer layer_disk_;
  double layer_failed_ = -1;
  bool layer_set_ = false;
  std::set<std::string> shadowed_;
  // The file differs from layer_; it is with the node to be written.
  bool layer_dirty_ = false;
  bool layer_pending_ = false;
  // A restart is to be asked for; when it was (below 0: not yet).
  bool restart_wanted_ = false;
  double restart_sent_ = -1;
  // A step inputs_assign ended without its table: the driver may still learn.
  bool leave_learning_ = false;
  Capabilities capabilities_;
  bool capabilities_known_ = false;
  double imu_at_ = -1;
  int solution_ = 0;
  double accuracy_ = 0;
  std::deque<double> fixes_;
  World world_;
  // The mower left the station, mowed and is on its way back by itself: what dock_test and mow wait for.
  bool undocked_ = false;
  bool mowed_ = false;
  bool docking_ = false;
  double docking_idle_since_ = -1;
  std::set<std::string> reached_;
  // hw/wheel_odom: the last speeds, the same filtered, when they came, and since when both stood below kStandSpeed.
  double left_ = 0;
  double right_ = 0;
  double left_filtered_ = 0;
  double right_filtered_ = 0;
  double wheels_at_ = -1;
  double still_since_ = -1;
  double track_ = 0;
  int restarts_left_ = 0;
  // The step to end setup/motion for, and the services to call.
  std::optional<std::string> motion_end_;
  std::optional<double> silence_due_;
  bool end_due_ = false;
  // The lock as the file holds it, whether it is to be written, and whether the last write went through.
  mowbite_hold::LockRecord lock_;
  bool lock_due_ = false;
  bool lock_writable_ = true;
};

}  // namespace mowbite_setup

#endif  // MOWBITE_SETUP_PROGRAM_H
