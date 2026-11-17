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

#include "mowbite_setup/steps.h"

namespace mowbite_setup {

// hw/status as the checks need it.
struct Hardware {
  uint32_t emergency_reason = 0;
  bool stop = false;
  bool lift = false;
  bool bumper = false;
  bool bumper_left = false;
  bool bumper_right = false;
  bool dock_contact = false;
  // A reading of the battery is there, and its charge (%).
  bool battery = false;
  double battery_percentage = 0;
  // Names of the motor controllers that are not ok.
  std::vector<std::string> motors_down;
  uint64_t link_errors = 0;
};

// hw/capabilities as the checks need it.
struct Capabilities {
  // Board and version, as the driver names them.
  std::vector<std::pair<std::string, std::string>> firmware;
  std::string safety_measurement_date;
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
  // current_state of the logic, empty before the first.
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
  void OnCapabilities(const Capabilities& capabilities);
  void OnImu(const ImuSample& sample, double now);
  // solution as GnssFix has it (3 float, 4 fixed), accuracy in m.
  void OnFix(int solution, double accuracy, double now);
  void OnWorld(const World& world);
  // Whether setup.json could be written the last time; without, nothing changes the state.
  void SetWritable(bool writable);
  bool writable() const { return writable_; }
  // Moves the running step on and ends the steps that wait for what the OS sees.
  void Tick(double now, double wall);

  // setup.*: the params as the call gave them. Nothing when done.
  std::optional<Refusal> Start(const json& params, double now, double wall);
  std::optional<Refusal> Answer(const json& params, double wall);
  std::optional<Refusal> Skip(const json& params, double wall);
  std::optional<Refusal> Abort(const json& params, double now, double wall);
  std::optional<Refusal> Reset(const json& params);

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
  };
  // The part of the running step: what it waits for.
  enum class Phase { kRest, kArmed, kSwitched, kWatch };
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
    double gravity[3] = {0, 0, 0};
    bool gravity_known = false;
    double yaw = 0;
    double last_imu = -1;
    // position
    double good_since = -1;
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
  void Fail(const std::string& reason, const std::string& message, json fields, double now, double wall);
  bool StageDone(const std::string& stage) const;
  bool Complete() const;
  std::string FirmwareText() const;
  static bool Fresh(double last, double now, double limit);
  bool Signal(const std::string& role) const;
  bool IsRun(const Step& step) const;
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
};

}  // namespace mowbite_setup

#endif  // MOWBITE_SETUP_PROGRAM_H
