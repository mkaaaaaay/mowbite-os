#ifndef MOWBITE_LOGIC_LOGIC_CORE_H
#define MOWBITE_LOGIC_LOGIC_CORE_H

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "mowbite_logic/area_recorder.h"

namespace mowbite_logic {

// What the logic needs from hw/status.
struct HardwareSnapshot {
  bool telemetry_ok = false;
  // Blocked by the driver; the reason bits come from it, numbered like the EMERGENCY event.
  bool emergency = false;
  int emergency_reason = 0;
  bool rain = false;
  bool dock_contact = false;
  bool battery_ok = false;
  float battery_voltage = 0;
  // Positive while discharging.
  float battery_current = 0;
  uint8_t battery_percentage = 0;
  int8_t battery_temperature = 0;
  struct Motor {
    bool ok = false;
    int16_t rpm = 0;
    float current = 0;
    int8_t temperature = 0;
  };
  Motor blade;
  Motor left;
  Motor right;
};

// What the logic needs from localization/pose.
struct PoseSnapshot {
  double x = 0;
  double y = 0;
  double heading = 0;
  float position_accuracy = 999;
  float orientation_accuracy = 999;
  bool orientation_valid = false;
};

struct LogicConfig {
  // 6S Li-ion; empty is an assumption until the discharge curve of the Tron is measured.
  double battery_empty_voltage = 19.2;
  double battery_full_voltage = 25.2;
  // The MCU reports no charge voltage, dock contact stands in for it.
  double dock_charge_voltage = 28.0;
  // GPS counts as good up to this accuracy (m), like max_position_accuracy of OpenMower. The localization reads the
  // same parameter (tron.yaml, /**).
  double max_position_accuracy = 0.1;
  // Its max_position_accuracy is the one above.
  RecorderConfig recorder;
};

// map/status of mowbite_map: whether the map fits the datum of the GNSS.
enum class MapDatum { kUnknown, kEmpty, kMatch, kMismatch, kUnbound };

struct Action {
  std::string id;
  std::string name;
  bool enabled = false;
};

struct ActionGroup {
  std::string prefix;
  std::vector<Action> actions;
};

struct RobotStateData {
  std::string state;
  std::string sub_state;
  float battery_percentage = 0;
  bool emergency = false;
  bool is_charging = false;
  bool rain_detected = false;
  // 1 at perfect accuracy, 0 at max_position_accuracy or worse, without a recent pose or without a heading.
  // OpenMower sends -1 for a bad position with a valid heading; a negative percentage is not shown here.
  float gps_percentage = 0;
  double pose_x = 0;
  double pose_y = 0;
  double pose_heading = 0;
  // 999 without a recent pose: an accuracy of 0 would read as a perfect RTK fix in the app.
  float pos_accuracy = 999;
  float heading_accuracy = 999;
  bool heading_valid = false;
  // Bits as in EMERGENCY events, 0 without emergency.
  int emergency_reason = 0;
};

constexpr uint8_t kSensorTypeString = 1;
constexpr uint8_t kSensorTypeDouble = 2;

struct SensorDefinition {
  std::string id;
  std::string name;
  std::string unit;
  uint8_t value_type = kSensorTypeDouble;
  uint8_t value_description = 0;
  bool has_min_max = false;
  double min_value = 0;
  double max_value = 0;
  bool has_critical_low = false;
  double lower_critical_value = 0;
  bool has_critical_high = false;
  double upper_critical_value = 0;
};

struct SensorReading {
  std::string id;
  double value = 0;
  std::string text;
};

enum class ActionResult { kUnknown, kRejected, kDone, kReleaseEmergency };

struct ReasonChange {
  int from = 0;
  int to = 0;
};

// Bits of the reason in an EMERGENCY event, as the app reads them (EmergencyReason of OpenMower) and as
// hw/status carries them.
constexpr int kEmergencyLatch = 1 << 0;
constexpr int kEmergencyTimeoutInputs = 1 << 1;
constexpr int kEmergencyStop = 1 << 2;
constexpr int kEmergencyLift = 1 << 3;
constexpr int kEmergencyCollision = 1 << 5;
constexpr int kEmergencyHighLevel = 1 << 7;
constexpr int kEmergencyFirmwareIncompatible = 1 << 12;

// "LATCH, LIFT": the names of the bits as OpenMower logs them, in bit order.
std::string EmergencyReasonNames(int reason);

// The app repeats teleop every 100 ms while the joystick is held and sends one zero when it is let go.
constexpr double kTeleopQuietS = 1.0;

// Mower states and actions: IDLE and AREA_RECORDING (remote control and recording of areas), emergency and sensors.
class LogicCore {
 public:
  explicit LogicCore(const LogicConfig& config);

  void OnHardwareStatus(const HardwareSnapshot& hw);
  // hw/status stopped coming.
  void OnHardwareTimeout();
  void OnPose(const PoseSnapshot& pose);
  // localization/pose stopped coming; events keep the last position.
  void OnPoseTimeout();
  // The localization moved its estimate onto GPS again by distance_m, the event is at the new position.
  void OnPositionJump(double to_x, double to_y, double distance_m, bool after_outliers);
  ActionResult OnAction(const std::string& action_id);
  // Recording starts and saves only while the map fits the datum in use, or is still empty and takes it.
  void OnMapStatus(MapDatum datum, const std::string& reason);
  // The change of the map a recording asks for, once; the node sends it to mowbite_map.
  std::optional<MapEdit> TakeMapEdit();
  void OnMapEditResult(bool ok, const std::string& id, const std::string& message);
  // Remote control reaches the wheels only while recording an area, like in OpenMower. After an emergency or a state
  // change it waits for a zero command or a pause of kTeleopQuietS, so a held joystick does not move the mower.
  // now_s is a monotonic clock.
  std::optional<std::pair<double, double>> OnTeleop(double vx, double vz, double now_s);

  RobotStateData RobotState() const;
  std::vector<ActionGroup> Actions() const;
  std::vector<SensorDefinition> Sensors() const;
  std::vector<SensorReading> Readings() const;

  // Events since the last call, oldest first.
  std::vector<Event> TakeEvents();
  // True once after the enabled flags of the actions changed.
  bool TakeActionsChanged();
  // Every change of the emergency reason since the last call, for the log line the app reads.
  std::vector<ReasonChange> TakeReasonChanges();
  // True once after the logic left a state that drives: the wheels get a zero command at once instead of waiting for
  // the cmd_vel watchdog of the driver.
  bool TakeStopRequest();
  // Lines for the log since the last call.
  std::vector<std::string> TakeNotices();
  bool Emergency() const;
  const AreaRecorder& recorder() const {
    return recorder_;
  }

 private:
  enum class State { kIdle, kAreaRecording };

  void SetState(State state);
  // Moves the events of the recorder into the events of the logic, at the current position.
  void PullRecorder();
  void AddEvent(const std::string& type, json details);
  void UpdateEmergency();
  int EmergencyReason() const;
  static const char* StateName(State state);

  LogicConfig config_;
  HardwareSnapshot hw_;
  State state_ = State::kIdle;
  bool emergency_ = false;
  // Without telemetry the mower counts as stopped until the driver reports.
  int reason_ = kEmergencyTimeoutInputs;
  // OpenMower logs the first reason as a change from none.
  int logged_reason_ = 0;
  std::vector<ReasonChange> reason_changes_;
  std::optional<PoseSnapshot> pose_;
  double last_x_ = 0;
  double last_y_ = 0;
  bool hw_seen_ = false;
  bool actions_changed_ = true;
  bool stop_requested_ = false;
  bool teleop_armed_ = false;
  double last_teleop_s_ = -std::numeric_limits<double>::infinity();
  std::vector<Event> events_;
  AreaRecorder recorder_;
  bool map_writable_ = false;
  std::string map_reason_ = "The map status is not known yet";
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_LOGIC_CORE_H
