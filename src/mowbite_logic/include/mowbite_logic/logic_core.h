#ifndef MOWBITE_LOGIC_LOGIC_CORE_H
#define MOWBITE_LOGIC_LOGIC_CORE_H

#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace mowbite_logic {

using json = nlohmann::ordered_json;

// What the logic needs from mcu/status.
struct McuSnapshot {
  bool telemetry_ok = false;
  bool emergency_latched = false;
  bool lift = false;
  bool stop = false;
  bool bumper = false;
  bool rain = false;
  bool battery_ok = false;
  float battery_voltage = 0;
  // Positive while discharging.
  float battery_current = 0;
  uint8_t battery_percentage = 0;
  bool battery_dock = false;
  struct Motor {
    bool ok = false;
    int16_t rpm = 0;
    float current = 0;
    int8_t temperature = 0;
  };
  Motor cutter;
  Motor left;
  Motor right;
};

struct LogicConfig {
  // 6S Li-ion; empty is an assumption until the discharge curve of the Tron is measured.
  double battery_empty_voltage = 19.2;
  double battery_full_voltage = 25.2;
  // The MCU reports no charge voltage, dock contact stands in for it.
  double dock_charge_voltage = 28.0;
};

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

struct Event {
  std::string type;
  json details;
};

enum class ActionResult { kUnknown, kRejected, kDone, kReleaseEmergency };

// Bits of the reason in an EMERGENCY event, as the app reads them (EmergencyReason of OpenMower).
constexpr int kEmergencyLatch = 1 << 0;
constexpr int kEmergencyTimeoutInputs = 1 << 1;
constexpr int kEmergencyStop = 1 << 2;
constexpr int kEmergencyLift = 1 << 3;
constexpr int kEmergencyCollision = 1 << 5;

// Mower states and actions of stage 1: IDLE and AREA_RECORDING (remote control only), emergency and sensors.
class LogicCore {
 public:
  explicit LogicCore(const LogicConfig& config);

  void OnMcuStatus(const McuSnapshot& mcu);
  // mcu/status stopped coming.
  void OnMcuTimeout();
  ActionResult OnAction(const std::string& action_id);
  // Remote control reaches the wheels only while recording an area, like in OpenMower.
  std::optional<std::pair<double, double>> OnTeleop(double vx, double vz) const;

  RobotStateData RobotState() const;
  std::vector<ActionGroup> Actions() const;
  std::vector<SensorDefinition> Sensors() const;
  std::vector<SensorReading> Readings() const;

  // Events since the last call, oldest first.
  std::vector<Event> TakeEvents();
  // True once after the enabled flags of the actions changed.
  bool TakeActionsChanged();
  bool Emergency() const;

 private:
  enum class State { kIdle, kAreaRecording };

  void SetState(State state);
  void UpdateEmergency();
  int EmergencyReason() const;
  static const char* StateName(State state);

  LogicConfig config_;
  McuSnapshot mcu_;
  State state_ = State::kIdle;
  bool emergency_ = false;
  bool mcu_seen_ = false;
  bool actions_changed_ = true;
  std::vector<Event> events_;
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_LOGIC_CORE_H
