#ifndef MOWBITE_BRIDGE_APP_JSON_H
#define MOWBITE_BRIDGE_APP_JSON_H

#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "mowbite_bridge/json_rpc.h"

namespace mowbite_bridge {

// The texts the app reads on MQTT, in the field order it knows.

struct Teleop {
  double vx = 0.0;
  double vz = 0.0;
};

// teleop is a BSON document with the numbers vx and vz; anything else, or more than 1 kB, gives nothing.
std::optional<Teleop> ReadTeleop(std::string_view payload);

struct RobotStateView {
  double battery_percentage = 0.0;
  double gps_percentage = 0.0;
  std::string current_state;
  std::string current_sub_state;
  long long current_area = -1;
  long long current_path = -1;
  long long current_path_index = -1;
  long long task_index = -1;
  long long task_count = -1;
  std::string pause_reason;
  std::string pause_kind;
  bool emergency = false;
  long long emergency_reason = 0;
  bool is_charging = false;
  bool rain_detected = false;
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double pos_accuracy = 0.0;
  double heading_accuracy = 0.0;
  bool heading_valid = false;
};

// robot_state/json: switches as 0/1, current_action_progress always 0.0.
std::string RobotStateText(const RobotStateView& state);

struct PoseAttributes {
  std::string job_id;
  std::string session_id;
  bool blades = false;
};

// position/json.
std::string PositionText(double x, double y, double heading, const PoseAttributes& attributes);

// What a sensor value means; distances count as unknown for the app.
enum class Meaning { kUnknown, kVoltage, kCurrent, kTemperature, kRevolutions };

const char* MeaningName(Meaning meaning);

struct SensorDescription {
  std::string id;
  std::string name;
  bool is_text = false;
  Meaning meaning = Meaning::kUnknown;
  std::string unit;
  // A range needs both ends.
  std::optional<double> min;
  std::optional<double> max;
  std::optional<double> critical_low;
  std::optional<double> critical_high;
};

// One entry of sensor_infos/json.
Json SensorInfo(const SensorDescription& sensor);

// sensor_infos/json: the entries sorted by id.
std::string SensorInfosText(const std::map<std::string, SensorDescription>& sensors);

// sensors/<id>/data of a number: six decimals.
std::string SensorNumberText(double value);

struct SensorReading {
  Json value;  // number or text
  double stamp = 0.0;
};

// sensors.list: sensor_infos/json entries with value and stamp; params {"ids": [...]} select, other shapes throw.
Json SensorList(const std::map<std::string, SensorDescription>& sensors,
                const std::map<std::string, SensorReading>& readings, const Json& params);

struct ActionItem {
  std::string id;
  std::string name;
  bool enabled = false;
  std::string reason = "";
};

// actions/json: the groups sorted by prefix, each in the order of its node, as "<prefix>/<id>"; reason only on an
// action that is off and has one.
std::string ActionsText(const std::map<std::string, std::vector<ActionItem>>& groups);

// capabilities/json, indented by two spaces.
std::string CapabilitiesText();

// version/json; an empty version is "UNKNOWN VERSION".
std::string VersionText(const std::string& version);

// system.info; stop_release and blade_height are null where the platform offers no release with STOP or no height,
// update_path where it is empty.
Json SystemInfo(const std::string& version, const std::string& update_path, const std::string& platform,
                int mqtt_version,
                const Json& stop_release = Json(), const Json& blade_height = Json());

// stop_release of system.info from the times of hw/capabilities: null without a free time.
Json StopRelease(double free_s, double hold_s, double max_hold_s);

// blade_height of system.info from hw/capabilities: null without blade_height_control.
Json BladeHeight(bool control, int min_mm, int max_mm, int default_mm);

// The schema of data_dir this image writes, from data_schema.json of the package (the start of the stack reads the same
// file).
int DataSchema();

}  // namespace mowbite_bridge

#endif  // MOWBITE_BRIDGE_APP_JSON_H
