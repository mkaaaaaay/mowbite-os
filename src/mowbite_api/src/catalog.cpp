#include <map>

#include "mowbite_api/catalog.h"

namespace mowbite_api {

namespace {

Field Number(std::string name, std::string unit, std::string text, bool required = true) {
  return {std::move(name), Type::kNumber, std::move(unit), required, std::move(text), {}, false, {}};
}

Field Integer(std::string name, std::string text, bool required = true) {
  return {std::move(name), Type::kInteger, "", required, std::move(text), {}, false, {}};
}

Field Text(std::string name, std::string text, bool required = true, std::vector<std::string> values = {}) {
  return {std::move(name), Type::kString, "", required, std::move(text), std::move(values), false, {}};
}

// A switch as OpenMower sends it in robot_state: 0 or 1.
Field Switch(std::string name, std::string text) {
  return {std::move(name), Type::kInteger, "", true, std::move(text), {}, false, {}};
}

}  // namespace

const std::vector<Field>& RobotState() {
  static const std::vector<Field> fields = {
      Number("battery_percentage", "", "Charge of the battery from 0 to 1"),
      Number("gps_percentage", "", "Quality of the position from 0 to 1, 0 without a recent pose or heading"),
      Number("current_action_progress", "", "Always 0, kept for the app of OpenMower"),
      Text("current_state", "State of the logic: IDLE, MOWING, DOCKING, UNDOCKING, AREA_RECORDING and the like"),
      Text("current_sub_state", "What the state does right now, empty for nothing in particular"),
      Integer("current_area", "While mowing the index of the area in the plan, otherwise -1"),
      Integer("current_path", "While mowing the index of the path in the plan of the area, otherwise -1"),
      Integer("current_path_index", "While mowing the index of the pose in the path, otherwise -1"),
      Switch("emergency", "1 while an emergency stops motion and blade"),
      Integer("emergency_reason", "The bits of the emergency (x-emergency-bits), 0 without one"),
      Switch("is_charging", "1 while the mower charges on the docking station"),
      Switch("rain_detected", "1 while the rain sensor reports rain"),
      {"pose",
       Type::kObject,
       "",
       true,
       "The pose of the mower in the map",
       {},
       false,
       {Number("x", "m", "East of the datum of the map"), Number("y", "m", "North of the datum of the map"),
        Number("heading", "rad", "Counter-clockwise from east, -pi to pi"),
        Number("pos_accuracy", "m", "Expected error of the position"),
        Number("heading_accuracy", "rad", "Expected error of the heading"),
        Switch("heading_valid", "1 once the heading is known")}},
  };
  return fields;
}

const std::vector<Field>& Envelope() {
  static const std::vector<Field> fields = {
      Text("id", "Random id of the event, 32 letters and digits"),
      Number("t", "unix s", "When it happened"),
      Text("type", "What happened; a type this version does not know carries only the envelope for sure"),
  };
  return fields;
}

const std::vector<Field>& LogicFields() {
  static const std::vector<Field> fields = {
      Number("x", "m", "Where the mower was, east of the datum of the map"),
      Number("y", "m", "Where the mower was, north of the datum of the map"),
      Text("job_id", "The job of mowing it belongs to", false),
      Text("session_id", "The session of the logic since its start", false),
  };
  return fields;
}

const std::vector<EmergencyBit>& EmergencyBits() {
  static const std::vector<EmergencyBit> bits = {
      {0, "LATCH", "The emergency holds until it is reset"},
      {1, "TIMEOUT_INPUTS", "The inputs of the board did not report in time"},
      {2, "STOP", "A stop button was pressed"},
      {3, "LIFT", "The mower was lifted"},
      {4, "LIFT_MULTIPLE", "More than one lift sensor triggered"},
      {5, "COLLISION", "A bumper hit something"},
      {6, "TIMEOUT_HIGH_LEVEL", "The board heard nothing from the OS in time"},
      {7, "HIGH_LEVEL", "The OS asked for the emergency"},
      {8, "SERVICE_NOT_READY", "A service of the board was not ready"},
      {9, "COLLISION_MULTIPLE", "More than one bumper triggered"},
      {10, "MOWER_RPM_TIMEOUT", "The blade motor did not report its speed in time"},
      {11, "MOWER_RPM_LIMIT", "The blade motor turned outside its limits"},
      {12, "FIRMWARE_INCOMPATIBLE", "The firmware of the board does not fit the driver"},
      {13, "TILT", "The body tilted beyond its limit"},
      {14, "FENCE", "An autonomous drive took the mower outside of the allowed areas"},
      {15, "STUCK", "The wheels do not move the mower as GNSS shows it"},
      {16, "IMU_TIMEOUT", "No IMU sample for the board, whose tilt detection depends on it"},
      {17, "MOWER_NOT_STOPPED", "The blade still turns after it was switched off"},
      {18, "WHEEL_RUNAWAY", "A wheel turned against the drive command"},
      {19, "FOREIGN_HOST", "Another host talks to the board"},
      {20, "DRIVER_STALL", "The input and output of the driver stalled"},
  };
  return bits;
}

const std::vector<Command>& Commands() {
  static const std::vector<Command> commands = {
      {"system.info",
       "Name, version and platform of the OS with the versions of its data and of this interface",
       {},
       {Text("name", "Always mowbite-os"), Text("version", "Version of the OS image"),
        Text("platform", "The platform its driver reports, empty until it did"),
        Integer("schema", "Version of the data in data_dir"),
        Integer("api", "Version of this interface, the x-api of the schema system.api gives")},
       "",
       ""},
      {"system.api",
       "This interface as JSON Schema with its version",
       {},
       {Integer("api", "Version of this interface"),
        {"json_schema", Type::kObject, "", true, "This schema", {}, false, {}}},
       "",
       ""},
      {"docking.start",
       "Drives home from IDLE to the docking station where the OS docks by itself; offered only there",
       {},
       {Text("state", "The state after the start", true, {"DOCKING"})},
       "DRIVE_REFUSED",
       ""},
  };
  return commands;
}

const EventEntry* FindEvent(const std::string& type) {
  static const std::map<std::string, const EventEntry*> index = [] {
    std::map<std::string, const EventEntry*> out;
    for (const auto& event : Events()) out[event.type] = &event;
    return out;
  }();
  const auto found = index.find(type);
  return found == index.end() ? nullptr : found->second;
}

Rating RateReason(const std::string& event, const std::string& code, const std::string& action) {
  const EventEntry* entry = FindEvent(event);
  if (!entry) return {"warning", "user"};
  for (const auto& reason : entry->reasons) {
    if (reason.code == code && (reason.action.empty() || reason.action == action)) {
      return {reason.severity.c_str(), reason.resume.c_str()};
    }
  }
  return {entry->other_severity.c_str(), entry->other_resume.c_str()};
}

std::string EmergencyNames(int bits) {
  std::string names;
  for (const auto& bit : EmergencyBits()) {
    if ((bits & (1 << bit.bit)) == 0) continue;
    if (!names.empty()) names += ", ";
    names += bit.name;
  }
  return names;
}

}  // namespace mowbite_api
