#ifndef MOWBITE_BRIDGE_COMPAT_H
#define MOWBITE_BRIDGE_COMPAT_H

#include <cstdint>
#include <string>

#include "mowbite_bridge/app_json.h"

namespace mowbite_bridge {

// The names the app of OpenMower knows, made from the own names of the OS when the bridge runs with
// openmower_names; the table of sensors is in the catalog (SensorEntry::openmower_id).

// The sensor as sensor_infos/json shows it: key is always its own id, id and name those of OpenMower where it has some.
SensorDescription ShownSensor(SensorDescription own, bool openmower);

// The id of sensors/<id>/data.
std::string ShownSensorId(const std::string& own_id, bool openmower);

// The value as sent: charge_state in the words of OpenMower (Not Charging, Fast Charge, Done).
Json ShownSensorValue(const std::string& own_id, Json value, bool openmower);

// The own id of an id a client sends; an id of OpenMower or an unknown one as it is.
std::string OwnSensorId(const std::string& id);

// The params of a call as its node takes them: the sensor_ids of sensors.history as own ids; other calls and params of
// another shape as they are, the node answers those.
Json OwnParams(const std::string& method, Json params);

// Whether the answer of a call goes out in other words than its node's: sensors.history with openmower_names.
bool ShowsAnswer(const std::string& method, bool openmower);

// Such an answer with the ids and names shown; an answer of another shape as it is.
Json ShownAnswer(Json result);

struct ReasonLine {
  bool warn = false;
  std::string text;
};

// The log line of a change of the reasons of the emergency (bits as in robot_state). The app of OpenMower shows the
// reasons of an active emergency from the line "Emergency reason changed from: '<names>' to: '<names>'".
ReasonLine EmergencyReasonLine(uint32_t before, uint32_t after, bool openmower);

// /ll/board for params/json with openmower_names: the app rates the battery by it, any value but YardForce makes it
// show battery_percentage.
std::string BoardParams();

}  // namespace mowbite_bridge

#endif  // MOWBITE_BRIDGE_COMPAT_H
