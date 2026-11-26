#ifndef MOWBITE_BRIDGE_COMPAT_H
#define MOWBITE_BRIDGE_COMPAT_H

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

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

// The params of a call as its node takes them: the sensor_ids of sensors.history as own ids, the names of params.set
// under /logic; other calls and params of another shape as they are, the node answers those.
Json OwnParams(const std::string& method, Json params);

// Whether the answer of a call goes out in other words than its node's: sensors.history and params.set with
// openmower_names.
bool ShowsAnswer(const std::string& method, bool openmower);

// Such an answer with the ids and names shown; an answer of another shape as it is.
Json ShownAnswer(const std::string& method, Json result);

// actions/json from the actions of each node, each with its own id. With openmower_names every action goes under
// the id of OpenMower, sorted by its group there, and the two of manual mowing OpenMower lists are in, always off;
// an action without an id of OpenMower is left out.
std::string ShownActionsText(const std::map<std::string, std::vector<ActionItem>>& nodes, bool openmower);

// The own id of what a client sent on action; nothing for what no node takes. An id of OpenMower becomes the own
// one in both modes. With openmower_names an action of its groups idle, area_recording, undocking and docking
// without a command goes on as it is: the logic counts it as a press that is refused.
std::optional<std::string> OwnAction(const std::string& sent, bool openmower);

// A bridge/params message as params/json shows it: with openmower_names the settings of the logic under
// /mower_logic.
std::string ShownParams(std::string_view message, bool openmower);

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
