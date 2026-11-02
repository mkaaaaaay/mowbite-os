#ifndef MOWBITE_BRIDGE_EVENT_BUILDER_H
#define MOWBITE_BRIDGE_EVENT_BUILDER_H

#include <nlohmann/json.hpp>
#include <string>

namespace mowbite_bridge {

// An event for events/json: id, t, type, then the details; non-object details and their keys id, t, type are dropped.
nlohmann::ordered_json BuildEvent(const std::string& type, const nlohmann::ordered_json& details, double t);

// Random digits and letters (0-9, A-Z, a-z).
std::string RandomId(std::size_t length = 32);

}  // namespace mowbite_bridge

#endif  // MOWBITE_BRIDGE_EVENT_BUILDER_H
