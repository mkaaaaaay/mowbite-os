#ifndef MOWBITE_BRIDGE_EVENT_BUILDER_H
#define MOWBITE_BRIDGE_EVENT_BUILDER_H

#include <nlohmann/json.hpp>
#include <string>

namespace mowbite_bridge {

// An event for events/json: id, t (seconds), type, then the details in their own order. Details that are not an
// object, and their keys id, t and type, are left out.
nlohmann::ordered_json BuildEvent(const std::string& type, const nlohmann::ordered_json& details, double t);

// Random digits and letters (0-9, A-Z, a-z).
std::string RandomId(std::size_t length = 32);

}  // namespace mowbite_bridge

#endif  // MOWBITE_BRIDGE_EVENT_BUILDER_H
