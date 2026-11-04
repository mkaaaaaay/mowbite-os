#include "mowbite_bridge/health.h"

#include <cctype>
#include <cerrno>
#include <cstdlib>

#include <nlohmann/json.hpp>

namespace mowbite_bridge {

std::optional<BrokerAddress> BrokerFromEnvironment(const char* host, const char* port, std::string* error) {
  BrokerAddress address{host && *host ? host : "127.0.0.1", 1883};
  if (port && *port) {
    char* end = nullptr;
    errno = 0;
    const long value = std::strtol(port, &end, 10);
    if (!std::isdigit(static_cast<unsigned char>(*port)) || errno != 0 || *end != '\0' || value < 1 || value > 65535) {
      *error = std::string("MOWBITE_MQTT_PORT is not a port: ") + port;
      return std::nullopt;
    }
    address.port = static_cast<int>(value);
  }
  return address;
}

std::string RobotStateProblem(const std::string& payload) {
  const auto state = nlohmann::json::parse(payload, nullptr, false);
  if (state.is_discarded() || !state.is_object()) return "robot_state/json is no JSON object";
  const auto current = state.find("current_state");
  if (current == state.end() || !current->is_string()) return "robot_state/json has no current_state";
  if (current->get<std::string>().empty()) return "robot_state/json has an empty current_state";
  return "";
}

std::string HaltLine(const std::string& note) {
  return note.substr(0, note.find('\n'));
}

}  // namespace mowbite_bridge
