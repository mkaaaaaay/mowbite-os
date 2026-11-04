#ifndef MOWBITE_BRIDGE_HEALTH_H
#define MOWBITE_BRIDGE_HEALTH_H

#include <optional>
#include <string>

namespace mowbite_bridge {

// The parts of mowbite_health (HEALTHCHECK of the image) that need no broker.

struct BrokerAddress {
  std::string host;
  int port = 0;
};

// MOWBITE_MQTT_HOST and MOWBITE_MQTT_PORT as given (null when unset), 127.0.0.1:1883 by default; nullopt and why for a
// port that is not a number from 1 to 65535.
std::optional<BrokerAddress> BrokerFromEnvironment(const char* host, const char* port, std::string* error);

// Empty for a robot_state/json of a running stack (an object with current_state as text), else why not.
std::string RobotStateProblem(const std::string& payload);

// The first line of the halt note that tron.launch.py leaves for mowbite_health, empty when it has none.
std::string HaltLine(const std::string& note);

}  // namespace mowbite_bridge

#endif  // MOWBITE_BRIDGE_HEALTH_H
