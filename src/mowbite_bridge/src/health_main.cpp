// HEALTHCHECK of the image, without ROS: healthy while the broker of the OS passes on robot_state/json from the logic.
// Exit 0, or 1 with one line why (docker inspect shows it in Health.Log).
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <mqtt/client.h>
#include <nlohmann/json.hpp>

#include "mowbite_bridge/health.h"

namespace {

constexpr auto kConnectTimeout = std::chrono::seconds(2);
// robot_state/json comes once a second.
constexpr auto kStateWait = std::chrono::seconds(3);
// Where tron.launch.py leaves its halt line; MOWBITE_HALT_FILE for both.
constexpr char kHaltFile[] = "/tmp/mowbite_os_halt";

std::string ReadFile(const std::string& path) {
  std::ifstream in(path);
  if (!in) return "";
  std::ostringstream text;
  text << in.rdbuf();
  return text.str();
}

}  // namespace

int main() {
  using namespace mowbite_bridge;
  const char* halt_file = std::getenv("MOWBITE_HALT_FILE");  // NOLINT(concurrency-mt-unsafe): one thread at start
  const std::string halt = HaltLine(ReadFile(halt_file && *halt_file ? halt_file : kHaltFile));
  if (!halt.empty()) {
    std::cout << halt << "\n";
    return 1;
  }
  std::string error;
  // NOLINTNEXTLINE(concurrency-mt-unsafe): read once at start, before the MQTT client starts its threads
  const auto broker = BrokerFromEnvironment(std::getenv("MOWBITE_MQTT_HOST"), std::getenv("MOWBITE_MQTT_PORT"), &error);
  if (!broker) {
    std::cout << "BAD_CONFIG " << error << "\n";
    return 1;
  }
  const std::string where = broker->host + ":" + std::to_string(broker->port);
  try {
    mqtt::client client("tcp://" + where, "mowbite_health_" + std::to_string(::getpid()));
    mqtt::connect_options options;
    options.set_clean_session(true);
    options.set_connect_timeout(kConnectTimeout);
    client.start_consuming();
    client.connect(options);
    client.subscribe("robot_state/json", 0);
    std::string problem = "no robot_state/json within " + std::to_string(kStateWait.count()) + " s";
    const auto deadline = std::chrono::steady_clock::now() + kStateWait;
    for (auto now = std::chrono::steady_clock::now(); now < deadline; now = std::chrono::steady_clock::now()) {
      mqtt::const_message_ptr message;
      if (!client.try_consume_message_for(&message, deadline - now) || !message) continue;
      const std::string payload = message->to_string();
      const std::string why = RobotStateProblem(payload);
      if (why.empty()) {
        client.disconnect();
        std::cout << "OK " << nlohmann::json::parse(payload)["current_state"].get<std::string>() << "\n";
        return 0;
      }
      problem = why;
    }
    client.disconnect();
    std::cout << "NO_ROBOT_STATE " << problem << " (broker " << where << ")" << "\n";
  } catch (const mqtt::exception& e) {
    std::cout << "MQTT_UNREACHABLE " << where << ": " << e.what() << "\n";
  }
  return 1;
}
