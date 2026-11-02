#include "mowbite_bridge/event_builder.h"

#include <random>

namespace mowbite_bridge {

nlohmann::ordered_json BuildEvent(const std::string& type, const nlohmann::ordered_json& details, double t) {
  nlohmann::ordered_json event = {{"id", RandomId()}, {"t", t}, {"type", type}};
  if (!details.is_object()) return event;
  for (const auto& [key, value] : details.items()) {
    if (key != "id" && key != "t" && key != "type") event[key] = value;
  }
  return event;
}

std::string RandomId(std::size_t length) {
  static constexpr char kSigns[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
  thread_local std::mt19937_64 engine{std::random_device{}()};
  std::uniform_int_distribution<std::size_t> pick(0, sizeof(kSigns) - 2);
  std::string id(length, ' ');
  for (char& c : id) c = kSigns[pick(engine)];
  return id;
}

}  // namespace mowbite_bridge
