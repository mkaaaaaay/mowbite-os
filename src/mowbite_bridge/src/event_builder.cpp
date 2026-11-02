#include "mowbite_bridge/event_builder.h"

#include <cstdint>
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
  // Base 62: ten digits fit in one 64-bit word; words at or above the last whole multiple of 62^10 are drawn again.
  static constexpr char kDigits[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
  static constexpr std::uint64_t kSpan = 839299365868340224ULL;  // 62^10
  static constexpr std::uint64_t kLimit = UINT64_MAX - UINT64_MAX % kSpan;
  thread_local std::mt19937_64 source{std::random_device{}()};
  std::string id;
  id.reserve(length);
  while (id.size() < length) {
    std::uint64_t word = source();
    if (word >= kLimit) continue;
    for (int i = 0; i < 10 && id.size() < length; ++i, word /= 62) id.push_back(kDigits[word % 62]);
  }
  return id;
}

}  // namespace mowbite_bridge
