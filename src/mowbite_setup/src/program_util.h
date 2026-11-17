// Pieces program.cpp, motion.cpp and ground.cpp share.
#ifndef MOWBITE_SETUP_PROGRAM_UTIL_H
#define MOWBITE_SETUP_PROGRAM_UTIL_H

#include <cmath>
#include <cstdint>
#include <string>
#include <utility>

#include "mowbite_api/setup.h"
#include "mowbite_setup/program.h"

namespace mowbite_setup::util {

// A source counts as there with a message this young (s); a running step gives up after kNoData without one.
constexpr double kFresh = 1.0;
constexpr double kNoData = 3.0;
// A step that moves the wheels: the logic takes SETUP within kEnterWait, the wheels turn within kStartWait of the
// first command (s).
constexpr double kEnterWait = 2.0;
constexpr double kStartWait = 3.0;

inline uint32_t LatchBit() {
  static const uint32_t bit = EmergencyBit("LATCH");
  return bit;
}

// The group of a role, empty for a text that is no role.
inline std::string GroupOf(const std::string& role) {
  const auto* known = mowbite_api::FindSetupName(mowbite_api::SetupRoles(), role);
  return known ? known->group : "";
}

inline double Rounded(double value, double scale) {
  return std::round(value * scale) / scale;
}

// A value of values[]: bounds and ok are null where there is none or it is not judged yet.
inline json Value(const std::string& id, json value, const std::string& unit, json min, json max, json ok,
                  json texts) {
  return {{"id", id},   {"value", std::move(value)}, {"unit", unit}, {"min", std::move(min)},
          {"max", std::move(max)}, {"ok", std::move(ok)},       {"texts", std::move(texts)}};
}

// A value a check of the catalog measures, by its id, with the unit and the texts from there.
inline json Measured(const char* check, const std::string& id, json value, json min, json max, json ok) {
  std::string unit;
  json texts = json::object();
  for (const auto& measure : mowbite_api::FindSetupCheck(check)->values) {
    if (measure.id != id) continue;
    unit = measure.unit;
    texts = mowbite_api::Texts(measure.texts);
  }
  return Value(id, std::move(value), unit, std::move(min), std::move(max), std::move(ok), std::move(texts));
}

inline Refusal Refuse(std::string reason, std::string message, json data = json::object()) {
  return {std::move(reason), std::move(message), std::move(data), ""};
}

inline Refusal BadParam(std::string field, std::string message) {
  return {"", std::move(message), json::object(), std::move(field)};
}

}  // namespace mowbite_setup::util

#endif  // MOWBITE_SETUP_PROGRAM_UTIL_H
