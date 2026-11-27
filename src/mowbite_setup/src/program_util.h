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
// The driver answers hw/silence and hw/end within this time (s).
constexpr double kServiceWait = 2.0;
// The emergency of the trigger stands within this time once the wheels or the blade have to stand still no longer (s).
constexpr double kLatchWait = 2.0;
// The release of the program ends that emergency within this time (s).
constexpr double kReleaseWithin = 10.0;
// A blade step of the firmware measurement with the blade off the mower: the logic keeps the blade order this long
// after an emergency of the inputs of the board alone, so that the board has to stop the blade (s, decision of the
// owner 27.11.; the driver holds no longer either).
constexpr double kBladeInputHold = 1.0;
// Wheels or a blade that stood this long already at the trigger did not stop by it: the pass measured nothing (s).
constexpr double kStoodBefore = 0.3;

// Whether what stands since since (below 0: it turns) stood kStoodBefore at the trigger.
inline bool StoodBefore(double since, double trigger) {
  return since >= 0 && trigger - since >= kStoodBefore;
}

inline uint32_t LatchBit() {
  static const uint32_t bit = EmergencyBit("LATCH");
  return bit;
}

inline uint32_t FirmwareBit() {
  static const uint32_t bit = EmergencyBit("FIRMWARE_INCOMPATIBLE");
  return bit;
}

// The emergencies an input group brings; without a group those of all three.
inline uint32_t InputBits(const std::string& group = "") {
  static const uint32_t stop = EmergencyBit("STOP");
  static const uint32_t lift = EmergencyBit("LIFT") | EmergencyBit("LIFT_MULTIPLE");
  static const uint32_t bumper = EmergencyBit("COLLISION") | EmergencyBit("COLLISION_MULTIPLE");
  if (group == "stop") return stop;
  if (group == "lift") return lift;
  if (group == "bumper") return bumper;
  return stop | lift | bumper;
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
