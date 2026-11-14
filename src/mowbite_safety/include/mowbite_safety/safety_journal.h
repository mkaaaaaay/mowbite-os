#ifndef MOWBITE_SAFETY_SAFETY_JOURNAL_H
#define MOWBITE_SAFETY_SAFETY_JOURNAL_H

#include <mowbite_journal/journaled.h>

#include "mowbite_safety/safety_filter.h"

// The journal of the SafetyFilter (stream "safety"). A new field of a struct below needs its place in its line, the
// size check makes the build say so.

namespace mowbite_safety {

inline void to_json(nlohmann::json& j, const SafetyConfig& c) {
  j = nlohmann::json{{"tilt_limit", c.tilt_limit},
                     {"tilt_time", c.tilt_time},
                     {"tilt_release", c.tilt_release},
                     {"imu_timeout", c.imu_timeout},
                     {"hw_timeout", c.hw_timeout},
                     {"pose_timeout", c.pose_timeout},
                     {"max_position_accuracy", c.max_position_accuracy},
                     {"dock_max_position_accuracy", c.dock_max_position_accuracy},
                     {"dock_accuracy_per_m", c.dock_accuracy_per_m},
                     {"fence_margin", c.fence_margin},
                     {"obstacle_margin", c.obstacle_margin},
                     {"station_ahead", c.station_ahead},
                     {"station_behind", c.station_behind},
                     {"station_half_width", c.station_half_width},
                     {"arm_quiet", c.arm_quiet},
                     {"owner_timeout", c.owner_timeout}};
}
// Fields that came later take the value the journals from before ran with; all else is required.
inline void from_json(const nlohmann::json& j, SafetyConfig& c) {
  j.at("tilt_limit").get_to(c.tilt_limit);
  j.at("tilt_time").get_to(c.tilt_time);
  j.at("tilt_release").get_to(c.tilt_release);
  j.at("imu_timeout").get_to(c.imu_timeout);
  j.at("hw_timeout").get_to(c.hw_timeout);
  j.at("pose_timeout").get_to(c.pose_timeout);
  j.at("max_position_accuracy").get_to(c.max_position_accuracy);
  c.dock_max_position_accuracy = j.value("dock_max_position_accuracy", SafetyConfig{}.dock_max_position_accuracy);
  c.dock_accuracy_per_m = j.value("dock_accuracy_per_m", 0.0);
  j.at("fence_margin").get_to(c.fence_margin);
  j.at("obstacle_margin").get_to(c.obstacle_margin);
  j.at("station_ahead").get_to(c.station_ahead);
  j.at("station_behind").get_to(c.station_behind);
  j.at("station_half_width").get_to(c.station_half_width);
  j.at("arm_quiet").get_to(c.arm_quiet);
  j.at("owner_timeout").get_to(c.owner_timeout);
}
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Twist2, linear, angular)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Decision, out, refusal, conflict)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LatchRequest, bits, reason, again)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Release, allowed, message, code, value, limit)

static_assert(sizeof(SafetyConfig) == 128);
static_assert(sizeof(Twist2) == 16);
static_assert(sizeof(Decision) == 64);
static_assert(sizeof(LatchRequest) == 48);
static_assert(sizeof(Release) == 88);

}  // namespace mowbite_safety

template <>
struct mowbite_journal::CoreTraits<mowbite_safety::SafetyFilter> {
  using F = mowbite_safety::SafetyFilter;
  static constexpr const char* kName = "safety";
  using Config = mowbite_safety::SafetyConfig;
  MOWBITE_JOURNAL_METHODS(&F::OnHardware, &F::OnCapabilities, &F::OnTilt, &F::OnPose, &F::SetMap, &F::SetMapFits,
                          &F::SetMaxPositionAccuracy, &F::OnCommand, &F::Tick, &F::TakeLatchRequest, &F::OnRelease,
                          &F::OnRestartHold, &F::OnDockPose);
  // The latched reason bits of S2.
  static uint32_t View(const F& core) {
    return core.Latched();
  }
};

#endif  // MOWBITE_SAFETY_SAFETY_JOURNAL_H
