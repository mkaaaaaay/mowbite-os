#ifndef MOWBITE_SAFETY_SAFETY_JOURNAL_H
#define MOWBITE_SAFETY_SAFETY_JOURNAL_H

#include <mowbite_journal/journaled.h>

#include "mowbite_safety/safety_filter.h"

// The journal of the SafetyFilter (stream "safety"). A new field of a struct below needs its place in its line, the
// size check makes the build say so.

namespace mowbite_safety {

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SafetyConfig, tilt_limit, tilt_time, tilt_release, imu_timeout, hw_timeout,
                                   pose_timeout, max_position_accuracy, fence_margin, obstacle_margin, station_ahead,
                                   station_behind, station_half_width, arm_quiet, owner_timeout)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Twist2, linear, angular)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Decision, out, refusal, conflict)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LatchRequest, bits, reason, again)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Release, allowed, message, code, value, limit)

static_assert(sizeof(SafetyConfig) == 112);
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
                          &F::SetMaxPositionAccuracy, &F::OnCommand, &F::Tick, &F::TakeLatchRequest, &F::OnRelease);
  // The latched reason bits of S2.
  static uint32_t View(const F& core) {
    return core.Latched();
  }
};

#endif  // MOWBITE_SAFETY_SAFETY_JOURNAL_H
