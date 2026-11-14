#ifndef MOWBITE_LOCALIZATION_DOCK_ESTIMATOR_JOURNAL_H
#define MOWBITE_LOCALIZATION_DOCK_ESTIMATOR_JOURNAL_H

#include <mowbite_journal/journaled.h>

#include "mowbite_localization/dock_estimator.h"
#include "mowbite_localization/estimator_journal.h"

// The journal of the DockEstimator (stream "dock_localization"). A new field of a struct below needs its place in its
// line, the size check makes the build say so.

namespace mowbite_localization {

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DockConfig, noise, marker_x, marker_y, marker_yaw, max_age, start_images,
                                   hint_hold, hint_sigma, position_chi2, yaw_chi2, min_outlier, min_outlier_yaw,
                                   outlier_reset, good_way, good_time, blind_distance, final_way, final_time)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Motion, time, distance, turn, gyro_time, still)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MarkerImage, time, x, y, yaw, yaw_valid, covariance)

static_assert(sizeof(DockConfig) == 152);
static_assert(sizeof(Motion) == 40);
static_assert(sizeof(MarkerImage) == 112);

}  // namespace mowbite_localization

template <>
struct mowbite_journal::CoreTraits<mowbite_localization::DockEstimator> {
  using D = mowbite_localization::DockEstimator;
  static constexpr const char* kName = "dock_localization";
  using Config = mowbite_localization::DockConfig;
  MOWBITE_JOURNAL_METHODS(&D::AddMotion, &D::AddMarker, &D::SetHint, &D::SetDockedHeading);
  static bool View(const D& core) { return core.has_estimate(); }
};
MOWBITE_JOURNAL_POD(mowbite_localization::Motion);
MOWBITE_JOURNAL_POD(mowbite_localization::MarkerImage);

#endif  // MOWBITE_LOCALIZATION_DOCK_ESTIMATOR_JOURNAL_H
