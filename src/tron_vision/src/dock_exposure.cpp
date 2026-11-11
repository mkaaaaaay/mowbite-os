#include "tron_vision/dock_exposure.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tron_vision {

namespace {

int CountUp(int n) {
  return n < std::numeric_limits<int>::max() ? n + 1 : n;
}

bool Same(const ExposureSetting& a, const ExposureSetting& b) {
  return a.exposure == b.exposure && a.gain == b.gain;
}

}  // namespace

DockExposureState NextDockExposure(const DockExposureState& now, const MarkerDetection& seen,
                                   const DockExposureLimits& limits) {
  DockExposureState next = now;
  next.since_change = CountUp(now.since_change);
  const ExposureRange& range = limits.range;
  if (range.exposure_min <= 0 || range.gain_min <= 0 || range.exposure_max < range.exposure_min ||
      range.gain_max < range.gain_min || !(limits.max_step > 1.0F)) {
    return next;
  }
  const double light = static_cast<double>(std::clamp(now.setting.exposure, range.exposure_min, range.exposure_max)) *
                       std::clamp(now.setting.gain, range.gain_min, range.gain_max);
  const auto apply = [&](double wanted) {
    next.setting = SplitLight(wanted, range);
    if (!Same(next.setting, now.setting)) next.since_change = 0;
  };
  const bool settled = next.since_change >= limits.settle_frames;
  const double down = 1.0 / limits.max_step;

  const bool read = seen.side_px > 0.0 && (seen.result == MarkerResult::kFound ||
                                           seen.result == MarkerResult::kBlurred ||
                                           seen.result == MarkerResult::kTooBright);
  if (read) {
    next.without_marker = 0;
    if (!settled) return next;
    if (seen.saturated > limits.saturated_share) {
      apply(light * down);
    } else if (std::fabs(seen.white - limits.white_target) > limits.white_band) {
      apply(light * std::clamp(limits.white_target / std::max(seen.white, 1.0), down,
                               static_cast<double>(limits.max_step)));
    }
    return next;
  }
  if (seen.result == MarkerResult::kClipped || seen.result == MarkerResult::kAmbiguous) {
    next.without_marker = 0;
    return next;
  }

  next.without_marker = CountUp(now.without_marker);
  if (next.without_marker < limits.search_after) return next;
  if (now.without_marker < limits.search_after) {
    if (seen.result == MarkerResult::kTooBright || seen.image_saturated >= limits.search_saturated) {
      next.search_down = true;
    } else if (seen.result == MarkerResult::kTooDark) {
      next.search_down = false;
    }
  }
  if (!settled) return next;
  apply(light * (next.search_down ? down : limits.max_step));
  if (Same(next.setting, now.setting)) {
    // At the end of the range: back the other way.
    next.search_down = !next.search_down;
    apply(light * (next.search_down ? down : limits.max_step));
  }
  return next;
}

}  // namespace tron_vision
