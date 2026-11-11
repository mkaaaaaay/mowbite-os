#include "tron_vision/exposure.h"

#include <algorithm>
#include <cmath>

namespace tron_vision {

ExposureSetting NextExposure(const ExposureSetting& now, float mean_y, const ExposureLimits& limits) {
  if (std::fabs(mean_y - limits.target_y) <= limits.band) return now;
  const float signal = std::max(mean_y - limits.black, 1.0F);
  const float factor = std::clamp((limits.target_y - limits.black) / signal, 1.0F / limits.max_step, limits.max_step);
  const double light = static_cast<double>(std::clamp(now.exposure, limits.exposure_min, limits.exposure_max)) *
                       std::clamp(now.gain, limits.gain_min, limits.gain_max) * factor;
  return SplitLight(light, {limits.exposure_min, limits.exposure_max, limits.gain_min, limits.gain_max});
}

ExposureSetting SplitLight(double light, const ExposureRange& range) {
  ExposureSetting next;
  if (light >= static_cast<double>(range.exposure_max) * range.gain_min) {
    next.exposure = range.exposure_max;
    next.gain = static_cast<int>(std::clamp(std::lround(light / range.exposure_max),
                                            static_cast<long>(range.gain_min), static_cast<long>(range.gain_max)));
  } else {
    next.gain = range.gain_min;
    next.exposure = static_cast<int>(std::clamp(std::lround(light / range.gain_min),
                                                static_cast<long>(range.exposure_min),
                                                static_cast<long>(range.exposure_max)));
  }
  return next;
}

}  // namespace tron_vision
