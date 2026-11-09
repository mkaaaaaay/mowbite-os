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
  ExposureSetting next;
  if (light >= static_cast<double>(limits.exposure_max) * limits.gain_min) {
    next.exposure = limits.exposure_max;
    next.gain = static_cast<int>(std::clamp(std::lround(light / limits.exposure_max),
                                            static_cast<long>(limits.gain_min), static_cast<long>(limits.gain_max)));
  } else {
    next.gain = limits.gain_min;
    next.exposure = static_cast<int>(std::clamp(std::lround(light / limits.gain_min),
                                                static_cast<long>(limits.exposure_min),
                                                static_cast<long>(limits.exposure_max)));
  }
  return next;
}

}  // namespace tron_vision
