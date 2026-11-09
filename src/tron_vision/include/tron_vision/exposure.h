#ifndef TRON_VISION_EXPOSURE_H
#define TRON_VISION_EXPOSURE_H

namespace tron_vision {

// Values of the sensor controls exposure (lines) and analogue_gain (64 = 1x).
struct ExposureSetting {
  int exposure = 1120;
  int gain = 64;
};

struct ExposureLimits {
  int exposure_min = 4;
  int exposure_max = 1120;
  int gain_min = 64;
  int gain_max = 8192;
  float target_y = 100.0F;
  // No change while the mean luma is this close to the target.
  float band = 12.0F;
  // Luma of black without light; the sensor output is not black-level corrected.
  float black = 16.0F;
  // Largest factor of one step, either way.
  float max_step = 2.0F;
};

// Next setting after a frame taken with `now` had the mean luma `mean_y`. Luma above black grows about linearly with
// exposure times gain, so the product is scaled; the exposure stays at its maximum and only the gain moves until the
// gain is at its minimum, then the exposure goes down (backlight).
ExposureSetting NextExposure(const ExposureSetting& now, float mean_y, const ExposureLimits& limits);

}  // namespace tron_vision

#endif  // TRON_VISION_EXPOSURE_H
