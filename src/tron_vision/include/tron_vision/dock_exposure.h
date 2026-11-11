#ifndef TRON_VISION_DOCK_EXPOSURE_H
#define TRON_VISION_DOCK_EXPOSURE_H

#include "tron_vision/dock_marker.h"
#include "tron_vision/exposure.h"

namespace tron_vision {

struct DockExposureLimits {
  // Of the camera controls; the largest exposure is the bound against motion blur.
  ExposureRange range;
  // The white of the marker is held here, below the saturation.
  float white_target = 170.0F;
  float white_band = 25.0F;
  // With more than this share of saturated white the true level is unknown and the light goes down a full step.
  float saturated_share = 0.1F;
  // Largest factor of one step, either way.
  float max_step = 2.0F;
  // Images a new setting takes to show.
  int settle_frames = 3;
  // Images without a marker before the search begins.
  int search_after = 5;
  // With this share of saturated pixels in the image the search goes down first: the marker itself may be the
  // saturated part of an otherwise dark image.
  float search_saturated = 0.0005F;
};

struct DockExposureState {
  ExposureSetting setting;
  int since_change = 0;
  int without_marker = 0;
  bool search_down = true;
};

// State after an image taken with `now` gave `seen`. With a marker the product of exposure and gain follows the white
// of the marker alone: it is lit from behind, so the rest of the image says nothing about it. Without a marker the
// product walks through its whole range, down and up again; it starts downwards if the image is too bright or has
// saturated parts, upwards if it is too dark.
// A marker that is cut off or not the only one leaves the setting alone.
DockExposureState NextDockExposure(const DockExposureState& now, const MarkerDetection& seen,
                                   const DockExposureLimits& limits);

}  // namespace tron_vision

#endif  // TRON_VISION_DOCK_EXPOSURE_H
