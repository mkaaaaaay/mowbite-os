#ifndef TRON_VISION_DOCK_POSE_H
#define TRON_VISION_DOCK_POSE_H

#include <array>

#include "tron_vision/lens_model.h"

namespace tron_vision {

// Where the camera sits on the mower: the lens in base_link (m) and its view, turned from the x axis of base_link by
// yaw about z, then pitch about y (positive looks down), then roll about the view (rad).
struct CameraMount {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
  double pitch = 0.0;
  double roll = 0.0;
};

struct PoseLimits {
  // Largest RMS distance between the corners and the fitted square: these pixels plus this share of the side.
  double fit_error_px = 0.5;
  double fit_error_share = 0.01;
  // The yaw counts only if its mirror image about the line of sight fits worse by this many standard deviations
  // of the corners.
  double mirror_sigmas = 4.0;
};

// Marker center and the direction its face looks in, in base_link (m, rad).
struct MarkerPose {
  bool valid = false;
  bool yaw_valid = false;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
  // Of x, y, yaw, row by row, from the covariance of the corners, widened if the square fits worse than that
  // explains; the yaw entries are 0 without yaw_valid.
  std::array<double, 9> covariance{};
  double fit_error_px = 0.0;
};

// Fits a square of the given side that hangs upright (its face contains the z axis of base_link) to the directions of
// its corners: top left, top right, bottom right, bottom left as the camera sees them. Four unknowns (x, y, height,
// yaw); pitch and roll come from the mount alone. `corner_covariance` is that of the directions (x and y of each
// corner in turn, 8 x 8 row by row); `focal_px` turns the fit error into pixels.
MarkerPose FitMarkerPose(const std::array<Point2, 4>& corners, const std::array<double, 64>& corner_covariance,
                         double focal_px, const CameraMount& mount, double marker_side, const PoseLimits& limits);

}  // namespace tron_vision

#endif  // TRON_VISION_DOCK_POSE_H
