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
  // Largest RMS distance between the corners and the fitted square: these pixels plus this share of the side. With
  // the roll the Tron's markers fit within 0.02 to 0.23 px from 0.28 to 1.2 m (2026-11-14 and -15); a corner off by a
  // twentieth of the side stays out.
  double fit_error_px = 0.5;
  double fit_error_share = 0.004;
  // Largest turn of the square in its face against the camera (rad): a station or a mower that leans, the ground. On
  // the Tron 1 to 3 degrees were measured (2026-11-14 and -15); well below the 45 degrees where a quarter turn of the
  // pattern would look alike.
  double max_roll = 30.0 * 3.14159265358979323846 / 180.0;
  // The yaw counts only if its mirror image about the line of sight fits worse by this many standard deviations
  // of the corners.
  double mirror_sigmas = 4.0;
  // Up to measured_yaw_range m from the camera the yaw of one image has the standard deviation measured_yaw_sigma (rad):
  // on the Tron (2026-11-13 and -15, 0.11 to 0.85 m) the yaw was off by a fixed 3 to 4 degrees per place while the
  // images agreed within 0.3 degrees; the corners gave 10 to 24 degrees, swinging with the light. Beyond it the
  // corners give it. 0 m leaves it to the corners everywhere.
  double measured_yaw_sigma = 4.0 * 3.14159265358979323846 / 180.0;
  double measured_yaw_range = 0.85;
};

// Marker center and the direction its face looks in, in base_link (m, rad).
struct MarkerPose {
  bool valid = false;
  bool yaw_valid = false;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
  // Turn of the square in its face against the camera, positive from its right side up (rad); for the record.
  double roll = 0.0;
  // Of x, y, yaw, row by row, from the covariance of the corners, widened if the square fits worse than that
  // explains; the yaw entries are 0 without yaw_valid. Within measured_yaw_range the yaw has measured_yaw_sigma, with
  // the correlations of the corners.
  std::array<double, 9> covariance{};
  double fit_error_px = 0.0;
  // For the record: the factor the fit widened the covariance by, and the yaw spread (rad) the corners gave.
  double widen = 0.0;
  double corner_yaw_sigma = 0.0;
};

// Fits a square of the given side that hangs upright (its face contains the z axis of base_link) to the directions of
// its corners: top left, top right, bottom right, bottom left as the camera sees them. Five unknowns (x, y, height,
// yaw, and the roll of the square in its face up to max_roll, which a leaning station or mower gives); the pitch comes
// from the mount alone. `corner_covariance` is that of the directions (x and y of each
// corner in turn, 8 x 8 row by row); `focal_px` turns the fit error into pixels.
MarkerPose FitMarkerPose(const std::array<Point2, 4>& corners, const std::array<double, 64>& corner_covariance,
                         double focal_px, const CameraMount& mount, double marker_side, const PoseLimits& limits);

}  // namespace tron_vision

#endif  // TRON_VISION_DOCK_POSE_H
