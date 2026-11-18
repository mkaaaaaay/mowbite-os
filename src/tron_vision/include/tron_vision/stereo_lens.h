#ifndef TRON_VISION_STEREO_LENS_H
#define TRON_VISION_STEREO_LENS_H

#include <cstdint>
#include <string>

#include "tron_vision/lens_model.h"

namespace tron_vision {

// Fisheye lens with the equidistant model: a ray at the angle theta to the axis lands at
// theta * (1 + k1 theta^2 + k2 theta^4 + k3 theta^6 + k4 theta^8) from the center, times fx and fy.
struct FisheyeLens {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  double k1 = 0.0;
  double k2 = 0.0;
  double k3 = 0.0;
  double k4 = 0.0;
};

// Where a ray (x/z, y/z) lands in the image, in pixels.
Point2 ToPixel(const FisheyeLens& lens, const Point2& ray);
// The ray of a pixel; false where the model cannot be inverted (beyond 89 deg or where the distortion folds back, which
// a calibration without boards there may do in the corners).
bool ToRay(const FisheyeLens& lens, const Point2& pixel, Point2& ray);

// Both eyes of the stereo camera, each width x height, and the pose of the right eye: a point X in the frame of the
// left eye is rotation * X + translation in that of the right one (m).
struct StereoCalibration {
  uint32_t width = 0;
  uint32_t height = 0;
  FisheyeLens left;
  FisheyeLens right;
  double rotation[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  double translation[3] = {0.0, 0.0, 0.0};
};

// The calibration file is text with one "name values" per line, '#' starts a comment:
//   size <width> <height>
//   left <fx> <fy> <cx> <cy> <k1> <k2> <k3> <k4>
//   right <fx> <fy> <cx> <cy> <k1> <k2> <k3> <k4>
//   rotation <9 numbers, row by row>
//   translation <x> <y> <z>
// False with the reason when a line is missing, malformed or the values do not fit a stereo camera.
bool ReadStereoCalibration(const std::string& path, StereoCalibration& calibration, std::string& error);
bool CheckStereoCalibration(const StereoCalibration& calibration, std::string& error);

// Where the point at `depth` (m, along the view of the left eye) on a ray of the left eye lands in the right image.
// False where it lies behind the right eye.
bool ToRightPixel(const StereoCalibration& calibration, const Point2& left_ray, double depth, Point2& pixel);

// Focal length times baseline of the pair (px m): the disparity of a point at 1 m.
double DisparityAtOneMeter(const StereoCalibration& calibration);

}  // namespace tron_vision

#endif  // TRON_VISION_STEREO_LENS_H
