#ifndef TRON_VISION_STEREO_DEPTH_H
#define TRON_VISION_STEREO_DEPTH_H

#include <cstdint>
#include <vector>

#include "tron_vision/stereo_lens.h"

namespace tron_vision {

// The luma of one eye inside a YUYV frame: pixel (x, y) at data[y * stride + 2 * x].
struct EyeLuma {
  const uint8_t* data = nullptr;
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t stride = 0;
  uint8_t At(int x, int y) const {
    return data[static_cast<size_t>(y) * stride + 2 * static_cast<size_t>(x)];
  }
};

struct StereoPair {
  EyeLuma left;
  EyeLuma right;
};

// Both eyes side by side in each line of `line_bytes`, each eye_width pixels wide.
StereoPair PairOf(const uint8_t* yuyv, uint32_t eye_width, uint32_t height, uint32_t line_bytes);

struct MatchParams {
  // Only depths within this range count (m, along the view of the left eye).
  double min_depth = 0.3;
  double max_depth = 3.0;
  // The patch compared is (2 * half_patch + 1) pixels square.
  int half_patch = 4;
  // The right point may lie this many lines off where the calibration puts it.
  int row_slack = 2;
  // Patches with less spread of luma than this have too little texture to match.
  double min_contrast = 4.0;
  // The best match has to cost less than this share of the best one more than two steps away.
  double uniqueness = 0.85;
};

// The depth of a pixel of the left eye from a search along its epipolar line in the right eye, one pixel of disparity
// per step and refined between the steps; false without a clear match within the range.
bool MatchPoint(const StereoCalibration& calibration, const StereoPair& pair, const Point2& left_pixel,
                const MatchParams& params, double& depth);

// Box corners in pixels of the left eye.
struct PixelBox {
  double x1 = 0.0;
  double y1 = 0.0;
  double x2 = 0.0;
  double y2 = 0.0;
};

struct BoxDepth {
  bool ok = false;
  // Median of the matched points (m) and the pixel its ray goes through: the middle of the box across, the middle of
  // the band searched upright.
  double depth = 0.0;
  Point2 pixel;
  int points = 0;
  int matched = 0;
};

// The median depth over a grid in the lower part of a box (lower_share of its height, the outer tenth on each side
// left out); it needs at least min_matched points and a quarter of the grid.
struct BoxDepthParams {
  int columns = 8;
  int rows = 4;
  double lower_share = 0.4;
  int min_matched = 3;
};
BoxDepth DepthOfBox(const StereoCalibration& calibration, const StereoPair& pair, const PixelBox& box,
                    const MatchParams& match, const BoxDepthParams& params);

// The share of a grid in the lower half of the image (where the ground lies within the range) that finds a depth.
double MatchedShare(const StereoCalibration& calibration, const StereoPair& pair, const MatchParams& match,
                    int columns = 8, int rows = 3);

}  // namespace tron_vision

#endif  // TRON_VISION_STEREO_DEPTH_H
