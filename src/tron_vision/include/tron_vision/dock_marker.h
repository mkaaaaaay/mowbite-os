#ifndef TRON_VISION_DOCK_MARKER_H
#define TRON_VISION_DOCK_MARKER_H

#include <array>
#include <cstdint>

#include "tron_vision/lens_model.h"

namespace tron_vision {

// Luma plane, one byte per pixel.
struct GrayView {
  const uint8_t* data = nullptr;
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t stride = 0;
};

// The marker is a black square of 6 x 6 cells on a white field at least 3/4 of a cell wide: one cell of black border
// around 4 x 4 cells that carry the pattern.
struct MarkerParams {
  // The 4 x 4 inner cells row by row from the top left, most significant bit first, white is 1.
  uint16_t pattern = 0;
  // The search runs on copies of the image reduced by 2^level, each finds squares of 18 to about 100 of its pixels.
  uint32_t first_level = 1;
  uint32_t last_level = 4;
  // A pixel counts as dark if it lies under the mean of its surroundings by this share of their standard deviation
  // and by at least this many steps.
  float dark_sigmas = 0.4F;
  float dark_step = 6.0F;
  // Shortest side of the square in pixels of the full image.
  float min_side_px = 40.0F;
  // Largest slant of the sides against the image axes, after the distortion is removed (rad).
  float max_slant = 0.44F;
  // Least difference between white and black, around every cell.
  float min_contrast = 30.0F;
  // White samples at or above this value count as saturated; with more than this share of them there is no result.
  float saturation = 250.0F;
  float max_saturated = 0.2F;
  // Widest transition from black to white at the sides, in cells: beyond it the cells run into each other. No limit
  // in pixels, since a near marker is large and out of focus at once.
  float max_edge_cells = 0.5F;
  // Mean luma of the image above or below which a missing marker is put down to the exposure.
  float bright_mean = 200.0F;
  float dark_mean = 25.0F;
  // Standard deviation added to every corner coordinate for what the scatter of the edge points does not show, in
  // pixels.
  float corner_sigma_floor_px = 0.05F;
};

enum class MarkerResult {
  kFound,
  kNoMarker,
  kClipped,    // the marker is there, but not all of it and its white field is in the image
  kAmbiguous,  // more than one
  kTooDark,
  kTooBright,
  kBlurred,
};

// Corners of the black square in the order top left, top right, bottom right, bottom left; filled with kFound,
// kBlurred and with kTooBright if a saturated marker was read. Side and luma describe the marker also with kClipped.
struct MarkerDetection {
  MarkerResult result = MarkerResult::kNoMarker;
  // As seen in the image, in pixels.
  std::array<Point2, 4> corners_px{};
  // The same as directions, the lens distortion removed.
  std::array<Point2, 4> corners{};
  // Covariance of the directions (x and y of each corner in turn, 8 x 8 row by row), from the scatter of the edge
  // points about the lines of the sides; corners that share a side move together.
  std::array<double, 64> corner_covariance{};
  // Root of its mean variance, in pixels.
  double corner_sigma_px = 0.0;
  // Mean side length in pixels.
  double side_px = 0.0;
  // Mean luma of the white and the black samples and the share of saturated white samples.
  double white = 0.0;
  double black = 0.0;
  double saturated = 0.0;
  // Width of the transition from black to white at the sides, in pixels.
  double edge_px = 0.0;
  // Mean luma of the whole image and its share of saturated pixels.
  double image_mean = 0.0;
  double image_saturated = 0.0;
};

// Finds the one marker with the given pattern, upright only and without a single wrong cell. Corners come from
// straight lines through the edge points of each side, fitted after the lens distortion is removed from the points.
MarkerDetection DetectMarker(const GrayView& image, const LensModel& lens, const MarkerParams& params);

}  // namespace tron_vision

#endif  // TRON_VISION_DOCK_MARKER_H
