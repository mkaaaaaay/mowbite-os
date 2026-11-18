#include "tron_vision/stereo_depth.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tron_vision {

namespace {

constexpr double kNoCost = std::numeric_limits<double>::infinity();
constexpr int kMaxPatch = 15;

bool Inside(const EyeLuma& eye, int x0, int y0, int x1, int y1) {
  return x0 >= 0 && y0 >= 0 && x1 < static_cast<int>(eye.width) && y1 < static_cast<int>(eye.height);
}

// The left patch around (u, v) less its mean, and its spread.
struct Patch {
  int half = 0;
  int side = 0;
  double values[kMaxPatch * kMaxPatch] = {};
  double spread = 0.0;
};

bool LeftPatch(const EyeLuma& eye, int u, int v, int half, Patch& patch) {
  if (!Inside(eye, u - half, v - half, u + half, v + half)) return false;
  patch.half = half;
  patch.side = 2 * half + 1;
  double sum = 0.0;
  double squares = 0.0;
  int i = 0;
  for (int dy = -half; dy <= half; dy++) {
    for (int dx = -half; dx <= half; dx++) {
      const double value = eye.At(u + dx, v + dy);
      patch.values[i++] = value;
      sum += value;
      squares += value * value;
    }
  }
  const double n = static_cast<double>(i);
  const double mean = sum / n;
  patch.spread = std::sqrt(std::max(0.0, squares / n - mean * mean));
  for (int k = 0; k < i; k++) patch.values[k] -= mean;
  return true;
}

// Sum of absolute differences against the right patch around (x, y), between pixels linear in both directions, both
// patches less their means.
double Cost(const Patch& left, const EyeLuma& eye, double x, double y) {
  const int h = left.half;
  const int x0 = static_cast<int>(std::floor(x));
  const int y0 = static_cast<int>(std::floor(y));
  if (!Inside(eye, x0 - h, y0 - h, x0 + h + 1, y0 + h + 1)) return kNoCost;
  const double fx = x - x0;
  const double fy = y - y0;
  double right[kMaxPatch * kMaxPatch];
  double sum = 0.0;
  int i = 0;
  for (int dy = -h; dy <= h; dy++) {
    for (int dx = -h; dx <= h; dx++) {
      const double a = eye.At(x0 + dx, y0 + dy);
      const double b = eye.At(x0 + dx + 1, y0 + dy);
      const double c = eye.At(x0 + dx, y0 + dy + 1);
      const double d = eye.At(x0 + dx + 1, y0 + dy + 1);
      const double top = a + fx * (b - a);
      right[i] = top + fy * (c + fx * (d - c) - top);
      sum += right[i++];
    }
  }
  const double mean = sum / i;
  double cost = 0.0;
  for (int k = 0; k < i; k++) cost += std::abs(left.values[k] - (right[k] - mean));
  return cost;
}

}  // namespace

StereoPair PairOf(const uint8_t* yuyv, uint32_t eye_width, uint32_t height, uint32_t line_bytes) {
  StereoPair pair;
  pair.left = {yuyv, eye_width, height, line_bytes};
  pair.right = {yuyv + 2 * static_cast<size_t>(eye_width), eye_width, height, line_bytes};
  return pair;
}

bool MatchPoint(const StereoCalibration& calibration, const StereoPair& pair, const Point2& left_pixel,
                const MatchParams& params, double& depth) {
  const int half = std::clamp(params.half_patch, 1, (kMaxPatch - 1) / 2);
  const int u = static_cast<int>(std::lround(left_pixel.x));
  const int v = static_cast<int>(std::lround(left_pixel.y));
  Patch patch;
  if (!LeftPatch(pair.left, u, v, half, patch) || patch.spread < params.min_contrast) return false;
  Point2 ray;
  if (!ToRay(calibration.left, {static_cast<double>(u), static_cast<double>(v)}, ray)) return false;
  const double per_meter = DisparityAtOneMeter(calibration);
  if (!(per_meter > 0.0) || !(params.min_depth > 0.0) || !(params.max_depth > params.min_depth)) return false;
  // Inverse depth in steps of about one pixel of disparity.
  const double w_min = 1.0 / params.max_depth;
  const double step = 1.0 / per_meter;
  const int steps = static_cast<int>(std::ceil((1.0 / params.min_depth - w_min) / step)) + 1;
  std::vector<double> costs(static_cast<size_t>(steps), kNoCost);
  // The offset from the epipolar line of each step's best match.
  std::vector<int> offsets(static_cast<size_t>(steps), 0);
  for (int i = 0; i < steps; i++) {
    Point2 at;
    if (!ToRightPixel(calibration, ray, 1.0 / (w_min + i * step), at)) continue;
    for (int dy = -params.row_slack; dy <= params.row_slack; dy++) {
      const double c = Cost(patch, pair.right, at.x, at.y + dy);
      if (c < costs[i]) {
        costs[i] = c;
        offsets[i] = dy;
      }
    }
  }
  int best = -1;
  for (int i = 0; i < steps; i++) {
    if (costs[i] < kNoCost && (best < 0 || costs[i] < costs[best])) best = i;
  }
  // A minimum at the end of the range lies beyond it.
  if (best <= 0 || best >= steps - 1) return false;
  double other = kNoCost;
  for (int i = 0; i < steps; i++) {
    if (std::abs(i - best) > 2) other = std::min(other, costs[i]);
  }
  if (!(costs[best] < params.uniqueness * other)) return false;
  // Refined at the offset of the best step.
  Point2 before;
  Point2 after;
  if (!ToRightPixel(calibration, ray, 1.0 / (w_min + (best - 1) * step), before) ||
      !ToRightPixel(calibration, ray, 1.0 / (w_min + (best + 1) * step), after)) {
    return false;
  }
  const double c0 = Cost(patch, pair.right, before.x, before.y + offsets[best]);
  const double c2 = Cost(patch, pair.right, after.x, after.y + offsets[best]);
  const double c1 = costs[best];
  double offset = 0.0;
  const double curve = c0 - 2.0 * c1 + c2;
  if (c0 < kNoCost && c2 < kNoCost && curve > 0.0) offset = std::clamp(0.5 * (c0 - c2) / curve, -0.5, 0.5);
  const double w = w_min + (best + offset) * step;
  if (!(w > 0.0)) return false;
  depth = 1.0 / w;
  return depth >= params.min_depth && depth <= params.max_depth;
}

BoxDepth DepthOfBox(const StereoCalibration& calibration, const StereoPair& pair, const PixelBox& box,
                    const MatchParams& match, const BoxDepthParams& params) {
  BoxDepth out;
  const double w = box.x2 - box.x1;
  const double h = box.y2 - box.y1;
  if (!(w > 0.0 && h > 0.0) || params.columns < 1 || params.rows < 1) return out;
  const double left = box.x1 + 0.1 * w;
  const double right = box.x2 - 0.1 * w;
  const double top = box.y2 - std::clamp(params.lower_share, 0.05, 1.0) * h;
  const double bottom = box.y2 - 0.05 * h;
  std::vector<double> depths;
  for (int r = 0; r < params.rows; r++) {
    const double y = params.rows == 1 ? 0.5 * (top + bottom) : top + (bottom - top) * r / (params.rows - 1);
    for (int c = 0; c < params.columns; c++) {
      const double x = params.columns == 1 ? 0.5 * (left + right) : left + (right - left) * c / (params.columns - 1);
      out.points++;
      double depth = 0.0;
      if (MatchPoint(calibration, pair, {x, y}, match, depth)) depths.push_back(depth);
    }
  }
  out.matched = static_cast<int>(depths.size());
  if (out.matched < params.min_matched || 4 * out.matched < out.points) return out;
  const auto middle = depths.begin() + static_cast<std::ptrdiff_t>(depths.size() / 2);
  std::nth_element(depths.begin(), middle, depths.end());
  out.depth = *middle;
  out.pixel = {0.5 * (box.x1 + box.x2), 0.5 * (top + bottom)};
  out.ok = true;
  return out;
}

double MatchedShare(const StereoCalibration& calibration, const StereoPair& pair, const MatchParams& match,
                    int columns, int rows) {
  if (columns < 1 || rows < 1) return 0.0;
  const double width = pair.left.width;
  const double height = pair.left.height;
  int matched = 0;
  for (int r = 0; r < rows; r++) {
    const double y = height * (0.55 + 0.35 * (rows == 1 ? 0.5 : static_cast<double>(r) / (rows - 1)));
    for (int c = 0; c < columns; c++) {
      const double x = width * (0.1 + 0.8 * (columns == 1 ? 0.5 : static_cast<double>(c) / (columns - 1)));
      double depth = 0.0;
      if (MatchPoint(calibration, pair, {x, y}, match, depth)) matched++;
    }
  }
  return static_cast<double>(matched) / (columns * rows);
}

}  // namespace tron_vision
