#include "tron_vision/dock_marker.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace tron_vision {

namespace {

constexpr int kCells = 6;
// Half width of the surroundings a pixel is compared with, in pixels of the level.
constexpr int kWindow = 8;
// The value below which a pixel is dark is set once per tile of this many pixels.
constexpr uint32_t kTile = 4;
constexpr uint32_t kMinLevelWidth = 64;
constexpr uint32_t kMinLevelHeight = 48;
// Smallest dark area worth a look, in pixels of the level; the short side allows for a marker seen from the side.
constexpr int kMinBlobLong = 10;
constexpr int kMinBlobShort = 6;
// Share of a side left out at both ends, where the corner rounds the edge.
constexpr double kCornerMargin = 0.12;
constexpr int kMaxSideSamples = 48;
constexpr int kMinSideSamples = 6;
// Samples per side of the quick first pass over a candidate.
constexpr int kQuickSideSamples = 10;
// Edge points this far off their line still belong to it, in pixels; in the quick pass the side may also bend by
// this share of its length under the lens distortion.
constexpr double kLineSlackPx = 0.25;
constexpr double kBendShare = 0.02;
// The edge is searched this far to both sides of the expected place, in cells.
constexpr double kEdgeReach = 0.8;
// Black and white next to an edge are read between these distances from it, in cells.
constexpr double kPlateauNear = 0.35;
constexpr double kPlateauFar = 0.65;
constexpr size_t kMaxProfile = 81;
// Cells and the white field are read at 3 x 3 points this far around their middle, in cells.
constexpr double kCellSpread = 0.22;
constexpr double kFieldSpread = 0.15;
constexpr double kFieldOffset = 0.5;
// A cell is white above this share of the way from black to white and black below the other.
constexpr double kWhiteShare = 0.6;
constexpr double kBlackShare = 0.4;
constexpr uint32_t kStatsGrid = 8;
// Share of the side that must be in the image of a marker the image cuts off.
constexpr double kMinVisible = 0.45;

struct Plane {
  uint32_t width = 0;
  uint32_t height = 0;
  std::vector<uint8_t> px;
};

Plane Copy(const GrayView& image) {
  Plane plane;
  plane.width = image.width;
  plane.height = image.height;
  plane.px.resize(static_cast<size_t>(image.width) * image.height);
  for (uint32_t y = 0; y < image.height; y++) {
    std::copy_n(image.data + static_cast<size_t>(y) * image.stride, image.width,
                plane.px.begin() + static_cast<std::ptrdiff_t>(static_cast<size_t>(y) * image.width));
  }
  return plane;
}

// Half the size, each pixel the mean of 2 x 2.
Plane Halve(const uint8_t* data, uint32_t width, uint32_t height, uint32_t stride) {
  Plane plane;
  plane.width = width / 2;
  plane.height = height / 2;
  plane.px.resize(static_cast<size_t>(plane.width) * plane.height);
  for (uint32_t y = 0; y < plane.height; y++) {
    const uint8_t* top = data + static_cast<size_t>(y) * 2 * stride;
    const uint8_t* bottom = top + stride;
    uint8_t* out = plane.px.data() + static_cast<size_t>(y) * plane.width;
    for (size_t x = 0; x < plane.width; x++) {
      const uint32_t sum = top[2 * x] + top[2 * x + 1] + bottom[2 * x] + bottom[2 * x + 1];
      out[x] = static_cast<uint8_t>((sum + 2) / 4);
    }
  }
  return plane;
}

// Per tile the value below which a pixel counts as dark: the mean of the window around the tile less a share of the
// scatter in that window. The measure follows the local contrast, so it holds in shadow and in a bright image alike.
// 0 where nothing can be dark.
std::vector<uint8_t> DarkLimits(const Plane& plane, const MarkerParams& params, uint32_t cols, uint32_t rows) {
  const auto w = static_cast<int>(plane.width);
  const auto h = static_cast<int>(plane.height);
  const size_t row = static_cast<size_t>(w) + 1;
  std::vector<uint32_t> sums(row * (static_cast<size_t>(h) + 1), 0);
  std::vector<uint64_t> squares(sums.size(), 0);
  for (int y = 0; y < h; y++) {
    uint32_t line = 0;
    uint64_t line_squares = 0;
    const size_t above = static_cast<size_t>(y) * row;
    for (int x = 0; x < w; x++) {
      const uint32_t value = plane.px[static_cast<size_t>(y) * plane.width + static_cast<size_t>(x)];
      line += value;
      line_squares += static_cast<uint64_t>(value) * value;
      sums[above + row + static_cast<size_t>(x) + 1] = sums[above + static_cast<size_t>(x) + 1] + line;
      squares[above + row + static_cast<size_t>(x) + 1] = squares[above + static_cast<size_t>(x) + 1] + line_squares;
    }
  }
  std::vector<uint8_t> limits(static_cast<size_t>(cols) * rows, 0);
  const int half_tile = static_cast<int>(kTile) / 2;
  for (uint32_t ty = 0; ty < rows; ty++) {
    const int y = static_cast<int>(ty * kTile) + half_tile;
    const int top = std::max(y - kWindow, 0);
    const int bottom = std::min(y + kWindow, h);
    const size_t y0 = static_cast<size_t>(top) * row;
    const size_t y1 = static_cast<size_t>(bottom) * row;
    for (uint32_t tx = 0; tx < cols; tx++) {
      const int x = static_cast<int>(tx * kTile) + half_tile;
      const auto x0 = static_cast<size_t>(std::max(x - kWindow, 0));
      const auto x1 = static_cast<size_t>(std::min(x + kWindow, w));
      const auto n = static_cast<double>(x1 - x0) * (bottom - top);
      const double mean = (sums[y1 + x1] - sums[y0 + x1] - sums[y1 + x0] + sums[y0 + x0]) / n;
      const double mean2 =
          static_cast<double>(squares[y1 + x1] - squares[y0 + x1] - squares[y1 + x0] + squares[y0 + x0]) / n;
      const double sigma = std::sqrt(std::max(mean2 - mean * mean, 0.0));
      const double limit = mean - std::max(static_cast<double>(params.dark_step), params.dark_sigmas * sigma);
      limits[static_cast<size_t>(ty) * cols + tx] = static_cast<uint8_t>(std::clamp(std::ceil(limit), 0.0, 255.0));
    }
  }
  return limits;
}

double Distance(const Point2& a, const Point2& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double Cross(const Point2& origin, const Point2& a, const Point2& b) {
  return (a.x - origin.x) * (b.y - origin.y) - (a.y - origin.y) * (b.x - origin.x);
}

double Distance2(const Point2& a, const Point2& b) {
  return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}

// Squared distance of a point from the stretch between a and b.
double SegmentDistance2(const Point2& p, const Point2& a, const Point2& b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double length2 = dx * dx + dy * dy;
  const double t = length2 > 0.0 ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / length2, 0.0, 1.0) : 0.0;
  return Distance2(p, {a.x + t * dx, a.y + t * dy});
}

double QuadArea(const std::array<Point2, 4>& q) {
  return 0.5 * std::fabs(Cross(q[0], q[1], q[2]) + Cross(q[0], q[2], q[3]));
}

// Convex hull of points sorted by y, then x.
std::vector<Point2> Hull(const std::vector<Point2>& sorted) {
  std::vector<Point2> hull;
  if (sorted.size() < 3) return hull;
  const auto add = [&hull](const Point2& p, size_t keep) {
    while (hull.size() > keep && Cross(hull[hull.size() - 2], hull.back(), p) <= 0.0) hull.pop_back();
    hull.push_back(p);
  };
  for (const Point2& p : sorted) add(p, 1);
  const size_t lower = hull.size();
  for (size_t i = sorted.size() - 1; i-- > 0;) add(sorted[i], lower);
  hull.pop_back();
  return hull;
}

// The four-cornered outline of a hull: the two points farthest apart and the farthest point on each side of the line
// between them. False if the hull has parts that outline does not cover.
bool HullQuad(const std::vector<Point2>& hull, std::array<Point2, 4>& quad) {
  if (hull.size() < 4) return false;
  size_t a = 0;
  size_t b = 0;
  double longest2 = 0.0;
  for (size_t i = 0; i < hull.size(); i++) {
    for (size_t j = i + 1; j < hull.size(); j++) {
      const double d2 = Distance2(hull[i], hull[j]);
      if (d2 > longest2) {
        longest2 = d2;
        a = i;
        b = j;
      }
    }
  }
  const double longest = std::sqrt(longest2);
  size_t left = a;
  size_t right = a;
  double most = 0.0;
  double least = 0.0;
  for (size_t i = 0; i < hull.size(); i++) {
    const double side = Cross(hull[a], hull[b], hull[i]) / longest;
    if (side > most) {
      most = side;
      left = i;
    }
    if (side < least) {
      least = side;
      right = i;
    }
  }
  if (most < 1.5 || least > -1.5) return false;
  quad = {hull[a], hull[left], hull[b], hull[right]};
  const double tolerance = 1.0 + 0.03 * longest;
  for (const Point2& p : hull) {
    double nearest2 = longest2;
    for (size_t k = 0; k < 4; k++) nearest2 = std::min(nearest2, SegmentDistance2(p, quad[k], quad[(k + 1) % 4]));
    if (nearest2 > tolerance * tolerance) return false;
  }
  return true;
}

// Top left, top right, bottom right, bottom left for an upright quad.
void SortCorners(std::array<Point2, 4>& quad) {
  Point2 mid;
  for (const Point2& p : quad) {
    mid.x += p.x / 4.0;
    mid.y += p.y / 4.0;
  }
  std::sort(quad.begin(), quad.end(), [&mid](const Point2& p, const Point2& q) {
    return std::atan2(p.y - mid.y, p.x - mid.x) < std::atan2(q.y - mid.y, q.x - mid.x);
  });
}

// Rough shape of an upright square seen from the front or the side, for corners in the order of SortCorners:
// opposite sides alike, the level ones not much longer than the upright ones, none slanted much beyond the limit.
// What the border of the image cuts off may have any proportions.
bool SquareLike(const std::array<Point2, 4>& q, double max_slant, bool cut) {
  const double top = Distance(q[0], q[1]);
  const double right = Distance(q[1], q[2]);
  const double bottom = Distance(q[2], q[3]);
  const double left = Distance(q[3], q[0]);
  if (std::max(top, bottom) > 2.0 * std::min(top, bottom) || std::max(left, right) > 2.0 * std::min(left, right)) {
    return false;
  }
  if (!cut && (top + bottom > 1.5 * (left + right) || left + right > 6.0 * (top + bottom))) return false;
  const double slack = max_slant + 0.25;
  return std::fabs(q[1].y - q[0].y) <= std::tan(slack) * std::fabs(q[1].x - q[0].x) &&
         std::fabs(q[2].y - q[3].y) <= std::tan(slack) * std::fabs(q[2].x - q[3].x) &&
         std::fabs(q[3].x - q[0].x) <= std::tan(slack) * std::fabs(q[3].y - q[0].y) &&
         std::fabs(q[2].x - q[1].x) <= std::tan(slack) * std::fabs(q[2].y - q[1].y);
}

// Reads the image between pixel centers.
class Sampler {
 public:
  explicit Sampler(const GrayView& image)
      : image_(image),
        max_x_(static_cast<double>(image.width) - 1.0),
        max_y_(static_cast<double>(image.height) - 1.0) {}

  bool At(double x, double y, double& value) const {
    if (!(x >= 0.0) || !(y >= 0.0) || x > max_x_ || y > max_y_) return false;
    const auto x0 = std::min(static_cast<uint32_t>(x), image_.width - 2);
    const auto y0 = std::min(static_cast<uint32_t>(y), image_.height - 2);
    const double fx = x - static_cast<double>(x0);
    const double fy = y - static_cast<double>(y0);
    const uint8_t* top = image_.data + static_cast<size_t>(y0) * image_.stride + x0;
    const uint8_t* bottom = top + image_.stride;
    value = (1.0 - fy) * ((1.0 - fx) * top[0] + fx * top[1]) + fy * ((1.0 - fx) * bottom[0] + fx * bottom[1]);
    return true;
  }

 private:
  GrayView image_;
  double max_x_;
  double max_y_;
};

struct EdgePoint {
  Point2 px;
  double width = 0.0;
};

// Where the image goes from dark to bright along `normal` near `at`. Exact: to a fraction of a pixel, the place
// halfway between the levels on both sides, from a line through the samples of the rise. Not exact: the rise nearest
// to `at` alone, good to about half a pixel.
bool FindEdge(const Sampler& image, const Point2& at, const Point2& normal, double cell, double min_contrast,
              bool exact, EdgePoint& edge) {
  std::array<double, kMaxProfile> v{};
  const auto read = [&](double t, double& value) {
    return image.At(at.x + t * normal.x, at.y + t * normal.y, value);
  };
  // Rise nearest to the expected place.
  const double reach = kEdgeReach * cell;
  const int half = std::clamp(static_cast<int>(std::lround(reach / 0.5)), 4, static_cast<int>(kMaxProfile / 2));
  const double step = reach / half;
  const size_t count = 2 * static_cast<size_t>(half) + 1;
  for (size_t k = 0; k < count; k++) {
    if (!read((static_cast<double>(k) - half) * step, v[k])) return false;
  }
  const auto [low, high] = std::minmax_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(count));
  if (*high - *low < min_contrast) return false;
  const double coarse_mid = (*low + *high) / 2.0;
  bool found = false;
  double coarse = 0.0;
  for (size_t k = 0; k + 1 < count; k++) {
    if (v[k] < coarse_mid && v[k + 1] >= coarse_mid) {
      const double t = (static_cast<double>(k) - half + (coarse_mid - v[k]) / (v[k + 1] - v[k])) * step;
      if (!found || std::fabs(t) < std::fabs(coarse)) coarse = t;
      found = true;
    }
  }
  if (!found) return false;
  if (!exact) {
    edge.px = {at.x + coarse * normal.x, at.y + coarse * normal.y};
    return true;
  }

  // Black and white on both sides of it.
  double dark = 0.0;
  double bright = 0.0;
  constexpr int kPlateauSamples = 5;
  for (int k = 0; k < kPlateauSamples; k++) {
    const double offset = (kPlateauNear + (kPlateauFar - kPlateauNear) * k / (kPlateauSamples - 1)) * cell;
    double inner = 0.0;
    double outer = 0.0;
    if (!read(coarse - offset, inner) || !read(coarse + offset, outer)) return false;
    dark += inner / kPlateauSamples;
    bright += outer / kPlateauSamples;
  }
  const double contrast = bright - dark;
  if (contrast < min_contrast) return false;

  // The rise itself in fine steps.
  const double span = kPlateauNear * cell;
  const int fine_half = std::clamp(static_cast<int>(std::lround(span / 0.5)), 2, static_cast<int>(kMaxProfile / 2));
  const double fine_step = span / fine_half;
  const size_t fine_count = 2 * static_cast<size_t>(fine_half) + 1;
  for (size_t k = 0; k < fine_count; k++) {
    if (!read(coarse + (static_cast<double>(k) - fine_half) * fine_step, v[k])) return false;
  }
  const double mid = dark + contrast / 2.0;
  found = false;
  size_t rise = 0;
  for (size_t k = 0; k + 1 < fine_count; k++) {
    if (v[k] < mid && v[k + 1] >= mid) {
      const auto here = std::fabs(static_cast<double>(k) + 0.5 - fine_half);
      if (!found || here < std::fabs(static_cast<double>(rise) + 0.5 - fine_half)) rise = k;
      found = true;
    }
  }
  if (!found) return false;
  size_t first = rise;
  size_t last = rise + 1;
  const double lower = dark + 0.2 * contrast;
  const double upper = dark + 0.8 * contrast;
  while (first > 0 && v[first - 1] >= lower && v[first - 1] <= v[first]) first--;
  while (last + 1 < fine_count && v[last + 1] <= upper && v[last + 1] >= v[last]) last++;
  const auto n = static_cast<double>(last - first + 1);
  double mean_t = 0.0;
  double mean_v = 0.0;
  for (size_t k = first; k <= last; k++) {
    mean_t += static_cast<double>(k) / n;
    mean_v += v[k] / n;
  }
  double stt = 0.0;
  double stv = 0.0;
  for (size_t k = first; k <= last; k++) {
    stt += (static_cast<double>(k) - mean_t) * (static_cast<double>(k) - mean_t);
    stv += (static_cast<double>(k) - mean_t) * (v[k] - mean_v);
  }
  if (!(stv > 0.0)) return false;
  const double slope = stv / stt / fine_step;
  const double t = coarse + (mean_t - fine_half) * fine_step + (mid - mean_v) / slope;
  edge.px = {at.x + t * normal.x, at.y + t * normal.y};
  edge.width = contrast / slope;
  return true;
}

struct Line {
  // Mean of its points.
  Point2 point;
  Point2 normal;
  double rms = 0.0;
  size_t count = 0;
  // Sum of the squared distances of its points from their mean, along the line.
  double spread = 0.0;
};

Line LineThrough(const std::vector<Point2>& points, const std::vector<uint8_t>& use) {
  Line line;
  for (size_t i = 0; i < points.size(); i++) {
    if (use[i] == 0) continue;
    line.point.x += points[i].x;
    line.point.y += points[i].y;
    line.count++;
  }
  const auto n = static_cast<double>(std::max<size_t>(line.count, 1));
  line.point.x /= n;
  line.point.y /= n;
  double sxx = 0.0;
  double sxy = 0.0;
  double syy = 0.0;
  for (size_t i = 0; i < points.size(); i++) {
    if (use[i] == 0) continue;
    const double dx = points[i].x - line.point.x;
    const double dy = points[i].y - line.point.y;
    sxx += dx * dx;
    sxy += dx * dy;
    syy += dy * dy;
  }
  const double direction = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
  line.normal = {-std::sin(direction), std::cos(direction)};
  double sum = 0.0;
  for (size_t i = 0; i < points.size(); i++) {
    if (use[i] == 0) continue;
    const double dx = points[i].x - line.point.x;
    const double dy = points[i].y - line.point.y;
    const double r = line.normal.x * dx + line.normal.y * dy;
    const double along = line.normal.y * dx - line.normal.x * dy;
    sum += r * r;
    line.spread += along * along;
  }
  line.rms = std::sqrt(sum / n);
  return line;
}

// Straight line through the points by their distances at right angles; points far off the others are left out.
// `slack` is the distance always tolerated. False with too few points left of the `wanted` ones.
bool FitLine(const std::vector<Point2>& points, size_t wanted, double slack, Line& line) {
  if (points.size() < kMinSideSamples) return false;
  std::vector<uint8_t> use(points.size(), 1);
  std::vector<double> distance(points.size());
  for (int round = 0; round < 2; round++) {
    line = LineThrough(points, use);
    for (size_t i = 0; i < points.size(); i++) {
      distance[i] =
          std::fabs(line.normal.x * (points[i].x - line.point.x) + line.normal.y * (points[i].y - line.point.y));
    }
    std::vector<double> sorted = distance;
    std::nth_element(sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(sorted.size() / 2), sorted.end());
    const double limit = std::max(3.0 * 1.4826 * sorted[sorted.size() / 2], slack);
    for (size_t i = 0; i < points.size(); i++) use[i] = distance[i] <= limit ? 1 : 0;
  }
  line = LineThrough(points, use);
  return line.count >= kMinSideSamples && line.count * 10 >= wanted * 6;
}

bool Intersect(const Line& a, const Line& b, Point2& p) {
  const double det = a.normal.x * b.normal.y - a.normal.y * b.normal.x;
  if (std::fabs(det) < 1e-6) return false;
  const double ca = a.normal.x * a.point.x + a.normal.y * a.point.y;
  const double cb = b.normal.x * b.point.x + b.normal.y * b.point.y;
  p = {(ca * b.normal.y - cb * a.normal.y) / det, (cb * a.normal.x - ca * b.normal.x) / det};
  return true;
}

struct Square {
  std::array<Point2, 4> px{};
  std::array<Point2, 4> rays{};
  std::array<double, 64> covariance{};
  double edge_px = 0.0;
};

// Covariance of the corner coordinates (x and y of each corner in turn) from the scatter of the edge points about
// their lines: every line is uncertain in its place and in its direction, and each takes two corners along.
std::array<double, 64> CornerCovariance(const std::array<Line, 4>& lines, const std::array<Point2, 4>& corners,
                                        double floor) {
  // Per corner coordinate the share of each line's shift and turn.
  std::array<std::array<double, 8>, 8> share{};
  std::array<double, 8> variance{};
  for (size_t k = 0; k < 4; k++) {
    const auto n = static_cast<double>(lines[k].count);
    const double point_variance = lines[k].rms * lines[k].rms * n / (n - 2.0);
    variance[2 * k] = point_variance / n;
    variance[2 * k + 1] = point_variance / lines[k].spread;
  }
  for (size_t j = 0; j < 4; j++) {
    const std::array<size_t, 2> sides = {(j + 3) % 4, j};
    const Line& a = lines[sides[0]];
    const Line& b = lines[sides[1]];
    const double det = a.normal.x * b.normal.y - a.normal.y * b.normal.x;
    // Columns of the inverse of the two normals as rows: the move of the corner per shift of either line there.
    const std::array<Point2, 2> move = {Point2{b.normal.y / det, -b.normal.x / det},
                                        Point2{-a.normal.y / det, a.normal.x / det}};
    for (size_t i = 0; i < 2; i++) {
      const Line& line = lines[sides[i]];
      const double along =
          line.normal.y * (corners[j].x - line.point.x) - line.normal.x * (corners[j].y - line.point.y);
      share[2 * j][2 * sides[i]] = move[i].x;
      share[2 * j][2 * sides[i] + 1] = move[i].x * along;
      share[2 * j + 1][2 * sides[i]] = move[i].y;
      share[2 * j + 1][2 * sides[i] + 1] = move[i].y * along;
    }
  }
  std::array<double, 64> covariance{};
  for (size_t r = 0; r < 8; r++) {
    for (size_t c = 0; c < 8; c++) {
      double sum = r == c ? floor * floor : 0.0;
      for (size_t k = 0; k < 8; k++) sum += share[r][k] * variance[k] * share[c][k];
      covariance[r * 8 + c] = sum;
    }
  }
  return covariance;
}

double MeanSide(const std::array<Point2, 4>& q) {
  double sum = 0.0;
  for (size_t k = 0; k < 4; k++) sum += Distance(q[k], q[(k + 1) % 4]) / 4.0;
  return sum;
}

// Lines through the edge points of the four sides, starting from corners good to a few pixels, and the corners
// where they cross. With `lens` the points are freed from the distortion first and the lines and `square.rays` are
// exact; without it the edge points are rough and the lines run through the pixels as they are, which is quick and
// good to about a hundredth of the side, and only `square.px` is set.
bool LineUp(const Sampler& image, const LensModel* lens, const MarkerParams& params, int max_samples,
            Square& square, std::array<Line, 4>& lines) {
  std::vector<Point2> points;
  std::vector<double> widths;
  const double side = MeanSide(square.px);
  for (size_t k = 0; k < 4; k++) {
    const Point2& a = square.px[k];
    const Point2& b = square.px[(k + 1) % 4];
    const double length = Distance(a, b);
    if (length < 1.0) return false;
    // Corners run clockwise in the image, so this normal points away from the square.
    const Point2 normal{(b.y - a.y) / length, -(b.x - a.x) / length};
    const double cell = (Distance(square.px[(k + 3) % 4], a) + Distance(b, square.px[(k + 2) % 4])) / (2.0 * kCells);
    const int wanted =
        std::clamp(static_cast<int>(length * (1.0 - 2.0 * kCornerMargin)), kMinSideSamples, max_samples);
    points.clear();
    for (int i = 0; i < wanted; i++) {
      const double f = kCornerMargin + (1.0 - 2.0 * kCornerMargin) * (i + 0.5) / wanted;
      const Point2 at{a.x + f * (b.x - a.x), a.y + f * (b.y - a.y)};
      EdgePoint edge;
      if (!FindEdge(image, at, normal, cell, params.min_contrast, lens != nullptr, edge)) continue;
      Point2 point = edge.px;
      if (lens != nullptr && !ToRay(*lens, edge.px, point)) continue;
      points.push_back(point);
      widths.push_back(edge.width);
    }
    const double slack = lens != nullptr ? kLineSlackPx / lens->fx : kLineSlackPx + kBendShare * side;
    if (!FitLine(points, static_cast<size_t>(wanted), slack, lines[k])) return false;
  }
  for (size_t k = 0; k < 4; k++) {
    Point2 corner;
    if (!Intersect(lines[(k + 3) % 4], lines[k], corner)) return false;
    if (lens != nullptr) {
      square.rays[k] = corner;
      square.px[k] = ToPixel(*lens, corner);
    } else {
      square.px[k] = corner;
    }
  }
  std::nth_element(widths.begin(), widths.begin() + static_cast<std::ptrdiff_t>(widths.size() / 2), widths.end());
  square.edge_px = widths[widths.size() / 2];
  return true;
}

// Upright and convex, judged on the directions of the corners.
bool Upright(const std::array<Point2, 4>& q, double max_slant) {
  for (size_t k = 0; k < 4; k++) {
    if (Cross(q[k], q[(k + 1) % 4], q[(k + 2) % 4]) <= 0.0) return false;
  }
  const auto slant = [](const Point2& from, const Point2& to, bool level) {
    const double along = level ? to.x - from.x : to.y - from.y;
    const double across = level ? to.y - from.y : to.x - from.x;
    return along > 0.0 ? std::fabs(std::atan2(across, along)) : 4.0;
  };
  return slant(q[0], q[1], true) <= max_slant && slant(q[3], q[2], true) <= max_slant &&
         slant(q[0], q[3], false) <= max_slant && slant(q[1], q[2], false) <= max_slant;
}

// Maps cell coordinates (0 to 6 across and down the square) to the image.
class Grid {
 public:
  Grid(const Sampler& image, const LensModel& lens, const std::array<Point2, 4>& rays) : image_(image), lens_(lens) {
    // Projective map of the unit square onto the four corners.
    const double sx = rays[0].x - rays[1].x + rays[2].x - rays[3].x;
    const double sy = rays[0].y - rays[1].y + rays[2].y - rays[3].y;
    const double dx1 = rays[1].x - rays[2].x;
    const double dx2 = rays[3].x - rays[2].x;
    const double dy1 = rays[1].y - rays[2].y;
    const double dy2 = rays[3].y - rays[2].y;
    const double det = dx1 * dy2 - dx2 * dy1;
    g_ = (sx * dy2 - dx2 * sy) / det;
    h_ = (dx1 * sy - sx * dy1) / det;
    a_ = rays[1].x - rays[0].x + g_ * rays[1].x;
    b_ = rays[3].x - rays[0].x + h_ * rays[3].x;
    c_ = rays[0].x;
    d_ = rays[1].y - rays[0].y + g_ * rays[1].y;
    e_ = rays[3].y - rays[0].y + h_ * rays[3].y;
    f_ = rays[0].y;
  }

  // Mean of 3 x 3 points `spread` cells around the place; false if one of them is outside the image.
  bool Read(double u, double v, double spread, double& value) const {
    double sum = 0.0;
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        const double s = (u + i * spread) / kCells;
        const double t = (v + j * spread) / kCells;
        const double w = g_ * s + h_ * t + 1.0;
        const Point2 px = ToPixel(lens_, {(a_ * s + b_ * t + c_) / w, (d_ * s + e_ * t + f_) / w});
        double one = 0.0;
        if (!image_.At(px.x, px.y, one)) return false;
        sum += one;
      }
    }
    value = sum / 9.0;
    return true;
  }

 private:
  const Sampler& image_;
  const LensModel& lens_;
  double a_ = 0.0;
  double b_ = 0.0;
  double c_ = 0.0;
  double d_ = 0.0;
  double e_ = 0.0;
  double f_ = 0.0;
  double g_ = 0.0;
  double h_ = 0.0;
};

struct Reading {
  bool match = false;
  double white = 0.0;
  double black = 0.0;
  double saturated = 0.0;
};

// Compares all 36 cells and the white field around them with the pattern. Black and white are judged against the
// border cells and the field nearest to each cell, so a shadow across the marker does not change the reading.
Reading ReadPattern(const Grid& grid, const MarkerParams& params) {
  Reading reading;
  std::array<std::array<double, kCells>, kCells> cell{};
  // Field next to the top, right, bottom and left side, and at the four corners.
  std::array<std::array<double, kCells>, 4> field{};
  std::array<double, 4> corner{};
  const double far = kCells + kFieldOffset;
  for (int i = 0; i < kCells; i++) {
    for (int j = 0; j < kCells; j++) {
      if (!grid.Read(j + 0.5, i + 0.5, kCellSpread, cell[i][j])) return reading;
    }
    if (!grid.Read(i + 0.5, -kFieldOffset, kFieldSpread, field[0][i]) ||
        !grid.Read(far, i + 0.5, kFieldSpread, field[1][i]) || !grid.Read(i + 0.5, far, kFieldSpread, field[2][i]) ||
        !grid.Read(-kFieldOffset, i + 0.5, kFieldSpread, field[3][i])) {
      return reading;
    }
  }
  if (!grid.Read(-kFieldOffset, -kFieldOffset, kFieldSpread, corner[0]) ||
      !grid.Read(far, -kFieldOffset, kFieldSpread, corner[1]) || !grid.Read(far, far, kFieldSpread, corner[2]) ||
      !grid.Read(-kFieldOffset, far, kFieldSpread, corner[3])) {
    return reading;
  }

  double white_sum = 0.0;
  double black_sum = 0.0;
  int white_count = 0;
  int black_count = 0;
  int saturated = 0;
  const auto white = [&](double value) {
    white_sum += value;
    white_count++;
    if (value >= params.saturation) saturated++;
  };
  const auto black = [&](double value) {
    black_sum += value;
    black_count++;
  };
  const int last = kCells - 1;
  // The border is black against the field next to it.
  for (int k = 0; k < kCells; k++) {
    if (field[0][k] - cell[0][k] < params.min_contrast || field[1][k] - cell[k][last] < params.min_contrast ||
        field[2][k] - cell[last][k] < params.min_contrast || field[3][k] - cell[k][0] < params.min_contrast) {
      return reading;
    }
    for (const auto& side : field) white(side[k]);
    black(cell[0][k]);
    black(cell[last][k]);
    if (k > 0 && k < last) {
      black(cell[k][0]);
      black(cell[k][last]);
    }
  }
  if (corner[0] - cell[0][0] < params.min_contrast || corner[1] - cell[0][last] < params.min_contrast ||
      corner[2] - cell[last][last] < params.min_contrast || corner[3] - cell[last][0] < params.min_contrast) {
    return reading;
  }
  for (const double value : corner) white(value);
  // The inner cells against black and white of the border and the field in their row and column, nearer ones more.
  for (int i = 1; i < last; i++) {
    for (int j = 1; j < last; j++) {
      const double w_top = 1.0 / i;
      const double w_bottom = 1.0 / (last - i);
      const double w_left = 1.0 / j;
      const double w_right = 1.0 / (last - j);
      const double weights = w_top + w_bottom + w_left + w_right;
      const double dark =
          (w_top * cell[0][j] + w_bottom * cell[last][j] + w_left * cell[i][0] + w_right * cell[i][last]) / weights;
      const double bright =
          (w_top * field[0][j] + w_right * field[1][i] + w_bottom * field[2][j] + w_left * field[3][i]) / weights;
      const double contrast = bright - dark;
      if (contrast < params.min_contrast) return reading;
      const int bit = (i - 1) * 4 + (j - 1);
      const bool expect_white = ((params.pattern >> (15 - bit)) & 1U) != 0;
      if (expect_white) {
        if (cell[i][j] < dark + kWhiteShare * contrast) return reading;
        white(cell[i][j]);
      } else {
        if (cell[i][j] > dark + kBlackShare * contrast) return reading;
        black(cell[i][j]);
      }
    }
  }
  reading.match = true;
  reading.white = white_sum / white_count;
  reading.black = black_sum / black_count;
  reading.saturated = static_cast<double>(saturated) / white_count;
  return reading;
}

// Compares the cells that are in the image with the pattern, for a marker the image cuts off: black and white come
// from the visible border and the visible white cells. Needs half of the cells, among them a full side of the border.
Reading ReadVisible(const Grid& grid, const MarkerParams& params) {
  Reading reading;
  std::array<std::array<double, kCells>, kCells> cell{};
  std::array<std::array<bool, kCells>, kCells> seen{};
  const int last = kCells - 1;
  const auto white_expected = [&params](int i, int j) {
    if (i == 0 || j == 0 || i == kCells - 1 || j == kCells - 1) return false;
    return ((params.pattern >> (15 - ((i - 1) * 4 + (j - 1)))) & 1U) != 0;
  };
  double white = 0.0;
  double black = 0.0;
  int white_count = 0;
  int black_count = 0;
  int border_count = 0;
  int saturated = 0;
  for (int i = 0; i < kCells; i++) {
    for (int j = 0; j < kCells; j++) {
      seen[i][j] = grid.Read(j + 0.5, i + 0.5, kCellSpread, cell[i][j]);
      if (!seen[i][j]) continue;
      if (white_expected(i, j)) {
        white += cell[i][j];
        white_count++;
        if (cell[i][j] >= params.saturation) saturated++;
      } else {
        black += cell[i][j];
        black_count++;
        if (i == 0 || j == 0 || i == last || j == last) border_count++;
      }
    }
  }
  if (white_count + black_count < kCells * kCells / 2 || border_count < kCells || white_count < 2) return reading;
  white /= white_count;
  black /= black_count;
  const double contrast = white - black;
  if (contrast < params.min_contrast) return reading;
  for (int i = 0; i < kCells; i++) {
    for (int j = 0; j < kCells; j++) {
      if (!seen[i][j]) continue;
      const bool fits = white_expected(i, j) ? cell[i][j] >= black + kWhiteShare * contrast
                                             : cell[i][j] <= black + kBlackShare * contrast;
      if (!fits) return reading;
    }
  }
  reading.match = true;
  reading.white = white;
  reading.black = black;
  reading.saturated = static_cast<double>(saturated) / white_count;
  return reading;
}

struct Hit {
  Square square;
  Reading reading;
  double side_px = 0.0;
  Point2 center;
};

Point2 Center(const std::array<Point2, 4>& q) {
  Point2 mid;
  for (const Point2& p : q) {
    mid.x += p.x / 4.0;
    mid.y += p.y / 4.0;
  }
  return mid;
}

// Checks a candidate whose corners in `hit.square.px` are good to a few pixels: a quick pass over the sides and the
// pattern sorts out what is no marker, the exact pass follows only for what is left.
bool ReadSquare(const Sampler& image, const LensModel& lens, const MarkerParams& params, Hit& hit) {
  std::array<Line, 4> lines;
  Square& square = hit.square;
  if (!LineUp(image, nullptr, params, kQuickSideSamples, square, lines)) return false;
  for (size_t k = 0; k < 4; k++) {
    if (!ToRay(lens, square.px[k], square.rays[k])) return false;
  }
  if (!Upright(square.rays, params.max_slant) || !ReadPattern(Grid(image, lens, square.rays), params).match) {
    return false;
  }
  if (!LineUp(image, &lens, params, kMaxSideSamples, square, lines)) return false;
  hit.side_px = MeanSide(square.px);
  if (hit.side_px < params.min_side_px || !Upright(square.rays, params.max_slant)) return false;
  hit.reading = ReadPattern(Grid(image, lens, square.rays), params);
  if (!hit.reading.match) return false;
  square.covariance = CornerCovariance(lines, square.rays, static_cast<double>(params.corner_sigma_floor_px) / lens.fx);
  hit.center = Center(square.px);
  return true;
}

// Corners of a candidate in pixels of the full image and the size of a pixel of the level it was found on.
struct Candidate {
  std::array<Point2, 4> px{};
  double scale = 1.0;
};

// Part of one row of dark pixels, first to last, and the run that stands for the connected area it belongs to.
struct Run {
  int y = 0;
  int first = 0;
  int last = 0;
  uint32_t parent = 0;
};

uint32_t Root(std::vector<Run>& runs, uint32_t i) {
  while (runs[i].parent != i) {
    runs[i].parent = runs[runs[i].parent].parent;
    i = runs[i].parent;
  }
  return i;
}

struct Blob {
  int min_x = 0;
  int max_x = 0;
  int min_y = 0;
  int max_y = 0;
  uint32_t pixels = 0;
  // First and last pixel of each of its rows, for the hull.
  std::vector<Point2> outline;
};

// Reads a candidate at the border of the image as a marker that is cut off. Either the whole square is in the image
// and only the white field is not, or the border cuts two corners off: then the square is completed from the side
// that is whole, as long as its sides, which is right for a marker seen from the front.
bool ReadClipped(const Sampler& image, const GrayView& view, const LensModel& lens, const MarkerParams& params,
                 const Candidate& candidate, Hit& hit) {
  const double near = 1.5 * candidate.scale + 1.0;
  const double reach = MeanSide(candidate.px) / kCells + near;
  std::array<bool, 4> cut{};
  int cut_count = 0;
  bool at_border = false;
  std::array<Point2, 4> rays;
  for (size_t k = 0; k < 4; k++) {
    const Point2& p = candidate.px[k];
    const double gap = std::min(std::min(p.x, p.y), std::min(view.width - 1.0 - p.x, view.height - 1.0 - p.y));
    cut[k] = gap < near;
    if (cut[k]) cut_count++;
    at_border = at_border || gap < reach;
    const Point2 inside{std::clamp(p.x, 0.0, view.width - 1.0), std::clamp(p.y, 0.0, view.height - 1.0)};
    if (!ToRay(lens, inside, rays[k])) return false;
  }
  if (!at_border || (cut_count != 0 && cut_count != 2)) return false;
  if (cut_count == 2) {
    size_t first = 0;
    while (first < 4 && !(cut[first] && cut[(first + 1) % 4])) first++;
    if (first == 4) return false;
    // The two whole corners, each next to one that is cut off.
    const std::array<std::array<size_t, 2>, 2> pairs = {{{(first + 2) % 4, (first + 1) % 4}, {(first + 3) % 4, first}}};
    const double side = Distance(rays[pairs[0][0]], rays[pairs[1][0]]);
    for (const auto& pair : pairs) {
      const Point2& whole = rays[pair[0]];
      Point2& lost = rays[pair[1]];
      const double visible = Distance(whole, lost);
      if (visible < kMinVisible * side) return false;
      lost = {whole.x + (lost.x - whole.x) * side / visible, whole.y + (lost.y - whole.y) * side / visible};
    }
  }
  if (!Upright(rays, params.max_slant)) return false;
  hit.reading = ReadVisible(Grid(image, lens, rays), params);
  if (!hit.reading.match) return false;
  for (size_t k = 0; k < 4; k++) hit.square.px[k] = ToPixel(lens, rays[k]);
  hit.side_px = MeanSide(hit.square.px);
  hit.center = Center(hit.square.px);
  return true;
}

// Candidate squares of one level: outlines of dark areas with four straight sides, in pixels of the full image.
// Dark pixels belong together across the four sides of a pixel.
void FindQuads(const Plane& plane, const MarkerParams& params, double scale, std::vector<Candidate>& quads) {
  const auto w = static_cast<int>(plane.width);
  const auto h = static_cast<int>(plane.height);
  const uint32_t cols = (plane.width + kTile - 1) / kTile;
  const uint32_t rows = (plane.height + kTile - 1) / kTile;
  const std::vector<uint8_t> limits = DarkLimits(plane, params, cols, rows);

  std::vector<Run> runs;
  size_t above_begin = 0;
  size_t above_end = 0;
  for (int y = 0; y < h; y++) {
    const uint8_t* line = plane.px.data() + static_cast<size_t>(y) * plane.width;
    const uint8_t* limit = limits.data() + static_cast<size_t>(static_cast<uint32_t>(y) / kTile) * cols;
    const size_t row_begin = runs.size();
    size_t above = above_begin;
    for (int x = 0; x < w; x++) {
      if (line[x] >= limit[static_cast<uint32_t>(x) / kTile]) continue;
      Run run;
      run.y = y;
      run.first = x;
      while (x + 1 < w && line[x + 1] < limit[static_cast<uint32_t>(x + 1) / kTile]) x++;
      run.last = x;
      const auto index = static_cast<uint32_t>(runs.size());
      run.parent = index;
      runs.push_back(run);
      // Joins the runs of the row above that touch this one.
      while (above < above_end && runs[above].last < run.first) above++;
      for (size_t k = above; k < above_end && runs[k].first <= run.last; k++) {
        const uint32_t mine = Root(runs, index);
        const uint32_t other = Root(runs, static_cast<uint32_t>(k));
        runs[std::max(mine, other)].parent = std::min(mine, other);
      }
    }
    above_begin = row_begin;
    above_end = runs.size();
  }

  std::vector<Blob> blobs;
  std::vector<uint32_t> blob_of(runs.size(), 0);
  for (uint32_t i = 0; i < runs.size(); i++) {
    const Run& run = runs[i];
    const uint32_t root = Root(runs, i);
    if (root == i) {
      blob_of[i] = static_cast<uint32_t>(blobs.size());
      blobs.push_back({run.first, run.last, run.y, run.y, 0, {}});
    }
    Blob& blob = blobs[blob_of[root]];
    blob.min_x = std::min(blob.min_x, run.first);
    blob.max_x = std::max(blob.max_x, run.last);
    blob.max_y = run.y;
    blob.pixels += static_cast<uint32_t>(run.last - run.first + 1);
  }
  std::vector<uint8_t> wanted(blobs.size(), 0);
  for (size_t i = 0; i < blobs.size(); i++) {
    const Blob& blob = blobs[i];
    const int width = blob.max_x - blob.min_x + 1;
    const int height = blob.max_y - blob.min_y + 1;
    // A square, even slanted and with white cells, fills more of its box than this.
    wanted[i] = std::max(width, height) >= kMinBlobLong && std::min(width, height) >= kMinBlobShort &&
                        blob.pixels * 5 >= static_cast<uint32_t>(width * height)
                    ? 1
                    : 0;
  }
  for (uint32_t i = 0; i < runs.size(); i++) {
    const uint32_t blob = blob_of[Root(runs, i)];
    if (wanted[blob] == 0) continue;
    std::vector<Point2>& outline = blobs[blob].outline;
    const auto y = static_cast<double>(runs[i].y);
    if (!outline.empty() && outline.back().y == y) {
      // Runs come row by row from the left: a further run of the same row moves the end of the row.
      outline.back().x = static_cast<double>(runs[i].last);
    } else {
      outline.push_back({static_cast<double>(runs[i].first), y});
      outline.push_back({static_cast<double>(runs[i].last), y});
    }
  }

  for (size_t i = 0; i < blobs.size(); i++) {
    if (wanted[i] == 0) continue;
    std::array<Point2, 4> quad;
    if (!HullQuad(Hull(blobs[i].outline), quad)) continue;
    const double area = QuadArea(quad);
    const auto filled = static_cast<double>(blobs[i].pixels);
    if (filled < 0.35 * area || filled > 1.3 * area + 2.0 * MeanSide(quad)) continue;
    SortCorners(quad);
    const Blob& blob = blobs[i];
    const bool cut = blob.min_x == 0 || blob.min_y == 0 || blob.max_x == w - 1 || blob.max_y == h - 1;
    if (!SquareLike(quad, params.max_slant, cut)) continue;
    // The hull runs through pixel centers, the area reaches half a pixel further; then to the full image.
    const Point2 mid = Center(quad);
    const double grow = 1.0 + 1.0 / MeanSide(quad);
    for (Point2& p : quad) {
      p.x = (mid.x + (p.x - mid.x) * grow + 0.5) * scale - 0.5;
      p.y = (mid.y + (p.y - mid.y) * grow + 0.5) * scale - 0.5;
    }
    quads.push_back({quad, scale});
  }
}

// Mean luma and share of saturated pixels, over every eighth pixel in both directions.
void ImageStats(const GrayView& image, float saturation, MarkerDetection& detection) {
  uint64_t sum = 0;
  uint64_t count = 0;
  uint64_t saturated = 0;
  for (uint32_t y = kStatsGrid / 2; y < image.height; y += kStatsGrid) {
    const uint8_t* row = image.data + static_cast<size_t>(y) * image.stride;
    for (uint32_t x = kStatsGrid / 2; x < image.width; x += kStatsGrid) {
      sum += row[x];
      count++;
      if (static_cast<float>(row[x]) >= saturation) saturated++;
    }
  }
  if (count == 0) return;
  detection.image_mean = static_cast<double>(sum) / static_cast<double>(count);
  detection.image_saturated = static_cast<double>(saturated) / static_cast<double>(count);
}

}  // namespace

MarkerDetection DetectMarker(const GrayView& image, const LensModel& lens, const MarkerParams& params) {
  MarkerDetection detection;
  if (image.data == nullptr || image.width < kMinLevelWidth || image.height < kMinLevelHeight ||
      image.stride < image.width || !(lens.fx > 0.0) || !(lens.fy > 0.0)) {
    return detection;
  }
  ImageStats(image, params.saturation, detection);
  const Sampler sampler(image);

  std::vector<Hit> hits;
  std::vector<Hit> clipped;
  std::vector<Candidate> quads;
  Plane plane;
  const auto known = [](const std::vector<Hit>& list, const Point2& mid) {
    return std::any_of(list.begin(), list.end(),
                       [&mid](const Hit& hit) { return Distance(hit.center, mid) < 0.5 * hit.side_px; });
  };
  for (uint32_t level = 0; level <= params.last_level; level++) {
    if (level == 0) {
      if (params.first_level == 0) plane = Copy(image);
    } else if (level == 1) {
      plane = Halve(image.data, image.width, image.height, image.stride);
    } else {
      plane = Halve(plane.px.data(), plane.width, plane.height, plane.width);
    }
    if (level < params.first_level) continue;
    if (plane.width < kMinLevelWidth || plane.height < kMinLevelHeight) break;
    quads.clear();
    FindQuads(plane, params, static_cast<double>(1U << level), quads);
    for (const Candidate& quad : quads) {
      if (known(hits, Center(quad.px))) continue;
      Hit hit;
      hit.square.px = quad.px;
      if (ReadSquare(sampler, lens, params, hit)) {
        hits.push_back(hit);
      } else if (ReadClipped(sampler, image, lens, params, quad, hit) && !known(clipped, hit.center)) {
        clipped.push_back(hit);
      }
    }
  }

  if (hits.size() > 1) {
    detection.result = MarkerResult::kAmbiguous;
    return detection;
  }
  if (hits.empty()) {
    // What is cut off cannot be measured; it is reported only if nothing whole is there.
    if (!clipped.empty()) {
      detection.result = MarkerResult::kClipped;
      detection.side_px = clipped.front().side_px;
      detection.white = clipped.front().reading.white;
      detection.black = clipped.front().reading.black;
      detection.saturated = clipped.front().reading.saturated;
    } else if (detection.image_mean >= params.bright_mean) {
      detection.result = MarkerResult::kTooBright;
    } else if (detection.image_mean <= params.dark_mean) {
      detection.result = MarkerResult::kTooDark;
    }
    return detection;
  }
  const Hit& hit = hits.front();
  detection.corners_px = hit.square.px;
  detection.corners = hit.square.rays;
  detection.corner_covariance = hit.square.covariance;
  double trace = 0.0;
  for (size_t i = 0; i < 8; i++) trace += hit.square.covariance[i * 9];
  detection.corner_sigma_px = std::sqrt(trace / 8.0) * lens.fx;
  detection.side_px = hit.side_px;
  detection.white = hit.reading.white;
  detection.black = hit.reading.black;
  detection.saturated = hit.reading.saturated;
  detection.edge_px = hit.square.edge_px;
  if (hit.reading.saturated > params.max_saturated) {
    detection.result = MarkerResult::kTooBright;
  } else if (hit.square.edge_px > params.max_edge_cells * hit.side_px / kCells) {
    detection.result = MarkerResult::kBlurred;
  } else {
    detection.result = MarkerResult::kFound;
  }
  return detection;
}

}  // namespace tron_vision
