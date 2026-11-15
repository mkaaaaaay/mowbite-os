#ifndef TRON_VISION_DOCK_SCENE_H
#define TRON_VISION_DOCK_SCENE_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include "tron_vision/dock_marker.h"
#include "tron_vision/dock_pose.h"
#include "tron_vision/lens_model.h"

// Draws markers as the camera would see them, for the tests of the detector and the fit. Geometry by rays through
// the pixels, independent of the code under test except for the lens model.
namespace tron_vision::test {

// A marker on its housing: the black square with the pattern, a white field around it, dark housing around that.
struct SceneMarker {
  // Center and the direction the face looks in, in base_link.
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
  uint16_t pattern = 0;
  // Turned in its plane by this many quarter turns.
  int quarter_turns = 0;
  // Leaning back by this angle, top away from the viewer.
  double lean = 0.0;
  // Turned in its face by this angle, from its right side up as the viewer sees it.
  double roll = 0.0;
};

// Rectangle on the first marker's plane, painted over it: right and up from the center, in sides of the square.
struct SceneCover {
  double left = 0.0;
  double right = 0.0;
  double bottom = 0.0;
  double top = 0.0;
  double luma = 0.0;
};

struct Scene {
  LensModel lens;
  uint32_t width = 0;
  uint32_t height = 0;
  CameraMount mount;
  std::vector<SceneMarker> markers;
  std::vector<SceneCover> covers;
  double side = 0.04;
  // Width of the white field and of the housing beyond it, in sides of the square.
  double field = 0.2;
  double housing = 0.6;
  // Luma at exposure 1 before the sensor clips.
  double white = 200.0;
  double black = 30.0;
  double housing_luma = 45.0;
  double background = 110.0;
  double exposure = 1.0;
  // Light falls to this share right of a line through the image that leans by `shadow_lean` pixels per row.
  double shadow = 1.0;
  double shadow_x = 0.0;
  double shadow_lean = 0.0;
  // Standard deviation of the lens blur in pixels and of the sensor noise in luma steps.
  double blur = 0.8;
  double noise = 0.0;
  uint32_t seed = 1;
};

struct SceneImage {
  uint32_t width = 0;
  uint32_t height = 0;
  std::vector<uint8_t> px;
  GrayView View() const {
    return {px.data(), width, height, width};
  }
};

namespace scene_detail {

struct Vec3 {
  double x;
  double y;
  double z;
};

// Direction in base_link of a ray given in the frame of the camera.
inline Vec3 ToBase(const CameraMount& m, const Point2& ray) {
  // Body of the camera: x along the view, y left, z up; turned by roll, then pitch, then yaw.
  const Vec3 v{1.0, -ray.x, -ray.y};
  const Vec3 a{v.x, v.y * std::cos(m.roll) - v.z * std::sin(m.roll), v.y * std::sin(m.roll) + v.z * std::cos(m.roll)};
  const Vec3 b{a.x * std::cos(m.pitch) + a.z * std::sin(m.pitch), a.y,
               -a.x * std::sin(m.pitch) + a.z * std::cos(m.pitch)};
  return {b.x * std::cos(m.yaw) - b.y * std::sin(m.yaw), b.x * std::sin(m.yaw) + b.y * std::cos(m.yaw), b.z};
}

// Pixel of a point given in base_link; false behind the camera or far outside its view.
inline bool ToImage(const Scene& s, const Vec3& point, Point2& px) {
  // Into the body of the camera: yaw, pitch and roll undone in turn.
  const Vec3 p{point.x - s.mount.x, point.y - s.mount.y, point.z - s.mount.z};
  const Vec3 a{p.x * std::cos(s.mount.yaw) + p.y * std::sin(s.mount.yaw),
               -p.x * std::sin(s.mount.yaw) + p.y * std::cos(s.mount.yaw), p.z};
  const Vec3 b{a.x * std::cos(s.mount.pitch) - a.z * std::sin(s.mount.pitch), a.y,
               a.x * std::sin(s.mount.pitch) + a.z * std::cos(s.mount.pitch)};
  const Vec3 c{b.x, b.y * std::cos(s.mount.roll) + b.z * std::sin(s.mount.roll),
               -b.y * std::sin(s.mount.roll) + b.z * std::cos(s.mount.roll)};
  if (c.x < 0.01 || std::hypot(c.y, c.z) > 0.8 * c.x) return false;
  px = ToPixel(s.lens, {-c.y / c.x, -c.z / c.x});
  return true;
}

// Point on the marker's plane, `right` and `up` from its center as the viewer in front of it sees them, in meters.
inline Vec3 OnMarker(const SceneMarker& m, double right_in_face, double up_in_face) {
  const double right = right_in_face * std::cos(m.roll) - up_in_face * std::sin(m.roll);
  const double up = right_in_face * std::sin(m.roll) + up_in_face * std::cos(m.roll);
  // The face direction turned by a quarter to the left is the viewer's right; leaning back moves the top along
  // the direction opposite to the face.
  const double out = -up * std::sin(m.lean);
  return {m.x - std::sin(m.yaw) * right + std::cos(m.yaw) * out, m.y + std::cos(m.yaw) * right + std::sin(m.yaw) * out,
          m.z + up * std::cos(m.lean)};
}

// Luma of the scene along a ray, before exposure.
inline double Look(const Scene& s, const Point2& ray) {
  const Vec3 dir = ToBase(s.mount, ray);
  for (size_t i = 0; i < s.markers.size(); i++) {
    const SceneMarker& m = s.markers[i];
    // Normal of the plane: the face direction, tipped up by the lean.
    const Vec3 n{std::cos(m.yaw) * std::cos(m.lean), std::sin(m.yaw) * std::cos(m.lean), std::sin(m.lean)};
    const double approach = n.x * dir.x + n.y * dir.y + n.z * dir.z;
    if (approach >= 0.0) continue;
    const double t = (n.x * (m.x - s.mount.x) + n.y * (m.y - s.mount.y) + n.z * (m.z - s.mount.z)) / approach;
    if (t <= 0.0) continue;
    const Vec3 p{s.mount.x + t * dir.x - m.x, s.mount.y + t * dir.y - m.y, s.mount.z + t * dir.z - m.z};
    const double plane_right = (-std::sin(m.yaw) * p.x + std::cos(m.yaw) * p.y) / s.side;
    const double plane_up =
        (-std::sin(m.lean) * (std::cos(m.yaw) * p.x + std::sin(m.yaw) * p.y) + std::cos(m.lean) * p.z) / s.side;
    double right = plane_right * std::cos(m.roll) + plane_up * std::sin(m.roll);
    double up = -plane_right * std::sin(m.roll) + plane_up * std::cos(m.roll);
    const double reach = std::max(std::fabs(right), std::fabs(up));
    if (reach > 0.5 + s.field + s.housing) continue;
    if (i == 0) {
      for (const SceneCover& c : s.covers) {
        if (right >= c.left && right <= c.right && up >= c.bottom && up <= c.top) return c.luma;
      }
    }
    if (reach > 0.5 + s.field) return s.housing_luma;
    if (reach > 0.5) return s.white;
    for (int q = 0; q < ((m.quarter_turns % 4) + 4) % 4; q++) {
      const double old = right;
      right = up;
      up = -old;
    }
    const int col = std::clamp(static_cast<int>((right + 0.5) * 6.0), 0, 5);
    const int row = std::clamp(static_cast<int>((0.5 - up) * 6.0), 0, 5);
    if (col == 0 || col == 5 || row == 0 || row == 5) return s.black;
    const int bit = (row - 1) * 4 + (col - 1);
    return ((m.pattern >> (15 - bit)) & 1U) != 0 ? s.white : s.black;
  }
  return s.background;
}

struct Rect {
  int x0;
  int y0;
  int x1;
  int y1;
};

// Gaussian blur of the pixels inside the rectangle.
inline void Blur(std::vector<float>& img, uint32_t w, uint32_t h, const Rect& rect, double sigma) {
  if (sigma <= 0.0) return;
  const int radius = std::max(1, static_cast<int>(std::ceil(3.0 * sigma)));
  std::vector<float> kernel(static_cast<size_t>(2 * radius + 1));
  float sum = 0.0F;
  for (int k = -radius; k <= radius; k++) {
    kernel[static_cast<size_t>(k + radius)] = static_cast<float>(std::exp(-0.5 * k * k / (sigma * sigma)));
    sum += kernel[static_cast<size_t>(k + radius)];
  }
  for (float& k : kernel) k /= sum;
  std::vector<float> tmp = img;
  const auto wi = static_cast<int>(w);
  const auto hi = static_cast<int>(h);
  for (int pass = 0; pass < 2; pass++) {
    for (int y = rect.y0; y < rect.y1; y++) {
      for (int x = rect.x0; x < rect.x1; x++) {
        float v = 0.0F;
        for (int k = -radius; k <= radius; k++) {
          const int xx = pass == 0 ? std::clamp(x + k, 0, wi - 1) : x;
          const int yy = pass == 0 ? y : std::clamp(y + k, 0, hi - 1);
          v += kernel[static_cast<size_t>(k + radius)] * img[static_cast<size_t>(yy) * w + static_cast<size_t>(xx)];
        }
        tmp[static_cast<size_t>(y) * w + static_cast<size_t>(x)] = v;
      }
    }
    img = tmp;
  }
}

// Part of the image the markers and their housings can show in, with `margin` pixels around; the whole image if
// that cannot be told.
inline Rect MarkerRect(const Scene& s, int margin) {
  const Rect all{0, 0, static_cast<int>(s.width), static_cast<int>(s.height)};
  Rect rect{all.x1, all.y1, 0, 0};
  const double reach = (0.5 + s.field + s.housing) * s.side;
  for (const SceneMarker& m : s.markers) {
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        Point2 px;
        if (!ToImage(s, OnMarker(m, i * reach, j * reach), px)) return all;
        rect.x0 = std::min(rect.x0, static_cast<int>(std::floor(px.x)) - margin);
        rect.y0 = std::min(rect.y0, static_cast<int>(std::floor(px.y)) - margin);
        rect.x1 = std::max(rect.x1, static_cast<int>(std::ceil(px.x)) + margin + 1);
        rect.y1 = std::max(rect.y1, static_cast<int>(std::ceil(px.y)) + margin + 1);
      }
    }
  }
  return {std::clamp(rect.x0, 0, all.x1), std::clamp(rect.y0, 0, all.y1), std::clamp(rect.x1, 0, all.x1),
          std::clamp(rect.y1, 0, all.y1)};
}

// Rays of the pixels, exact on a grid and by their share in between: the distortion is smooth.
class RayField {
 public:
  RayField(const LensModel& lens, uint32_t width, uint32_t height)
      : cols_(width / kGrid + 2), rows_(height / kGrid + 2), rays_(static_cast<size_t>(cols_) * rows_) {
    for (uint32_t r = 0; r < rows_; r++) {
      for (uint32_t c = 0; c < cols_; c++) {
        ok_ = ToRay(lens, {static_cast<double>(c * kGrid), static_cast<double>(r * kGrid)},
                    rays_[static_cast<size_t>(r) * cols_ + c]) &&
              ok_;
      }
    }
  }
  bool ok() const {
    return ok_;
  }
  Point2 At(double x, double y) const {
    const double gx = std::clamp(x / kGrid, 0.0, cols_ - 1.001);
    const double gy = std::clamp(y / kGrid, 0.0, rows_ - 1.001);
    const auto c = static_cast<size_t>(gx);
    const auto r = static_cast<size_t>(gy);
    const double fx = gx - static_cast<double>(c);
    const double fy = gy - static_cast<double>(r);
    const Point2& p00 = rays_[r * cols_ + c];
    const Point2& p10 = rays_[r * cols_ + c + 1];
    const Point2& p01 = rays_[(r + 1) * cols_ + c];
    const Point2& p11 = rays_[(r + 1) * cols_ + c + 1];
    return {(1 - fy) * ((1 - fx) * p00.x + fx * p10.x) + fy * ((1 - fx) * p01.x + fx * p11.x),
            (1 - fy) * ((1 - fx) * p00.y + fx * p10.y) + fy * ((1 - fx) * p01.y + fx * p11.y)};
  }

 private:
  static constexpr uint32_t kGrid = 4;
  uint32_t cols_;
  uint32_t rows_;
  std::vector<Point2> rays_;
  bool ok_ = true;
};

}  // namespace scene_detail

// Where the corners of a marker's black square are in the image: top left, top right, bottom right, bottom left.
inline std::array<Point2, 4> CornerPixels(const Scene& s, const SceneMarker& m) {
  std::array<Point2, 4> corners{};
  const double half = s.side / 2.0;
  const std::array<std::array<double, 2>, 4> signs = {{{-1.0, 1.0}, {1.0, 1.0}, {1.0, -1.0}, {-1.0, -1.0}}};
  for (size_t k = 0; k < 4; k++) {
    scene_detail::ToImage(s, scene_detail::OnMarker(m, signs[k][0] * half, signs[k][1] * half), corners[k]);
  }
  return corners;
}

// The scene before the sensor: exposure and lens blur applied, values not yet clipped. Pixels on an edge take the
// mean of many rays; outside the part of the image the markers can show in, plain background.
inline std::vector<float> Expose(const Scene& s) {
  std::vector<float> img(static_cast<size_t>(s.width) * s.height);
  const auto light = [&s](uint32_t x, uint32_t y) {
    const bool shaded = s.shadow < 1.0 && static_cast<double>(x) > s.shadow_x + s.shadow_lean * y;
    return s.exposure * (shaded ? s.shadow : 1.0);
  };
  for (uint32_t y = 0; y < s.height; y++) {
    for (uint32_t x = 0; x < s.width; x++) {
      img[static_cast<size_t>(y) * s.width + x] = static_cast<float>(s.background * light(x, y));
    }
  }
  const scene_detail::RayField rays(s.lens, s.width, s.height);
  if (!rays.ok()) return img;
  const int margin = static_cast<int>(std::ceil(3.0 * s.blur)) + 3;
  const scene_detail::Rect rect = scene_detail::MarkerRect(s, margin);
  // Sample places spread evenly over a pixel in both directions, so that an edge along the pixel grid is weighed
  // by its true share too.
  constexpr int kSamples = 128;
  for (int y = rect.y0; y < rect.y1; y++) {
    for (int x = rect.x0; x < rect.x1; x++) {
      // One value where the corners and the middle of the pixel see the same.
      double sum = scene_detail::Look(s, rays.At(x, y));
      bool plain = true;
      for (const auto& corner : {std::array<double, 2>{-0.5, -0.5}, {0.5, -0.5}, {-0.5, 0.5}, {0.5, 0.5}}) {
        plain = plain && scene_detail::Look(s, rays.At(x + corner[0], y + corner[1])) == sum;
      }
      if (!plain) {
        sum = 0.0;
        for (int i = 0; i < kSamples; i++) {
          const double fx = std::fmod(0.5 + i * 0.7548776662466927, 1.0) - 0.5;
          const double fy = std::fmod(0.5 + i * 0.5698402909980532, 1.0) - 0.5;
          sum += scene_detail::Look(s, rays.At(x + fx, y + fy)) / kSamples;
        }
      }
      const auto ux = static_cast<uint32_t>(x);
      const auto uy = static_cast<uint32_t>(y);
      img[static_cast<size_t>(uy) * s.width + ux] = static_cast<float>(sum * light(ux, uy));
    }
  }
  scene_detail::Blur(img, s.width, s.height, rect, s.blur);
  return img;
}

// Sensor: noise, clipping at 255, whole steps.
inline SceneImage Develop(const std::vector<float>& exposed, const Scene& s, uint32_t seed) {
  SceneImage image;
  image.width = s.width;
  image.height = s.height;
  image.px.resize(exposed.size());
  std::mt19937 rng(seed);
  // Sum of four bytes: close to normal, mean 510, standard deviation 147.8.
  constexpr double kSumSigma = 147.80;
  for (size_t i = 0; i < exposed.size(); i++) {
    double v = exposed[i];
    if (s.noise > 0.0) {
      const uint32_t r = rng();
      const double sum = (r & 255U) + ((r >> 8U) & 255U) + ((r >> 16U) & 255U) + (r >> 24U);
      v += s.noise * (sum - 510.0) / kSumSigma;
    }
    image.px[i] = static_cast<uint8_t>(std::clamp(std::lround(v), 0L, 255L));
  }
  return image;
}

inline SceneImage Render(const Scene& s) {
  return Develop(Expose(s), s, s.seed);
}

// Pattern of the markers in the tests, the one of the stations of the Tron.
constexpr uint16_t kDockPattern = 0xB532;

// Camera at the rear, looking back and a little down, not quite level; lens with strong barrel distortion.
inline Scene RearScene() {
  Scene s;
  s.lens = {1540.0, 1540.0, 986.0, 521.0, -0.37, 0.17, -0.0001, -0.0005, 0.0};
  s.width = 1920;
  s.height = 1080;
  s.mount = {-0.2, 0.01, 0.25, 3.14159265358979323846, 0.05, 0.01};
  return s;
}

// A marker this far behind the camera and to its side, a little above it.
inline SceneMarker Behind(const Scene& s, double distance, double lateral, double yaw) {
  SceneMarker m;
  m.x = s.mount.x - distance;
  m.y = s.mount.y + lateral;
  m.z = s.mount.z + 0.02;
  m.yaw = yaw;
  m.pattern = kDockPattern;
  return m;
}

}  // namespace tron_vision::test

#endif  // TRON_VISION_DOCK_SCENE_H
