// Computed pictures of the stereo camera: flat textured ground and upright textured boxes in base_link, seen by both
// eyes of a calibration, as one YUYV frame with the eyes side by side.
#ifndef TRON_VISION_TEST_STEREO_SCENE_H
#define TRON_VISION_TEST_STEREO_SCENE_H

#include <cmath>
#include <cstdint>
#include <vector>

#include "mowbite_obstacles/ground.h"
#include "tron_vision/stereo_lens.h"

namespace tron_vision::test {

// The calibration of the Tron (fisheye, 16.11.), rounded.
inline StereoCalibration TronCalibration() {
  StereoCalibration c;
  c.width = 640;
  c.height = 480;
  c.left = {446.43, 447.85, 318.99, 232.69, -0.0336, -0.3081, 0.6034, -0.3920};
  c.right = {445.88, 448.23, 314.35, 231.34, -0.0666, -0.1297, 0.2200, -0.1225};
  const double r[9] = {0.99996, 0.00309, 0.00856, -0.00312, 0.99999, 0.00287, -0.00855, -0.00290, 0.99996};
  for (int i = 0; i < 9; i++) c.rotation[i] = r[i];
  c.translation[0] = -0.06033;
  c.translation[1] = 0.00015;
  c.translation[2] = 0.00325;
  return c;
}

struct V3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

// Rz(yaw) Ry(pitch) Rx(roll) of a vector, as mowbite_obstacles::Mount means it.
inline V3 Turn(const mowbite_obstacles::Mount& m, const V3& v) {
  const double cr = std::cos(m.roll), sr = std::sin(m.roll), cp = std::cos(m.pitch), sp = std::sin(m.pitch);
  const double cy = std::cos(m.yaw), sy = std::sin(m.yaw);
  const V3 a{v.x, cr * v.y - sr * v.z, sr * v.y + cr * v.z};
  const V3 b{cp * a.x + sp * a.z, a.y, -sp * a.x + cp * a.z};
  return {cy * b.x - sy * b.y, sy * b.x + cy * b.y, b.z};
}

// A vector of the frame of the left eye (x right, y down, z ahead) in base_link.
inline V3 CameraToBase(const mowbite_obstacles::Mount& m, const V3& v) {
  return Turn(m, {v.z, -v.x, -v.y});
}

// An upright box standing on the ground: x from near to far, y from right to left, up to height.
struct Block {
  double near = 0.0;
  double far = 0.0;
  double right = 0.0;
  double left = 0.0;
  double height = 0.0;
};

struct Scene {
  StereoCalibration calibration = TronCalibration();
  mowbite_obstacles::Mount mount{0.62, 0.03, 0.19, 0.0, 0.26, 0.0};
  double ground_z = -0.1125;
  std::vector<Block> blocks;
  // Size of a cell of the texture (m).
  double cell = 0.01;
  uint8_t sky = 120;

  // Luma of a point from a hash of its cell.
  uint8_t Texture(double a, double b) const {
    const auto i = static_cast<int64_t>(std::floor(a / cell));
    const auto j = static_cast<int64_t>(std::floor(b / cell));
    uint64_t h = static_cast<uint64_t>(i) * 0x9E3779B97F4A7C15ULL ^ static_cast<uint64_t>(j) * 0xC2B2AE3D27D4EB4FULL;
    h ^= h >> 29;
    h *= 0xBF58476D1CE4E5B9ULL;
    h ^= h >> 32;
    return static_cast<uint8_t>(40 + h % 160);
  }

  // What a ray from `origin` along `dir` (base_link) meets first.
  uint8_t Trace(const V3& origin, const V3& dir) const {
    double best = 1e9;
    uint8_t value = sky;
    if (dir.z < -1e-9) {
      const double t = (ground_z - origin.z) / dir.z;
      if (t > 0.0 && t < best) {
        best = t;
        value = Texture(origin.x + t * dir.x, origin.y + t * dir.y);
      }
    }
    for (const Block& b : blocks) {
      // The face towards the camera.
      if (std::abs(dir.x) < 1e-9) continue;
      const double t = (b.near - origin.x) / dir.x;
      const double y = origin.y + t * dir.y;
      const double z = origin.z + t * dir.z;
      if (t > 0.0 && t < best && y >= b.right && y <= b.left && z >= ground_z && z <= ground_z + b.height) {
        best = t;
        value = Texture(y + 7.0, z);
      }
    }
    return value;
  }

  // Mean of 2 x 2 rays across a pixel, like the blur of a lens.
  uint8_t Pixel(const FisheyeLens& lens, double x, double y, const V3& origin, bool right) const {
    const double* r = calibration.rotation;
    int sum = 0;
    for (const double dy : {-0.25, 0.25}) {
      for (const double dx : {-0.25, 0.25}) {
        Point2 ray;
        if (!ToRay(lens, {x + dx, y + dy}, ray)) {
          sum += sky;
          continue;
        }
        // A ray of the right eye turned into the frame of the left one: R^T d.
        const V3 d = right ? V3{r[0] * ray.x + r[3] * ray.y + r[6], r[1] * ray.x + r[4] * ray.y + r[7],
                                r[2] * ray.x + r[5] * ray.y + r[8]}
                           : V3{ray.x, ray.y, 1.0};
        sum += Trace(origin, CameraToBase(mount, d));
      }
    }
    return static_cast<uint8_t>((sum + 2) / 4);
  }

  // Both eyes as YUYV, 4 * width bytes per line, chroma 128.
  std::vector<uint8_t> Render() const {
    const uint32_t w = calibration.width;
    const uint32_t h = calibration.height;
    std::vector<uint8_t> frame(static_cast<size_t>(4) * w * h, 128);
    const V3 origin{mount.x, mount.y, mount.z};
    // Center of the right eye in the left frame: -R^T T.
    const double* r = calibration.rotation;
    const double* t = calibration.translation;
    const V3 right_center{-(r[0] * t[0] + r[3] * t[1] + r[6] * t[2]), -(r[1] * t[0] + r[4] * t[1] + r[7] * t[2]),
                          -(r[2] * t[0] + r[5] * t[1] + r[8] * t[2])};
    const V3 rc = CameraToBase(mount, right_center);
    const V3 right_origin{origin.x + rc.x, origin.y + rc.y, origin.z + rc.z};
    for (uint32_t y = 0; y < h; y++) {
      for (uint32_t x = 0; x < w; x++) {
        frame[static_cast<size_t>(y) * 4 * w + 2 * x] = Pixel(calibration.left, x, y, origin, false);
        frame[static_cast<size_t>(y) * 4 * w + 2 * w + 2 * x] = Pixel(calibration.right, x, y, right_origin, true);
      }
    }
    return frame;
  }

  // Where a point of base_link lands in the left image; false behind the camera.
  bool Project(const V3& p, Point2& pixel) const {
    mowbite_obstacles::Ray ray;
    if (!mowbite_obstacles::RayOf(mount, {p.x, p.y, p.z}, &ray)) return false;
    pixel = ToPixel(calibration.left, {ray.x, ray.y});
    return true;
  }
};

}  // namespace tron_vision::test

#endif  // TRON_VISION_TEST_STEREO_SCENE_H
