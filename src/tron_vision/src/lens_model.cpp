#include "tron_vision/lens_model.h"

#include <cmath>

namespace tron_vision {

namespace {

constexpr int kMaxIterations = 40;
// In units of the focal length; a millionth of a pixel at 1000 px.
constexpr double kSettled = 1e-9;

Point2 Distort(const LensModel& lens, const Point2& ray) {
  const double r2 = ray.x * ray.x + ray.y * ray.y;
  const double radial = 1.0 + r2 * (lens.k1 + r2 * (lens.k2 + r2 * lens.k3));
  Point2 d;
  d.x = ray.x * radial + 2.0 * lens.p1 * ray.x * ray.y + lens.p2 * (r2 + 2.0 * ray.x * ray.x);
  d.y = ray.y * radial + lens.p1 * (r2 + 2.0 * ray.y * ray.y) + 2.0 * lens.p2 * ray.x * ray.y;
  return d;
}

}  // namespace

Point2 ToPixel(const LensModel& lens, const Point2& ray) {
  const Point2 d = Distort(lens, ray);
  return {lens.fx * d.x + lens.cx, lens.fy * d.y + lens.cy};
}

bool ToRay(const LensModel& lens, const Point2& pixel, Point2& ray) {
  if (!(lens.fx > 0.0) || !(lens.fy > 0.0)) return false;
  const Point2 target{(pixel.x - lens.cx) / lens.fx, (pixel.y - lens.cy) / lens.fy};
  // The distortion is small against the direction itself: correct the guess by what it misses until it stands.
  Point2 guess = target;
  for (int i = 0; i < kMaxIterations; i++) {
    const Point2 d = Distort(lens, guess);
    const double ex = target.x - d.x;
    const double ey = target.y - d.y;
    guess.x += ex;
    guess.y += ey;
    if (std::fabs(ex) < kSettled && std::fabs(ey) < kSettled) {
      ray = guess;
      return true;
    }
  }
  return false;
}

}  // namespace tron_vision
