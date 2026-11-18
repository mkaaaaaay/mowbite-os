#include "mowbite_obstacles/ground.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mowbite_obstacles {

namespace {

// Step of the numeric derivatives (rad, m).
constexpr double kStep = 1e-6;
constexpr double kTiny = 1e-12;

double Dot(const Vec3& a, const Vec3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 Origin(const Mount& mount) {
  return {mount.x, mount.y, mount.z};
}

// Rz(yaw) * Ry(pitch) * Rx(roll) applied to v.
Vec3 Rotate(const Mount& mount, const Vec3& v) {
  const double cr = std::cos(mount.roll), sr = std::sin(mount.roll);
  const double cp = std::cos(mount.pitch), sp = std::sin(mount.pitch);
  const double cy = std::cos(mount.yaw), sy = std::sin(mount.yaw);
  const Vec3 a{v.x, cr * v.y - sr * v.z, sr * v.y + cr * v.z};
  const Vec3 b{cp * a.x + sp * a.z, a.y, -sp * a.x + cp * a.z};
  return {cy * b.x - sy * b.y, sy * b.x + cy * b.y, b.z};
}

// The inverse of Rotate.
Vec3 RotateBack(const Mount& mount, const Vec3& v) {
  const double cr = std::cos(mount.roll), sr = std::sin(mount.roll);
  const double cp = std::cos(mount.pitch), sp = std::sin(mount.pitch);
  const double cy = std::cos(mount.yaw), sy = std::sin(mount.yaw);
  const Vec3 b{cy * v.x + sy * v.y, -sy * v.x + cy * v.y, v.z};
  const Vec3 a{cp * b.x - sp * b.z, b.y, sp * b.x + cp * b.z};
  return {a.x, cr * a.y + sr * a.z, -sr * a.y + cr * a.z};
}

double Horizontal(const Vec3& from, const Vec3& to) {
  return std::hypot(to.x - from.x, to.y - from.y);
}

// Adds the spread of one input to a covariance: its derivative times its sigma, squared.
void AddSpread(Spot* spot, double dx, double dy, double sigma) {
  spot->cov_xx += dx * dx * sigma * sigma;
  spot->cov_xy += dx * dy * sigma * sigma;
  spot->cov_yy += dy * dy * sigma * sigma;
}

}  // namespace

Plane FlatGround(double wheel_radius) {
  return {{0.0, 0.0, 1.0}, -wheel_radius};
}

Vec3 RayInBase(const Mount& mount, const Ray& ray) {
  // The camera frame (x right, y down, z ahead) in base_link: ahead is x, right is -y, down is -z.
  return Rotate(mount, {1.0, -ray.x, -ray.y});
}

bool RayOf(const Mount& mount, const Vec3& point, Ray* ray) {
  const Vec3 v = RotateBack(mount, {point.x - mount.x, point.y - mount.y, point.z - mount.z});
  if (v.x <= kTiny) return false;
  *ray = {-v.y / v.x, -v.z / v.x};
  return true;
}

bool HitGround(const Mount& mount, const Plane& ground, const Ray& ray, Vec3* point) {
  const Vec3 direction = RayInBase(mount, ray);
  const Vec3 origin = Origin(mount);
  const double rate = Dot(ground.normal, direction);
  const double above = Dot(ground.normal, origin) - ground.offset;
  if (rate > -kTiny || above <= 0.0) return false;
  const double t = -above / rate;
  *point = {origin.x + t * direction.x, origin.y + t * direction.y, origin.z + t * direction.z};
  return true;
}

Footprint FootprintOf(const Mount& mount, const Plane& ground, const BoxRays& box, const Errors& errors,
                      double range_max) {
  Footprint out;
  Vec3 foot;
  if (!HitGround(mount, ground, box.bottom, &foot)) {
    out.result = GroundResult::kAboveHorizon;
    out.height = std::numeric_limits<double>::quiet_NaN();
    return out;
  }
  const Vec3 origin = Origin(mount);
  out.range = Horizontal(origin, foot);
  out.result = out.range > range_max ? GroundResult::kTooFar : GroundResult::kOk;

  Vec3 left, right;
  if (HitGround(mount, ground, box.bottom_left, &left) && HitGround(mount, ground, box.bottom_right, &right)) {
    // The corners see the sides of a round thing tangentially: its middle lies one radius behind the near side.
    const double half = 0.5 * std::hypot(left.x - right.x, left.y - right.y);
    const double sine = std::sin(std::atan2(half, out.range));
    out.radius = sine < 0.9 ? out.range * sine / (1.0 - sine) : half;
  }
  // The lower edge is the near side of the thing; its middle lies one radius farther along the view.
  const double shift = out.range > kTiny ? out.radius / out.range : 0.0;
  out.spot.x = foot.x + (foot.x - origin.x) * shift;
  out.spot.y = foot.y + (foot.y - origin.y) * shift;

  // The upper edge where the thing stands: the point of its ray as far from the camera as the footprint.
  out.height = std::numeric_limits<double>::quiet_NaN();
  const Vec3 up = RayInBase(mount, box.top);
  const double reach = std::hypot(up.x, up.y);
  if (reach > kTiny) {
    const double top_z = origin.z + up.z * out.range / reach;
    out.height = std::max(0.0, top_z - foot.z);
  }

  // Spread of the footprint: the lower edge off in each direction, the pitch and the height of the camera.
  const auto moved = [&](const Mount& m, const Ray& r, double* dx, double* dy, double step) {
    Vec3 p;
    if (!HitGround(m, ground, r, &p)) return false;
    *dx = (p.x - foot.x) / step;
    *dy = (p.y - foot.y) / step;
    return true;
  };
  double dx = 0.0, dy = 0.0;
  if (moved(mount, {box.bottom.x + kStep, box.bottom.y}, &dx, &dy, kStep)) AddSpread(&out.spot, dx, dy, errors.ray);
  if (moved(mount, {box.bottom.x, box.bottom.y + kStep}, &dx, &dy, kStep)) AddSpread(&out.spot, dx, dy, errors.ray);
  Mount tilted = mount;
  tilted.pitch += kStep;
  if (moved(tilted, box.bottom, &dx, &dy, kStep)) AddSpread(&out.spot, dx, dy, errors.pitch);
  Mount raised = mount;
  raised.z += kStep;
  if (moved(raised, box.bottom, &dx, &dy, kStep)) AddSpread(&out.spot, dx, dy, errors.height);
  return out;
}

Spot StereoSpot(const Mount& mount, const Ray& ray, double depth, double depth_sigma, double ray_sigma) {
  const auto at = [&mount](const Ray& r, double d) {
    const Vec3 v = RayInBase(mount, r);
    return Vec3{mount.x + d * v.x, mount.y + d * v.y, mount.z + d * v.z};
  };
  const Vec3 p = at(ray, depth);
  Spot spot{p.x, p.y, 0.0, 0.0, 0.0};
  const Vec3 deeper = at(ray, depth + kStep);
  AddSpread(&spot, (deeper.x - p.x) / kStep, (deeper.y - p.y) / kStep, depth_sigma);
  const Vec3 across = at({ray.x + kStep, ray.y}, depth);
  AddSpread(&spot, (across.x - p.x) / kStep, (across.y - p.y) / kStep, ray_sigma);
  const Vec3 down = at({ray.x, ray.y + kStep}, depth);
  AddSpread(&spot, (down.x - p.x) / kStep, (down.y - p.y) / kStep, ray_sigma);
  return spot;
}

Fused Fuse(const Spot& a, const Spot& b, double gate) {
  const double sxx = a.cov_xx + b.cov_xx;
  const double sxy = a.cov_xy + b.cov_xy;
  const double syy = a.cov_yy + b.cov_yy;
  const double det = sxx * syy - sxy * sxy;
  const double ex = b.x - a.x;
  const double ey = b.y - a.y;
  Fused out;
  if (det > kTiny * kTiny) {
    const double distance = (syy * ex * ex - 2.0 * sxy * ex * ey + sxx * ey * ey) / det;
    out.agrees = distance <= gate;
  } else {
    out.agrees = std::hypot(ex, ey) < 1e-9;
  }
  if (out.agrees && det > kTiny * kTiny) {
    // K = Ca * S^-1; the fused spot is a + K (b - a) with covariance Ca - K Ca.
    const double ixx = syy / det, ixy = -sxy / det, iyy = sxx / det;
    const double kxx = a.cov_xx * ixx + a.cov_xy * ixy, kxy = a.cov_xx * ixy + a.cov_xy * iyy;
    const double kyx = a.cov_xy * ixx + a.cov_yy * ixy, kyy = a.cov_xy * ixy + a.cov_yy * iyy;
    out.spot.x = a.x + kxx * ex + kxy * ey;
    out.spot.y = a.y + kyx * ex + kyy * ey;
    out.spot.cov_xx = a.cov_xx - (kxx * a.cov_xx + kxy * a.cov_xy);
    out.spot.cov_xy = a.cov_xy - (kxx * a.cov_xy + kxy * a.cov_yy);
    out.spot.cov_yy = a.cov_yy - (kyx * a.cov_xy + kyy * a.cov_yy);
    return out;
  }
  if (out.agrees) {
    out.spot = a;
    return out;
  }
  out.spot = std::hypot(a.x, a.y) <= std::hypot(b.x, b.y) ? a : b;
  out.spot.cov_xx += ex * ex;
  out.spot.cov_xy += ex * ey;
  out.spot.cov_yy += ey * ey;
  return out;
}

bool ViewOf(const Mount& mount, const Plane& ground, const Ray& left_edge, const Ray& right_edge,
            const Ray& bottom_middle, double range_max, View* view) {
  Vec3 near;
  if (!HitGround(mount, ground, bottom_middle, &near)) return false;
  const Vec3 left = RayInBase(mount, left_edge);
  const Vec3 right = RayInBase(mount, right_edge);
  view->x = mount.x;
  view->y = mount.y;
  view->angle_left = std::atan2(left.y, left.x);
  view->angle_right = std::atan2(right.y, right.x);
  view->range_min = Horizontal(Origin(mount), near);
  view->range_max = range_max;
  return true;
}

}  // namespace mowbite_obstacles
