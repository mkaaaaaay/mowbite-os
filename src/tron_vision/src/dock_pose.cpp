#include "tron_vision/dock_pose.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace tron_vision {

namespace {

constexpr size_t kUnknowns = 5;  // x, y, z, yaw, roll
constexpr size_t kResiduals = 8;
constexpr int kMaxIterations = 60;
constexpr double kStep = 1e-6;
constexpr double kSettled = 1e-10;
constexpr double kMinDepth = 1e-3;
constexpr double kPi = 3.14159265358979323846;

using Vec = std::array<double, kUnknowns>;
using Mat = std::array<double, kUnknowns * kUnknowns>;
using Residuals = std::array<double, kResiduals>;

// Corner offsets along the marker's right and up, in half sides: top left, top right, bottom right, bottom left.
constexpr std::array<std::array<double, 2>, 4> kCornerSigns = {{{-1.0, 1.0}, {1.0, 1.0}, {1.0, -1.0}, {-1.0, -1.0}}};

double Wrap(double angle) {
  return std::remainder(angle, 2.0 * kPi);
}

struct Problem {
  std::array<Point2, 4> corners;
  // base_link from the camera body (x along the view, y left, z up), row by row.
  std::array<double, 9> rotation;
  CameraMount mount;
  double half_side;
};

std::array<double, 9> MountRotation(const CameraMount& mount) {
  const double cy = std::cos(mount.yaw);
  const double sy = std::sin(mount.yaw);
  const double cp = std::cos(mount.pitch);
  const double sp = std::sin(mount.pitch);
  const double cr = std::cos(mount.roll);
  const double sr = std::sin(mount.roll);
  return {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
          sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
          -sp,     cp * sr,                cp * cr};
}

// Predicted minus measured corner directions; false if a corner would lie behind the camera.
bool Evaluate(const Problem& p, const Vec& q, Residuals& e) {
  const double right_x = -std::sin(q[3]);
  const double right_y = std::cos(q[3]);
  const double cr = std::cos(q[4]);
  const double sr = std::sin(q[4]);
  for (size_t k = 0; k < 4; k++) {
    // The corner turned by the roll about the middle of the face.
    const double side = kCornerSigns[k][0] * p.half_side;
    const double high = kCornerSigns[k][1] * p.half_side;
    const double along = side * cr - high * sr;
    const double dx = q[0] + along * right_x - p.mount.x;
    const double dy = q[1] + along * right_y - p.mount.y;
    const double dz = q[2] + side * sr + high * cr - p.mount.z;
    const double view = p.rotation[0] * dx + p.rotation[3] * dy + p.rotation[6] * dz;
    const double left = p.rotation[1] * dx + p.rotation[4] * dy + p.rotation[7] * dz;
    const double up = p.rotation[2] * dx + p.rotation[5] * dy + p.rotation[8] * dz;
    if (view < kMinDepth) return false;
    e[2 * k] = -left / view - p.corners[k].x;
    e[2 * k + 1] = -up / view - p.corners[k].y;
  }
  return true;
}

double SquareSum(const Residuals& e) {
  double sum = 0.0;
  for (const double v : e) sum += v * v;
  return sum;
}

// Gaussian elimination with row pivoting; false if the matrix is singular.
bool Solve(Mat a, Vec b, Vec& x) {
  for (size_t col = 0; col < kUnknowns; col++) {
    size_t pivot = col;
    for (size_t row = col + 1; row < kUnknowns; row++) {
      if (std::fabs(a[row * kUnknowns + col]) > std::fabs(a[pivot * kUnknowns + col])) pivot = row;
    }
    if (std::fabs(a[pivot * kUnknowns + col]) < 1e-300) return false;
    if (pivot != col) {
      for (size_t k = 0; k < kUnknowns; k++) std::swap(a[col * kUnknowns + k], a[pivot * kUnknowns + k]);
      std::swap(b[col], b[pivot]);
    }
    for (size_t row = col + 1; row < kUnknowns; row++) {
      const double factor = a[row * kUnknowns + col] / a[col * kUnknowns + col];
      for (size_t k = col; k < kUnknowns; k++) a[row * kUnknowns + k] -= factor * a[col * kUnknowns + k];
      b[row] -= factor * b[col];
    }
  }
  for (size_t i = kUnknowns; i-- > 0;) {
    double sum = b[i];
    for (size_t k = i + 1; k < kUnknowns; k++) sum -= a[i * kUnknowns + k] * x[k];
    x[i] = sum / a[i * kUnknowns + i];
  }
  return true;
}

using Jacobian = std::array<Residuals, kUnknowns>;

// Derivatives of the residuals by each unknown at q, by central differences.
bool Derive(const Problem& p, const Vec& q, Jacobian& columns) {
  for (size_t u = 0; u < kUnknowns; u++) {
    Vec plus = q;
    Vec minus = q;
    plus[u] += kStep;
    minus[u] -= kStep;
    Residuals e_plus{};
    Residuals e_minus{};
    if (!Evaluate(p, plus, e_plus) || !Evaluate(p, minus, e_minus)) return false;
    for (size_t r = 0; r < kResiduals; r++) columns[u][r] = (e_plus[r] - e_minus[r]) / (2.0 * kStep);
  }
  return true;
}

Mat NormalMatrix(const Jacobian& columns) {
  Mat jtj{};
  for (size_t a = 0; a < kUnknowns; a++) {
    for (size_t b = 0; b < kUnknowns; b++) {
      for (size_t r = 0; r < kResiduals; r++) jtj[a * kUnknowns + b] += columns[a][r] * columns[b][r];
    }
  }
  return jtj;
}

struct Fit {
  bool ok = false;
  Vec q{};
  double cost = 0.0;
  Jacobian columns{};
};

// Damped least squares from the start value to the nearest minimum.
Fit Minimize(const Problem& p, const Vec& start) {
  Fit fit;
  fit.q = start;
  Residuals e{};
  if (!Evaluate(p, fit.q, e)) return fit;
  fit.cost = SquareSum(e);
  double damping = 1e-3;
  for (int i = 0; i < kMaxIterations; i++) {
    if (!Derive(p, fit.q, fit.columns)) return fit;
    const Mat jtj = NormalMatrix(fit.columns);
    bool stepped = false;
    double step_size = 0.0;
    while (!stepped && damping < 1e9) {
      Mat damped = jtj;
      Vec rhs{};
      for (size_t u = 0; u < kUnknowns; u++) {
        damped[u * kUnknowns + u] *= 1.0 + damping;
        for (size_t r = 0; r < kResiduals; r++) rhs[u] -= fit.columns[u][r] * e[r];
      }
      Vec step{};
      Vec next = fit.q;
      Residuals e_next{};
      if (Solve(damped, rhs, step)) {
        for (size_t u = 0; u < kUnknowns; u++) next[u] += step[u];
        if (Evaluate(p, next, e_next) && SquareSum(e_next) <= fit.cost) {
          fit.q = next;
          e = e_next;
          fit.cost = SquareSum(e);
          for (const double s : step) step_size = std::max(step_size, std::fabs(s));
          stepped = true;
          damping *= 0.3;
          continue;
        }
      }
      damping *= 10.0;
    }
    if (!stepped || step_size < kSettled) break;
  }
  fit.ok = Derive(p, fit.q, fit.columns);
  return fit;
}

// Direction from the marker to the camera on the ground plane.
double LineOfSight(const CameraMount& mount, const Vec& q) {
  return std::atan2(mount.y - q[1], mount.x - q[0]);
}

// How the unknowns follow the corner coordinates near the minimum: (J^T J)^-1 J^T, one row per unknown.
bool Gain(const Jacobian& columns, std::array<Residuals, kUnknowns>& gain) {
  const Mat jtj = NormalMatrix(columns);
  for (size_t r = 0; r < kResiduals; r++) {
    Vec rhs{};
    Vec x{};
    for (size_t u = 0; u < kUnknowns; u++) rhs[u] = columns[u][r];
    if (!Solve(jtj, rhs, x)) return false;
    for (size_t u = 0; u < kUnknowns; u++) gain[u][r] = x[u];
  }
  return true;
}

double Distance(const Point2& a, const Point2& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

}  // namespace

MarkerPose FitMarkerPose(const std::array<Point2, 4>& corners, const std::array<double, 64>& corner_covariance,
                         double focal_px, const CameraMount& mount, double marker_side, const PoseLimits& limits) {
  MarkerPose pose;
  if (!(marker_side > 0.0) || !(focal_px > 0.0)) return pose;
  Problem p;
  p.corners = corners;
  p.rotation = MountRotation(mount);
  p.mount = mount;
  p.half_side = marker_side / 2.0;

  // Start: the upright sides give the distance, the middle of the corners the direction, the face towards the camera.
  const double upright = (Distance(corners[1], corners[2]) + Distance(corners[3], corners[0])) / 2.0;
  if (!(upright > 0.0)) return pose;
  const double depth = marker_side / upright;
  const double mid_x = (corners[0].x + corners[1].x + corners[2].x + corners[3].x) / 4.0;
  const double mid_y = (corners[0].y + corners[1].y + corners[2].y + corners[3].y) / 4.0;
  const std::array<double, 3> body = {depth, -mid_x * depth, -mid_y * depth};
  Vec start{};
  start[0] = mount.x + p.rotation[0] * body[0] + p.rotation[1] * body[1] + p.rotation[2] * body[2];
  start[1] = mount.y + p.rotation[3] * body[0] + p.rotation[4] * body[1] + p.rotation[5] * body[2];
  start[2] = mount.z + p.rotation[6] * body[0] + p.rotation[7] * body[1] + p.rotation[8] * body[2];
  start[3] = LineOfSight(mount, start);

  Fit best = Minimize(p, start);
  if (!best.ok) return pose;
  // From far away the square turned by the same angle to the other side of the line of sight looks almost the same.
  const double sight = LineOfSight(mount, best.q);
  Vec mirrored = best.q;
  mirrored[3] = sight - Wrap(best.q[3] - sight);
  Fit other = Minimize(p, mirrored);
  if (other.ok && other.cost < best.cost) std::swap(best, other);

  const double side_px = upright * focal_px;
  pose.fit_error_px = std::sqrt(best.cost / static_cast<double>(kResiduals)) * focal_px;
  pose.x = best.q[0];
  pose.y = best.q[1];
  pose.z = best.q[2];
  pose.yaw = Wrap(best.q[3]);
  pose.roll = Wrap(best.q[4]);
  if (pose.fit_error_px > limits.fit_error_px + limits.fit_error_share * side_px) return pose;
  if (std::fabs(pose.roll) > limits.max_roll) return pose;

  std::array<Residuals, kUnknowns> gain{};
  if (!Gain(best.columns, gain)) return pose;
  // What the corner covariance leaves of the residuals after the fit: its trace without the part the fit absorbs.
  double expected = 0.0;
  for (size_t i = 0; i < kResiduals; i++) {
    expected += corner_covariance[i * kResiduals + i];
    for (size_t k = 0; k < kResiduals; k++) {
      double absorbed = 0.0;
      for (size_t u = 0; u < kUnknowns; u++) absorbed += best.columns[u][i] * gain[u][k];
      expected -= absorbed * corner_covariance[k * kResiduals + i];
    }
  }
  if (!(expected > 0.0)) return pose;
  // A square that fits worse than that widens the covariance instead of hiding in it.
  const double widen = std::max(1.0, best.cost / expected);
  pose.widen = widen;
  constexpr std::array<size_t, 3> kReported = {0, 1, 3};
  for (size_t a = 0; a < 3; a++) {
    for (size_t b = 0; b < 3; b++) {
      double sum = 0.0;
      for (size_t i = 0; i < kResiduals; i++) {
        for (size_t k = 0; k < kResiduals; k++) {
          sum += gain[kReported[a]][i] * corner_covariance[i * kResiduals + k] * gain[kReported[b]][k];
        }
      }
      pose.covariance[a * 3 + b] = widen * sum;
    }
  }
  if (!(pose.covariance[0] > 0.0) || !(pose.covariance[4] > 0.0) || !(pose.covariance[8] > 0.0)) return pose;
  pose.valid = true;

  const double yaw_sigma = std::sqrt(pose.covariance[8]);
  pose.corner_yaw_sigma = yaw_sigma;
  const bool two_minima = other.ok && std::fabs(Wrap(other.q[3] - best.q[3])) > 0.5 * yaw_sigma;
  pose.yaw_valid =
      !two_minima || other.cost - best.cost >= limits.mirror_sigmas * limits.mirror_sigmas * widen * expected /
                                                   static_cast<double>(kResiduals - kUnknowns);
  if (!pose.yaw_valid) {
    for (const size_t i : {2U, 5U, 6U, 7U, 8U}) pose.covariance[i] = 0.0;
    return pose;
  }
  // The yaw and its rows scaled alike keep the correlations and the covariance positive definite.
  const double range = std::hypot(pose.x - mount.x, pose.y - mount.y, pose.z - mount.z);
  if (range <= limits.measured_yaw_range) {
    const double scale = limits.measured_yaw_sigma / yaw_sigma;
    for (const size_t i : {2U, 5U, 6U, 7U}) pose.covariance[i] *= scale;
    pose.covariance[8] = limits.measured_yaw_sigma * limits.measured_yaw_sigma;
  }
  return pose;
}

}  // namespace tron_vision
