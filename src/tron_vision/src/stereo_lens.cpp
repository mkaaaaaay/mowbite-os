#include "tron_vision/stereo_lens.h"

#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <vector>

namespace tron_vision {

namespace {

constexpr double kMaxTheta = 89.0 * M_PI / 180.0;
constexpr int kNewtonSteps = 30;
constexpr double kTolerance = 1e-10;

double Distorted(const FisheyeLens& l, double theta) {
  const double t2 = theta * theta;
  return theta * (1.0 + t2 * (l.k1 + t2 * (l.k2 + t2 * (l.k3 + t2 * l.k4))));
}

double Slope(const FisheyeLens& l, double theta) {
  const double t2 = theta * theta;
  return 1.0 + t2 * (3.0 * l.k1 + t2 * (5.0 * l.k2 + t2 * (7.0 * l.k3 + t2 * 9.0 * l.k4)));
}

bool CheckLens(const char* name, const FisheyeLens& l, uint32_t width, uint32_t height, std::string& error) {
  if (!(l.fx > 0.0 && l.fy > 0.0) || !(l.cx > 0.0 && l.cx < width) || !(l.cy > 0.0 && l.cy < height)) {
    error = std::string(name) + ": focal length or center outside the image";
    return false;
  }
  // The middles of the edges must map back to rays; the corners may lie beyond what the model holds.
  const double w = width - 1.0;
  const double h = height - 1.0;
  for (const Point2& p : {Point2{0.0, l.cy}, Point2{w, l.cy}, Point2{l.cx, 0.0}, Point2{l.cx, h}}) {
    Point2 ray;
    if (!ToRay(l, p, ray)) {
      error = std::string(name) + ": the distortion folds back inside the image";
      return false;
    }
  }
  return true;
}

}  // namespace

Point2 ToPixel(const FisheyeLens& lens, const Point2& ray) {
  const double r = std::hypot(ray.x, ray.y);
  const double scale = r > 1e-12 ? Distorted(lens, std::atan(r)) / r : 1.0;
  return {lens.fx * ray.x * scale + lens.cx, lens.fy * ray.y * scale + lens.cy};
}

bool ToRay(const FisheyeLens& lens, const Point2& pixel, Point2& ray) {
  if (!(lens.fx > 0.0 && lens.fy > 0.0)) return false;
  const double xd = (pixel.x - lens.cx) / lens.fx;
  const double yd = (pixel.y - lens.cy) / lens.fy;
  const double rd = std::hypot(xd, yd);
  if (rd < 1e-12) {
    ray = {xd, yd};
    return true;
  }
  double theta = std::min(rd, kMaxTheta);
  bool converged = false;
  for (int i = 0; i < kNewtonSteps; i++) {
    const double slope = Slope(lens, theta);
    if (slope <= 0.0) return false;
    const double step = (Distorted(lens, theta) - rd) / slope;
    theta -= step;
    if (!(theta > 0.0 && theta < kMaxTheta)) return false;
    if (std::abs(step) < kTolerance) {
      converged = true;
      break;
    }
  }
  if (!converged || Slope(lens, theta) <= 0.0) return false;
  const double scale = std::tan(theta) / rd;
  ray = {xd * scale, yd * scale};
  return true;
}

bool CheckStereoCalibration(const StereoCalibration& c, std::string& error) {
  if (c.width == 0 || c.height == 0) {
    error = "no image size";
    return false;
  }
  if (!CheckLens("left", c.left, c.width, c.height, error) || !CheckLens("right", c.right, c.width, c.height, error)) {
    return false;
  }
  const double* r = c.rotation;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      const double dot = r[3 * i] * r[3 * j] + r[3 * i + 1] * r[3 * j + 1] + r[3 * i + 2] * r[3 * j + 2];
      if (std::abs(dot - (i == j ? 1.0 : 0.0)) > 1e-3) {
        error = "rotation is not a rotation";
        return false;
      }
    }
  }
  const double baseline = std::sqrt(c.translation[0] * c.translation[0] + c.translation[1] * c.translation[1] +
                                    c.translation[2] * c.translation[2]);
  // The right eye lies to the right of the left one, a few cm away.
  if (!(baseline > 0.02 && baseline < 0.2) || !(c.translation[0] < 0.0)) {
    error = "translation does not put the right eye 2 to 20 cm to the right";
    return false;
  }
  return true;
}

bool ReadStereoCalibration(const std::string& path, StereoCalibration& calibration, std::string& error) {
  std::ifstream in(path);
  if (!in) {
    error = path + " missing";
    return false;
  }
  const std::map<std::string, size_t> counts = {{"size", 2}, {"left", 8}, {"right", 8}, {"rotation", 9},
                                                {"translation", 3}};
  std::map<std::string, std::vector<double>> values;
  std::string line;
  int number = 0;
  while (std::getline(in, line)) {
    number++;
    line = line.substr(0, line.find('#'));
    std::istringstream words(line);
    std::string name;
    if (!(words >> name)) continue;
    const auto expected = counts.find(name);
    if (expected == counts.end() || values.count(name) > 0) {
      error = path + ":" + std::to_string(number) + ": unknown or repeated " + name;
      return false;
    }
    std::vector<double> numbers;
    double v = 0.0;
    while (words >> v) numbers.push_back(v);
    if (!words.eof() || numbers.size() != expected->second) {
      error = path + ":" + std::to_string(number) + ": " + name + " needs " + std::to_string(expected->second) +
              " numbers";
      return false;
    }
    values[name] = numbers;
  }
  for (const auto& [name, count] : counts) {
    if (values.count(name) == 0) {
      error = path + ": no " + name;
      return false;
    }
  }
  StereoCalibration c;
  const auto& size = values["size"];
  if (!(size[0] >= 1.0 && size[0] <= 8192.0 && size[1] >= 1.0 && size[1] <= 8192.0)) {
    error = path + ": size out of range";
    return false;
  }
  c.width = static_cast<uint32_t>(size[0]);
  c.height = static_cast<uint32_t>(size[1]);
  const auto lens = [](const std::vector<double>& v) {
    return FisheyeLens{v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]};
  };
  c.left = lens(values["left"]);
  c.right = lens(values["right"]);
  for (int i = 0; i < 9; i++) c.rotation[i] = values["rotation"][i];
  for (int i = 0; i < 3; i++) c.translation[i] = values["translation"][i];
  if (!CheckStereoCalibration(c, error)) {
    error = path + ": " + error;
    return false;
  }
  calibration = c;
  return true;
}

bool ToRightPixel(const StereoCalibration& c, const Point2& left_ray, double depth, Point2& pixel) {
  const double p[3] = {left_ray.x * depth, left_ray.y * depth, depth};
  const double* r = c.rotation;
  const double x = r[0] * p[0] + r[1] * p[1] + r[2] * p[2] + c.translation[0];
  const double y = r[3] * p[0] + r[4] * p[1] + r[5] * p[2] + c.translation[1];
  const double z = r[6] * p[0] + r[7] * p[1] + r[8] * p[2] + c.translation[2];
  if (z <= 1e-6) return false;
  pixel = ToPixel(c.right, {x / z, y / z});
  return true;
}

double DisparityAtOneMeter(const StereoCalibration& c) {
  return c.left.fx * std::sqrt(c.translation[0] * c.translation[0] + c.translation[1] * c.translation[1] +
                               c.translation[2] * c.translation[2]);
}

}  // namespace tron_vision
