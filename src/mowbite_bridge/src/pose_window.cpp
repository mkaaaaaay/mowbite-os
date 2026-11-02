#include "mowbite_bridge/pose_window.h"

#include <algorithm>
#include <cmath>

namespace mowbite_bridge {

namespace {

double Wrapped(double angle) {
  return std::remainder(angle, 2.0 * M_PI);
}

double UpperMiddle(std::vector<double>& values) {
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), middle, values.end());
  return *middle;
}

}  // namespace

void PoseWindow::Add(const PlanePose& pose) {
  if (capacity_ == 0) return;
  if (poses_.size() < capacity_) {
    poses_.push_back(pose);
    return;
  }
  // Full: a ring, first_ marks the oldest.
  poses_[first_] = pose;
  first_ = (first_ + 1) % capacity_;
}

std::optional<PlanePose> PoseWindow::Take() {
  if (poses_.empty()) return std::nullopt;
  const double reference = poses_[first_].heading;
  std::vector<double> xs, ys, turns;
  xs.reserve(poses_.size());
  ys.reserve(poses_.size());
  turns.reserve(poses_.size());
  for (const auto& pose : poses_) {
    xs.push_back(pose.x);
    ys.push_back(pose.y);
    turns.push_back(Wrapped(pose.heading - reference));
  }
  poses_.clear();
  first_ = 0;
  return PlanePose{UpperMiddle(xs), UpperMiddle(ys), Wrapped(reference + UpperMiddle(turns))};
}

}  // namespace mowbite_bridge
