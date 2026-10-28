// Median filter for the poses that arrive between two position/json messages. Based on pose_publish_timer_callback
// in xbot_monitoring.cpp (OpenMower), MIT License, Copyright (c) 2022 Clemens Elflein. See LICENSE.xbot_monitoring.
#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace mowbite_bridge {

struct PoseSample {
  double x = 0, y = 0, heading = 0;
};

inline double NormalizeAngle(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a <= 0) a += 2.0 * M_PI;
  return a - M_PI;
}

// Median of x, y and heading, taken separately. The heading is measured against the first sample, so a turn across
// +-pi gives a heading near pi and not the one of the other side. Nothing for an empty list.
inline std::optional<PoseSample> MedianPose(const std::vector<PoseSample>& samples) {
  if (samples.empty()) return std::nullopt;
  const size_t n = samples.size();
  const size_t mid = n / 2;
  const double reference = samples.front().heading;
  std::vector<double> xs(n), ys(n), hs(n);
  for (size_t i = 0; i < n; ++i) {
    xs[i] = samples[i].x;
    ys[i] = samples[i].y;
    hs[i] = NormalizeAngle(samples[i].heading - reference);
  }
  std::nth_element(xs.begin(), xs.begin() + static_cast<std::ptrdiff_t>(mid), xs.end());
  std::nth_element(ys.begin(), ys.begin() + static_cast<std::ptrdiff_t>(mid), ys.end());
  std::nth_element(hs.begin(), hs.begin() + static_cast<std::ptrdiff_t>(mid), hs.end());
  return PoseSample{xs[mid], ys[mid], NormalizeAngle(reference + hs[mid])};
}

}  // namespace mowbite_bridge
