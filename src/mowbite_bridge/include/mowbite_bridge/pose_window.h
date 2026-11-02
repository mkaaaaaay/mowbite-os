#ifndef MOWBITE_BRIDGE_POSE_WINDOW_H
#define MOWBITE_BRIDGE_POSE_WINDOW_H

#include <cstddef>
#include <optional>
#include <vector>

namespace mowbite_bridge {

struct PlanePose {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
};

// The poses since the last send of position/json, sent as one: x, y and heading each the middle value on its own.
class PoseWindow {
 public:
  explicit PoseWindow(std::size_t capacity = 500) : capacity_(capacity) {}

  // Beyond the capacity the oldest pose goes.
  void Add(const PlanePose& pose);
  // The middle pose of the window, then an empty window; nothing without a pose since the last call. Headings count
  // from the first one, so values on both sides of +-pi stay together; of an even count the upper middle value.
  std::optional<PlanePose> Take();

 private:
  std::size_t capacity_;
  std::size_t first_ = 0;
  std::vector<PlanePose> poses_;
};

}  // namespace mowbite_bridge

#endif  // MOWBITE_BRIDGE_POSE_WINDOW_H
