#ifndef MOWBITE_WATCH_ATTACH_H
#define MOWBITE_WATCH_ATTACH_H

#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "mowbite_watch/watch.h"

namespace mowbite_watch {

// A node in the watch of its process: a timer in its own callbacks signs every kPeriod, names the thread after the
// node at its first run and logs a sign later than a quarter of the limit. Leaves the watch when destroyed.
class Attached {
 public:
  Attached(rclcpp::Node& node, double limit_s);

  // What the node does now, for the line should it stall; where must outlive the node.
  void Mark(const char* where) { watch_.Mark(where); }

 private:
  void Tick();

  Watch watch_;
  const double limit_s_;
  const rclcpp::Logger logger_;
  const std::string thread_name_;
  int64_t last_ns_ = 0;
  rclcpp::TimerBase::SharedPtr timer_;
};

// One call per node in a container, at the end of its constructor, so its own start does not count as silence.
std::unique_ptr<Attached> Attach(rclcpp::Node& node, double limit_s = kLimit);

}  // namespace mowbite_watch

#endif  // MOWBITE_WATCH_ATTACH_H
