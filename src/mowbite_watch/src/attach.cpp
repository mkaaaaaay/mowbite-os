#include "mowbite_watch/attach.h"

#include <pthread.h>

#include <chrono>
#include <mutex>
#include <string>

namespace mowbite_watch {
namespace {

// The thread name the kernel keeps (15 characters), without the prefix all nodes of MowBite OS share.
std::string ThreadName(const std::string& node) {
  const std::string prefix = "mowbite_";
  const std::string name = node.compare(0, prefix.size(), prefix) == 0 ? node.substr(prefix.size()) : node;
  return name.substr(0, 15);
}

int64_t SteadyNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

}  // namespace

Attached::Attached(rclcpp::Node& node, double limit_s)
    : watch_(node.get_name(), limit_s),
      limit_s_(limit_s),
      logger_(node.get_logger()),
      thread_name_(ThreadName(node.get_name())) {
  // Once per process: from the shutdown of ROS on only the shutdown limit counts.
  static std::once_flag once;
  static rclcpp::PreShutdownCallbackHandle shutdown;
  std::call_once(once, [&node] {
    shutdown = node.get_node_base_interface()->get_context()->add_pre_shutdown_callback(
        [] { Runtime::Process().Shutdown(); });
  });
  if (Runtime::Process().factor() > 1.0) {
    RCLCPP_WARN(logger_, "Watch limit %.1f s times %.1f (sanitizer run)", limit_s_, Runtime::Process().factor());
  }
  timer_ = node.create_wall_timer(std::chrono::duration<double>(kPeriod), [this] { Tick(); });
}

void Attached::Tick() {
  const int64_t now = SteadyNs();
  if (last_ns_ == 0) {
    pthread_setname_np(pthread_self(), thread_name_.c_str());
  } else {
    const double late = static_cast<double>(now - last_ns_) * 1e-9 - kPeriod;
    if (late > limit_s_ / 4) RCLCPP_WARN(logger_, "Watch: sign %.2f s late (limit %.1f s)", late, limit_s_);
  }
  last_ns_ = now;
  watch_.Sign();
}

std::unique_ptr<Attached> Attach(rclcpp::Node& node, double limit_s) {
  return std::make_unique<Attached>(node, limit_s);
}

}  // namespace mowbite_watch
