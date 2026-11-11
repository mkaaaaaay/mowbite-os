#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "tron_vision/rear_node.h"

// Ends with the exit code of the loop (3 memory) or of the watch (86), which the launch notes before it starts the
// process again.
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tron_vision::RearNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  const int code = node->ExitCode();
  node.reset();
  rclcpp::shutdown();
  return code;
}
