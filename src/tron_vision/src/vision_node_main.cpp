#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "tron_vision/vision_node.h"

// Ends with the exit code of the loop (3 memory, 4 NPU), which the launch notes before it starts the process again.
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tron_vision::VisionNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  const int code = node->ExitCode();
  node.reset();
  rclcpp::shutdown();
  return code;
}
