#ifndef MOWBITE_COMPONENT_NODE_OPTIONS_H
#define MOWBITE_COMPONENT_NODE_OPTIONS_H

#include <rclcpp/node_options.hpp>

namespace mowbite_component {

// Every node of MowBite OS reads its parameters once at the start: no parameter services, no parameter events.
inline rclcpp::NodeOptions StartupOnly(const rclcpp::NodeOptions& options) {
  return rclcpp::NodeOptions(options).start_parameter_services(false).start_parameter_event_publisher(false);
}

}  // namespace mowbite_component

#endif  // MOWBITE_COMPONENT_NODE_OPTIONS_H
