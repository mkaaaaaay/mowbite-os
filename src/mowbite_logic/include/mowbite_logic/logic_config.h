// What logic_node reads when it starts: its parameters and the id of this installation.
#ifndef MOWBITE_LOGIC_LOGIC_CONFIG_H
#define MOWBITE_LOGIC_LOGIC_CONFIG_H

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "mowbite_logic/logic_core.h"

namespace mowbite_logic {

// A number of the platform's file (parameters geometry.*); one that is missing stops the start.
double GeometryNumber(rclcpp::Node& node, const std::string& key);
// The parameters of the logic; a value out of its range stops the start.
LogicConfig LoadLogicConfig(rclcpp::Node& node);
// by of recorded_body: a random id of this installation in <data_dir>/recorded_by, made once. Without a file that can
// be written it lives until the next start (problem says why when data_dir is set), and areas recorded before do not
// follow robot.geometry.set.
std::string LoadRecordedBy(const std::string& data_dir, std::string* problem);

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_LOGIC_CONFIG_H
