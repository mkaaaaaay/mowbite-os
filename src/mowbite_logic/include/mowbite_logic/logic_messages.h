// What logic_node turns messages into for the LogicCore, and back.
#ifndef MOWBITE_LOGIC_LOGIC_MESSAGES_H
#define MOWBITE_LOGIC_LOGIC_MESSAGES_H

#include <cstdint>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <mowbite_msgs/msg/motor_state.hpp>
#include <mowbite_msgs/msg/robot_pose.hpp>
#include <string>

#include "mowbite_logic/logic_core.h"

namespace mowbite_logic {

MapDatum ToDatum(uint8_t state);
geometry_msgs::msg::Polygon ToPolygon(const Ring& ring);
geometry_msgs::msg::Pose PoseAt(double x, double y, double heading);
double Yaw(const geometry_msgs::msg::Quaternion& q);
// How a drive of the follower ended, for the job: a bad position goes on by itself, the rest waits for continue.
LegEnd ToLegEnd(uint8_t reason);
// The name of a result of the follower, as in FollowPath.action.
std::string FollowerCode(uint8_t reason);
HardwareSnapshot::Motor ToMotor(const mowbite_msgs::msg::MotorState& m);
PoseSnapshot ToPose(const mowbite_msgs::msg::RobotPose& msg);

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_LOGIC_MESSAGES_H
