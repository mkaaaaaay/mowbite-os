#include "mowbite_logic/logic_messages.h"

#include <cmath>

#include <geometry_msgs/msg/point32.hpp>
#include <mowbite_msgs/action/follow_path.hpp>
#include <mowbite_msgs/msg/map_status.hpp>
#include <mowbite_msgs/msg/sensor_spec.hpp>

namespace mowbite_logic {

namespace {

using mowbite_msgs::msg::SensorSpec;
static_assert(static_cast<uint8_t>(SensorMeaning::kUnknown) == SensorSpec::MEANING_UNKNOWN);
static_assert(static_cast<uint8_t>(SensorMeaning::kVoltage) == SensorSpec::MEANING_VOLTAGE);
static_assert(static_cast<uint8_t>(SensorMeaning::kCurrent) == SensorSpec::MEANING_CURRENT);
static_assert(static_cast<uint8_t>(SensorMeaning::kTemperature) == SensorSpec::MEANING_TEMPERATURE);
static_assert(static_cast<uint8_t>(SensorMeaning::kRevolutions) == SensorSpec::MEANING_REVOLUTIONS);
static_assert(static_cast<uint8_t>(SensorMeaning::kDistance) == SensorSpec::MEANING_DISTANCE);

}  // namespace


MapDatum ToDatum(uint8_t state) {
  using mowbite_msgs::msg::MapStatus;
  switch (state) {
    case MapStatus::DATUM_EMPTY: return MapDatum::kEmpty;
    case MapStatus::DATUM_MATCH: return MapDatum::kMatch;
    case MapStatus::DATUM_MISMATCH: return MapDatum::kMismatch;
    case MapStatus::DATUM_UNBOUND: return MapDatum::kUnbound;
    default: return MapDatum::kUnknown;
  }
}

geometry_msgs::msg::Polygon ToPolygon(const Ring& ring) {
  geometry_msgs::msg::Polygon polygon;
  for (const auto& p : ring) {
    geometry_msgs::msg::Point32 point;
    point.x = static_cast<float>(p.x);
    point.y = static_cast<float>(p.y);
    polygon.points.push_back(point);
  }
  return polygon;
}

geometry_msgs::msg::Pose PoseAt(double x, double y, double heading) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.z = std::sin(heading / 2);
  pose.orientation.w = std::cos(heading / 2);
  return pose;
}

double Yaw(const geometry_msgs::msg::Quaternion& q) {
  const double sine = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosine = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(sine, cosine);
}

LegEnd ToLegEnd(uint8_t reason) {
  using Result = mowbite_msgs::action::FollowPath::Result;
  switch (reason) {
    case Result::REASON_DONE: return LegEnd::kDone;
    case Result::REASON_CANCELED: return LegEnd::kCanceled;
    case Result::REASON_EMERGENCY: return LegEnd::kEmergency;
    case Result::REASON_POSE_LOST:
    case Result::REASON_POSITION_ACCURACY:
    case Result::REASON_HEADING: return LegEnd::kGps;
    default: return LegEnd::kFailed;
  }
}

std::string FollowerCode(uint8_t reason) {
  using Result = mowbite_msgs::action::FollowPath::Result;
  switch (reason) {
    case Result::REASON_DONE: return "DONE";
    case Result::REASON_CANCELED: return "CANCELED";
    case Result::REASON_INVALID: return "INVALID";
    case Result::REASON_NOT_ALLOWED: return "NOT_ALLOWED";
    case Result::REASON_EMERGENCY: return "EMERGENCY";
    case Result::REASON_POSE_LOST: return "POSE_LOST";
    case Result::REASON_POSITION_ACCURACY: return "POSITION_ACCURACY";
    case Result::REASON_HEADING: return "HEADING";
    case Result::REASON_CROSS_TRACK: return "CROSS_TRACK";
    case Result::REASON_OUTSIDE: return "OUTSIDE";
    case Result::REASON_TIMEOUT: return "TIMEOUT";
    default: return "INVALID";
  }
}

HardwareSnapshot::Motor ToMotor(const mowbite_msgs::msg::MotorState& m) {
  HardwareSnapshot::Motor motor;
  motor.ok = m.ok;
  motor.rpm = m.rpm;
  motor.current = m.current;
  motor.temperature = m.temperature;
  return motor;
}

PoseSnapshot ToPose(const mowbite_msgs::msg::RobotPose& msg) {
  PoseSnapshot pose;
  pose.x = msg.x;
  pose.y = msg.y;
  pose.heading = msg.heading;
  pose.position_accuracy = msg.position_accuracy;
  pose.orientation_accuracy = msg.heading_accuracy;
  pose.orientation_valid = msg.heading_valid;
  return pose;
}

}  // namespace mowbite_logic
