cmake_minimum_required(VERSION 3.8)
project(mowbite_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AbsolutePose.msg"
  "msg/HardwareCapabilities.msg"
  "msg/HardwareStatus.msg"
  "msg/MapStatus.msg"
  "msg/MotorState.msg"
  "msg/RobotState.msg"
  "msg/SensorInfo.msg"
  "msg/SensorValue.msg"
  "msg/ActionInfo.msg"
  "msg/ActionList.msg"
  "msg/RpcMethods.msg"
  "msg/RpcRequest.msg"
  "msg/RpcResponse.msg"
  "msg/RpcError.msg"
  "msg/MqttPublish.msg"
  "msg/PositionJump.msg"
  "msg/Outline.msg"
  "srv/AddArea.srv"
  "srv/ReplaceMap.srv"
  "srv/SetBlade.srv"
  "srv/SetDock.srv"
  "srv/SetEmergency.srv"
  "srv/SetPose.srv"
  "action/FollowPath.action"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs diagnostic_msgs action_msgs
)

install(FILES LICENSE.xbot_monitoring DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
