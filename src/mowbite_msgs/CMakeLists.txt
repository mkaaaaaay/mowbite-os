cmake_minimum_required(VERSION 3.8)
project(mowbite_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AbsolutePose.msg"
  "msg/MotorState.msg"
  "msg/McuStatus.msg"
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
  "srv/SetPose.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

install(FILES LICENSE.xbot_monitoring DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
