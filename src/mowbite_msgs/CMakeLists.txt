cmake_minimum_required(VERSION 3.8)
project(mowbite_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
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
  DEPENDENCIES builtin_interfaces
)

install(FILES LICENSE.xbot_monitoring DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
