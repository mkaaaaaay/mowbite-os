cmake_minimum_required(VERSION 3.8)
project(mowbite_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mowbite_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(PahoMqttCpp REQUIRED)

# Payloads and RPC handling without ROS and MQTT, testable on any machine.
add_library(mowbite_bridge_core STATIC src/monitor.cpp src/params_store.cpp)
target_include_directories(mowbite_bridge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowbite_bridge_core nlohmann_json::nlohmann_json)
set_target_properties(mowbite_bridge_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(mowbite_bridge_component SHARED src/bridge_node.cpp)
target_link_libraries(mowbite_bridge_component mowbite_bridge_core PahoMqttCpp::paho-mqttpp3)
ament_target_dependencies(mowbite_bridge_component rclcpp rclcpp_components rcl_interfaces geometry_msgs std_msgs mowbite_msgs)
rclcpp_components_register_node(mowbite_bridge_component
  PLUGIN "mowbite_bridge::BridgeNode"
  EXECUTABLE mowbite_bridge_node)

install(DIRECTORY include/ DESTINATION include)
install(FILES LICENSE.xbot_monitoring DESTINATION share/${PROJECT_NAME})
install(TARGETS mowbite_bridge_core mowbite_bridge_component
  EXPORT export_mowbite_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_bridge_test test/test_monitor.cpp test/test_position.cpp)
  target_link_libraries(mowbite_bridge_test mowbite_bridge_core)
endif()

ament_export_targets(export_mowbite_bridge HAS_LIBRARY_TARGET)
ament_package()
