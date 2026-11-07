cmake_minimum_required(VERSION 3.8)
project(tron_mcu)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(mowbite_msgs REQUIRED)
find_package(mowbite_hold REQUIRED)
find_package(mowbite_journal REQUIRED)

# Codec, control logic and port without ROS, testable on any machine.
add_library(tron_mcu_core STATIC src/mcu_codec.cpp src/mcu_control.cpp src/serial_port.cpp)
target_include_directories(tron_mcu_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(tron_mcu_core mowbite_hold::mowbite_hold_core)
set_target_properties(tron_mcu_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(tron_mcu_component SHARED src/mcu_node.cpp)
target_link_libraries(tron_mcu_component tron_mcu_core mowbite_hold::mowbite_hold_core mowbite_journal::mowbite_journal)
ament_target_dependencies(tron_mcu_component rclcpp rclcpp_components geometry_msgs sensor_msgs diagnostic_msgs mowbite_msgs)
rclcpp_components_register_node(tron_mcu_component
  PLUGIN "tron_mcu::McuNode"
  EXECUTABLE tron_mcu_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS tron_mcu_core tron_mcu_component
  EXPORT export_tron_mcu
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(tron_mcu_test test/test_mcu_codec.cpp test/test_mcu_control.cpp)
  target_link_libraries(tron_mcu_test tron_mcu_core)
endif()

ament_export_targets(export_tron_mcu HAS_LIBRARY_TARGET)
ament_export_dependencies(mowbite_hold)
ament_package()
