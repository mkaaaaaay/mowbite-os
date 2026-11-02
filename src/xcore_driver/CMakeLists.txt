cmake_minimum_required(VERSION 3.8)
project(xcore_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(mowbite_msgs REQUIRED)
find_package(mowbite_gnss REQUIRED)

# Frames, CBOR, service descriptions and control logic without ROS, testable on any machine.
add_library(xcore_core STATIC src/wire.cpp src/cbor.cpp src/schema.cpp src/services.cpp src/odometry.cpp src/xcore_control.cpp src/udp.cpp src/drive_regulator.cpp src/input_config.cpp src/link.cpp src/gnss.cpp)
target_include_directories(xcore_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(xcore_core mowbite_gnss::mowbite_gnss_core)
set_target_properties(xcore_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(xcore_driver_component SHARED src/xcore_node.cpp)
target_link_libraries(xcore_driver_component xcore_core)
ament_target_dependencies(xcore_driver_component rclcpp rclcpp_components geometry_msgs sensor_msgs diagnostic_msgs mowbite_msgs)
rclcpp_components_register_node(xcore_driver_component
  PLUGIN "xcore_driver::XcoreNode"
  EXECUTABLE xcore_driver_node)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS xcore_core xcore_driver_component
  EXPORT export_xcore_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(xcore_test test/test_wire.cpp test/test_cbor.cpp test/test_schema.cpp test/test_odometry.cpp test/test_xcore_control.cpp test/test_udp.cpp test/test_drive_regulator.cpp test/test_input_config.cpp test/test_link.cpp test/test_gnss.cpp)
  target_link_libraries(xcore_test xcore_core)
endif()

ament_export_targets(export_xcore_driver HAS_LIBRARY_TARGET)
ament_export_dependencies(mowbite_gnss)
ament_package()
