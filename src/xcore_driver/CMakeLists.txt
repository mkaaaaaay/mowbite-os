cmake_minimum_required(VERSION 3.8)
project(xcore_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mowbite_component REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(mowbite_msgs REQUIRED)
find_package(mowbite_gnss REQUIRED)
find_package(mowbite_hold REQUIRED)
find_package(mowbite_outbox REQUIRED)
find_package(mowbite_watch REQUIRED)

# Frames, CBOR, service descriptions and control logic without ROS, testable on any machine.
add_library(xcore_core STATIC src/wire.cpp src/cbor.cpp src/schema.cpp src/services.cpp src/odometry.cpp src/xcore_control.cpp src/udp.cpp src/drive_regulator.cpp src/input_config.cpp src/link.cpp src/gnss.cpp src/socket_table.cpp)
target_include_directories(xcore_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(xcore_core mowbite_gnss::mowbite_gnss_core mowbite_hold::mowbite_hold_core)
set_target_properties(xcore_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The I/O thread with sockets, link and control, also without ROS: it cannot publish or log by itself.
add_library(xcore_io STATIC src/xcore_io.cpp)
target_link_libraries(xcore_io xcore_core mowbite_outbox::mowbite_outbox_core)
set_target_properties(xcore_io PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(xcore_driver_component SHARED src/xcore_node.cpp)
target_link_libraries(xcore_driver_component xcore_io mowbite_hold::mowbite_hold_core mowbite_watch::mowbite_watch)
# Only the image of the safety measurement (docker/measure.Dockerfile): the driver takes measurement_speed_limit as its
# cap for driving, up to 0.6 m/s; every other build drives at most 0.3 m/s.
option(XCORE_MEASUREMENT_BUILD "driver of the measurement image, see docker/measure.Dockerfile" OFF)
target_compile_definitions(xcore_driver_component PRIVATE XCORE_MEASUREMENT_BUILD=$<BOOL:${XCORE_MEASUREMENT_BUILD}>)
ament_target_dependencies(xcore_driver_component rclcpp rclcpp_components geometry_msgs sensor_msgs diagnostic_msgs mowbite_msgs)
mowbite_component_register(xcore_driver_component
  PLUGIN "xcore_driver::XcoreNode"
  EXECUTABLE xcore_driver_node)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(TARGETS xcore_core xcore_io xcore_driver_component
  EXPORT export_xcore_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(xcore_test test/test_wire.cpp test/test_cbor.cpp test/test_schema.cpp test/test_odometry.cpp test/test_xcore_control.cpp test/test_udp.cpp test/test_drive_regulator.cpp test/test_input_config.cpp test/test_link.cpp test/test_gnss.cpp test/test_socket_table.cpp)
  target_link_libraries(xcore_test xcore_core)
  ament_add_gtest(xcore_io_test test/test_xcore_io.cpp TIMEOUT 120)
  target_link_libraries(xcore_io_test xcore_io)
endif()

ament_export_targets(export_xcore_driver HAS_LIBRARY_TARGET)
ament_export_dependencies(mowbite_gnss mowbite_outbox mowbite_hold)
ament_package()
