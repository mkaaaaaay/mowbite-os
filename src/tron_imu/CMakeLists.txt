cmake_minimum_required(VERSION 3.8)
project(tron_imu)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tron_mcu REQUIRED)

# WIT parser and mounting rotation without ROS, testable on any machine.
add_library(tron_imu_core STATIC src/wit_parser.cpp src/wit_imu.cpp)
target_include_directories(tron_imu_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(tron_imu_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(tron_imu_component SHARED src/imu_node.cpp)
target_link_libraries(tron_imu_component tron_imu_core tron_mcu::tron_mcu_core)
ament_target_dependencies(tron_imu_component rclcpp rclcpp_components sensor_msgs)
rclcpp_components_register_node(tron_imu_component
  PLUGIN "tron_imu::ImuNode"
  EXECUTABLE tron_imu_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS tron_imu_core tron_imu_component
  EXPORT export_tron_imu
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(tron_imu_test test/test_wit_parser.cpp test/test_wit_imu.cpp)
  target_link_libraries(tron_imu_test tron_imu_core)
endif()

ament_export_targets(export_tron_imu HAS_LIBRARY_TARGET)
ament_package()
