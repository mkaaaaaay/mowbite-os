cmake_minimum_required(VERSION 3.8)
project(tron_gnss)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mowbite_msgs REQUIRED)
find_package(tron_mcu REQUIRED)
find_package(mowbite_gnss REQUIRED)

# NMEA parser and tracker without ROS, testable on any machine.
add_library(tron_gnss_core STATIC src/nmea.cpp src/gnss_tracker.cpp src/board_link.cpp)
target_include_directories(tron_gnss_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(tron_gnss_core mowbite_gnss::mowbite_gnss_core)
set_target_properties(tron_gnss_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(tron_gnss_component SHARED src/gnss_node.cpp)
target_link_libraries(tron_gnss_component tron_gnss_core tron_mcu::tron_mcu_core)
ament_target_dependencies(tron_gnss_component rclcpp rclcpp_components sensor_msgs std_msgs mowbite_msgs)
rclcpp_components_register_node(tron_gnss_component
  PLUGIN "tron_gnss::GnssNode"
  EXECUTABLE tron_gnss_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS tron_gnss_core tron_gnss_component
  EXPORT export_tron_gnss
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(tron_gnss_test test/test_nmea.cpp test/test_gnss_tracker.cpp
    test/test_board_link.cpp)
  target_link_libraries(tron_gnss_test tron_gnss_core)
endif()

ament_export_targets(export_tron_gnss HAS_LIBRARY_TARGET)
ament_export_dependencies(mowbite_gnss)
ament_package()
