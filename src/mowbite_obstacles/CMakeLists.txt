cmake_minimum_required(VERSION 3.8)
project(mowbite_obstacles)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mowbite_api REQUIRED)
find_package(mowbite_bridge REQUIRED)
find_package(mowbite_component REQUIRED)
find_package(mowbite_hold REQUIRED)
find_package(mowbite_map REQUIRED)
find_package(mowbite_msgs REQUIRED)
find_package(mowbite_ntrip REQUIRED)
find_package(mowbite_watch REQUIRED)

# Where a box of a camera stands on the ground, from rays and the mount; for the platforms, without ROS.
add_library(mowbite_obstacles_ground STATIC src/ground.cpp)
target_include_directories(mowbite_obstacles_ground PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(mowbite_obstacles_ground PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Tracks, sight and zones, and the list of the app, without files, clock or ROS.
add_library(mowbite_obstacles_core STATIC src/tracker.cpp src/obstacle_list.cpp)
target_link_libraries(mowbite_obstacles_core PUBLIC mowbite_obstacles_ground nlohmann_json::nlohmann_json)
set_target_properties(mowbite_obstacles_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The list with its photos in data_dir and vision.obstacles.*, without ROS.
add_library(mowbite_obstacles_service STATIC src/obstacle_service.cpp)
target_link_libraries(mowbite_obstacles_service PUBLIC mowbite_obstacles_core mowbite_api::mowbite_api_core
  mowbite_bridge::mowbite_bridge_core mowbite_hold::mowbite_hold_core mowbite_map::mowbite_map_core
  mowbite_ntrip::mowbite_ntrip_core)
set_target_properties(mowbite_obstacles_service PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(mowbite_obstacles_component SHARED src/obstacles_node.cpp)
target_link_libraries(mowbite_obstacles_component mowbite_obstacles_service mowbite_watch::mowbite_watch)
ament_target_dependencies(mowbite_obstacles_component rclcpp rclcpp_components mowbite_msgs std_msgs)
mowbite_component_register(mowbite_obstacles_component
  PLUGIN "mowbite_obstacles::ObstaclesNode"
  EXECUTABLE mowbite_obstacles_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_obstacles_ground mowbite_obstacles_core mowbite_obstacles_service mowbite_obstacles_component
  EXPORT export_mowbite_obstacles
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_obstacles_test test/test_ground.cpp test/test_tracker.cpp test/test_obstacle_list.cpp
    test/test_obstacle_service.cpp)
  target_link_libraries(mowbite_obstacles_test mowbite_obstacles_service)
endif()

ament_export_targets(export_mowbite_obstacles HAS_LIBRARY_TARGET)
ament_export_dependencies(nlohmann_json)
ament_package()
