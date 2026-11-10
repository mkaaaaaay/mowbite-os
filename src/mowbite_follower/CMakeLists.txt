cmake_minimum_required(VERSION 3.8)
project(mowbite_follower)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mowbite_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mowbite_watch REQUIRED)

# Path tracking and its checks without ROS, testable on any machine.
add_library(mowbite_follower_core STATIC src/path.cpp src/speed_plan.cpp src/steering.cpp src/path_follower.cpp
  src/guard.cpp)
target_include_directories(mowbite_follower_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(mowbite_follower_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(mowbite_follower_component SHARED src/follower_node.cpp)
target_link_libraries(mowbite_follower_component mowbite_follower_core mowbite_watch::mowbite_watch)
ament_target_dependencies(mowbite_follower_component rclcpp rclcpp_action rclcpp_components geometry_msgs mowbite_msgs
  std_msgs)
rclcpp_components_register_node(mowbite_follower_component
  PLUGIN "mowbite_follower::FollowerNode"
  EXECUTABLE mowbite_follower_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_follower_core mowbite_follower_component
  EXPORT export_mowbite_follower
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_follower_test test/test_path.cpp test/test_speed_plan.cpp test/test_steering.cpp
    test/test_path_follower.cpp test/test_guard.cpp)
  target_link_libraries(mowbite_follower_test mowbite_follower_core)
endif()

ament_export_targets(export_mowbite_follower HAS_LIBRARY_TARGET)
ament_package()
