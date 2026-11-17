cmake_minimum_required(VERSION 3.8)
project(mowbite_setup)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(mowbite_api REQUIRED)
find_package(mowbite_hold REQUIRED)
find_package(mowbite_bridge REQUIRED)
find_package(mowbite_map REQUIRED)
find_package(mowbite_ntrip REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mowbite_component REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mowbite_msgs REQUIRED)
find_package(mowbite_watch REQUIRED)

# The step list of a platform and the program over it, without ROS.
add_library(mowbite_setup_core STATIC src/steps.cpp src/program.cpp)
target_include_directories(mowbite_setup_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowbite_setup_core mowbite_api::mowbite_api_core nlohmann_json::nlohmann_json)
set_target_properties(mowbite_setup_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Checks a step list (JSON on stdin) and its images (folder as argument) by the rules of the node.
add_executable(mowbite_setup_check src/check_main.cpp)
target_link_libraries(mowbite_setup_check mowbite_setup_core)

add_library(mowbite_setup_component SHARED src/setup_node.cpp)
target_link_libraries(mowbite_setup_component mowbite_setup_core mowbite_hold::mowbite_hold_core
  mowbite_bridge::mowbite_bridge_core mowbite_map::mowbite_map_core mowbite_ntrip::mowbite_ntrip_core
  mowbite_watch::mowbite_watch)
ament_target_dependencies(mowbite_setup_component rclcpp rclcpp_components sensor_msgs std_msgs mowbite_msgs)
mowbite_component_register(mowbite_setup_component
  PLUGIN "mowbite_setup::SetupNode"
  EXECUTABLE mowbite_setup_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_setup_check DESTINATION lib/${PROJECT_NAME})
install(TARGETS mowbite_setup_core mowbite_setup_component
  EXPORT export_mowbite_setup
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_setup_test test/test_steps.cpp test/test_program.cpp)
  target_link_libraries(mowbite_setup_test mowbite_setup_core)
endif()

ament_export_targets(export_mowbite_setup HAS_LIBRARY_TARGET)
ament_export_dependencies(mowbite_api nlohmann_json)
ament_package()
