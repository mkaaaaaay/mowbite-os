cmake_minimum_required(VERSION 3.8)
project(mowbite_safety)

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
find_package(std_msgs REQUIRED)
find_package(mowbite_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(mowbite_journal REQUIRED)

# The rules of the filter without ROS, testable on any machine.
add_library(mowbite_safety_core STATIC src/region.cpp src/safety_filter.cpp)
target_include_directories(mowbite_safety_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowbite_safety_core nlohmann_json::nlohmann_json)
set_target_properties(mowbite_safety_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(mowbite_safety_component SHARED src/safety_node.cpp)
target_link_libraries(mowbite_safety_component mowbite_safety_core mowbite_journal::mowbite_journal)
ament_target_dependencies(mowbite_safety_component rclcpp rclcpp_components geometry_msgs sensor_msgs std_msgs
  mowbite_msgs)
rclcpp_components_register_node(mowbite_safety_component
  PLUGIN "mowbite_safety::SafetyNode"
  EXECUTABLE mowbite_safety_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_safety_core mowbite_safety_component
  EXPORT export_mowbite_safety
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_safety_test test/test_region.cpp test/test_safety_filter.cpp)
  target_link_libraries(mowbite_safety_test mowbite_safety_core)
endif()

ament_export_targets(export_mowbite_safety HAS_LIBRARY_TARGET)
ament_package()
