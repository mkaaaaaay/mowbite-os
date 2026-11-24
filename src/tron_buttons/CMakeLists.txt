cmake_minimum_required(VERSION 3.8)
project(tron_buttons)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mowbite_component REQUIRED)
find_package(mowbite_msgs REQUIRED)
find_package(mowbite_watch REQUIRED)

# Reading the lines and telling the presses apart without ROS, testable on any machine.
add_library(tron_buttons_core STATIC src/buttons.cpp)
target_include_directories(tron_buttons_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(tron_buttons_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(tron_buttons_component SHARED src/buttons_node.cpp)
target_link_libraries(tron_buttons_component tron_buttons_core mowbite_watch::mowbite_watch)
ament_target_dependencies(tron_buttons_component rclcpp rclcpp_components mowbite_msgs)
mowbite_component_register(tron_buttons_component
  PLUGIN "tron_buttons::ButtonsNode"
  EXECUTABLE tron_buttons_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS tron_buttons_core tron_buttons_component
  EXPORT export_tron_buttons
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(tron_buttons_test test/test_buttons.cpp)
  target_link_libraries(tron_buttons_test tron_buttons_core)
endif()

ament_export_targets(export_tron_buttons HAS_LIBRARY_TARGET)
ament_package()
