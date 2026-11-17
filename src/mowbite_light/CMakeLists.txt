cmake_minimum_required(VERSION 3.8)
project(mowbite_light)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(mowbite_hold REQUIRED)
find_package(mowbite_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mowbite_component REQUIRED)
find_package(mowbite_msgs REQUIRED)
find_package(mowbite_serial REQUIRED)
find_package(mowbite_watch REQUIRED)

# Effects, profiles, the RPC methods and the WS2812 frames without ROS, testable on any machine.
add_library(mowbite_light_core STATIC src/ws2812.cpp src/effect.cpp src/light_service.cpp src/output.cpp)
target_include_directories(mowbite_light_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowbite_light_core nlohmann_json::nlohmann_json mowbite_hold::mowbite_hold_core
  mowbite_bridge::mowbite_bridge_core)
set_target_properties(mowbite_light_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(mowbite_light_component SHARED src/light_node.cpp)
target_link_libraries(mowbite_light_component mowbite_light_core mowbite_serial::mowbite_serial mowbite_watch::mowbite_watch)
ament_target_dependencies(mowbite_light_component rclcpp rclcpp_components mowbite_msgs)
mowbite_component_register(mowbite_light_component
  PLUGIN "mowbite_light::LightNode"
  EXECUTABLE mowbite_light_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_light_core mowbite_light_component
  EXPORT export_mowbite_light
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(mowbite_api REQUIRED)
  ament_add_gtest(mowbite_light_test test/test_light.cpp test/test_light_service.cpp)
  target_link_libraries(mowbite_light_test mowbite_light_core mowbite_api::mowbite_api_core)
endif()

ament_export_targets(export_mowbite_light HAS_LIBRARY_TARGET)
ament_export_dependencies(mowbite_hold mowbite_bridge)
ament_package()
