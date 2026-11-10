cmake_minimum_required(VERSION 3.8)
project(mowbite_map)

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
find_package(mowbite_bridge REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(mowbite_watch REQUIRED)

# Map model, storage and RPC handling without ROS, testable on any machine.
add_library(mowbite_map_core STATIC src/map_model.cpp src/map_store.cpp src/map_service.cpp src/sha256.cpp)
target_include_directories(mowbite_map_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowbite_map_core nlohmann_json::nlohmann_json OpenSSL::Crypto)
set_target_properties(mowbite_map_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(mowbite_map_component SHARED src/map_node.cpp)
target_link_libraries(mowbite_map_component mowbite_map_core mowbite_bridge::mowbite_bridge_core
  mowbite_watch::mowbite_watch)
ament_target_dependencies(mowbite_map_component rclcpp rclcpp_components sensor_msgs std_msgs mowbite_msgs)
rclcpp_components_register_node(mowbite_map_component
  PLUGIN "mowbite_map::MapNode"
  EXECUTABLE mowbite_map_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_map_core mowbite_map_component
  EXPORT export_mowbite_map
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(mowbite_api REQUIRED)
  ament_add_gtest(mowbite_map_test test/test_map_model.cpp test/test_map_store.cpp test/test_map_service.cpp
    test/test_sha256.cpp)
  target_link_libraries(mowbite_map_test mowbite_map_core mowbite_api::mowbite_api_core)
endif()

ament_export_targets(export_mowbite_map HAS_LIBRARY_TARGET)
ament_package()
