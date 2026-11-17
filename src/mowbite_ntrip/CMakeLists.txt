cmake_minimum_required(VERSION 3.8)
project(mowbite_ntrip)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mowbite_component REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mowbite_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(mowbite_bridge REQUIRED)
find_package(mowbite_watch REQUIRED)

# Access file and its RPC, the source used last, NTRIP protocol, RTCM frames and the client's states without ROS and
# sockets, testable on any machine.
add_library(mowbite_ntrip_core STATIC src/rtcm.cpp src/ntrip_config.cpp src/ntrip_access.cpp src/ntrip_source.cpp
  src/ntrip_protocol.cpp src/ntrip_client.cpp)
target_include_directories(mowbite_ntrip_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowbite_ntrip_core nlohmann_json::nlohmann_json)
set_target_properties(mowbite_ntrip_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(mowbite_ntrip_component SHARED src/ntrip_node.cpp)
target_link_libraries(mowbite_ntrip_component mowbite_ntrip_core mowbite_bridge::mowbite_bridge_core
  mowbite_watch::mowbite_watch)
ament_target_dependencies(mowbite_ntrip_component rclcpp rclcpp_components std_msgs mowbite_msgs)
mowbite_component_register(mowbite_ntrip_component
  PLUGIN "mowbite_ntrip::NtripNode"
  EXECUTABLE mowbite_ntrip_node)

install(DIRECTORY include/ DESTINATION include)
# The access file written as the node reads it, for the import from OpenMower (mowbite_bringup migrate.py).
add_executable(mowbite_ntrip_write src/write_main.cpp)
target_link_libraries(mowbite_ntrip_write mowbite_ntrip_core)
install(TARGETS mowbite_ntrip_write DESTINATION lib/${PROJECT_NAME})

install(TARGETS mowbite_ntrip_core mowbite_ntrip_component
  EXPORT export_mowbite_ntrip
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(mowbite_api REQUIRED)
  ament_add_gtest(mowbite_ntrip_test test/test_rtcm.cpp test/test_ntrip_config.cpp test/test_ntrip_access.cpp
    test/test_ntrip_source.cpp test/test_ntrip_protocol.cpp test/test_ntrip_client.cpp)
  target_link_libraries(mowbite_ntrip_test mowbite_ntrip_core mowbite_api::mowbite_api_core)
endif()

ament_export_targets(export_mowbite_ntrip HAS_LIBRARY_TARGET)
ament_package()
