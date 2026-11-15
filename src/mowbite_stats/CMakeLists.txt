cmake_minimum_required(VERSION 3.8)
project(mowbite_stats)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mowbite_component REQUIRED)
find_package(mowbite_msgs REQUIRED)
find_package(mowbite_hold REQUIRED)
find_package(mowbite_watch REQUIRED)
find_package(mowbite_api REQUIRED)
find_package(mowbite_journal REQUIRED)
find_package(mowbite_bridge REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# The counter, the sensor history, the incident keeper and the answers of stats.* and sensors.history without clock,
# files or ROS.
add_library(mowbite_stats_core STATIC src/counter.cpp src/sensor_history.cpp src/stats_rpc.cpp src/incident_keeper.cpp)
target_include_directories(mowbite_stats_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowbite_stats_core PUBLIC nlohmann_json::nlohmann_json mowbite_api::mowbite_api_core)
set_target_properties(mowbite_stats_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The incidents of the journal: files and a worker thread, no ROS; mowbite_replay copies runs without their place.
add_library(mowbite_stats_incidents STATIC src/incident_store.cpp src/incidents.cpp)
target_link_libraries(mowbite_stats_incidents PUBLIC mowbite_stats_core mowbite_journal::mowbite_journal
  mowbite_hold::mowbite_hold_core OpenSSL::Crypto)
set_target_properties(mowbite_stats_incidents PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(mowbite_stats_component SHARED src/stats_node.cpp)
target_link_libraries(mowbite_stats_component mowbite_stats_core mowbite_stats_incidents
  mowbite_hold::mowbite_hold_core mowbite_journal::mowbite_journal mowbite_watch::mowbite_watch
  mowbite_bridge::mowbite_bridge_core)
ament_target_dependencies(mowbite_stats_component rclcpp rclcpp_components mowbite_msgs rcl_interfaces std_msgs)
mowbite_component_register(mowbite_stats_component
  PLUGIN "mowbite_stats::StatsNode"
  EXECUTABLE mowbite_stats_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_stats_core mowbite_stats_incidents mowbite_stats_component
  EXPORT export_mowbite_stats
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_stats_test test/test_counter.cpp test/test_sensor_history.cpp test/test_stats_rpc.cpp
    test/test_incident_keeper.cpp)
  target_link_libraries(mowbite_stats_test mowbite_stats_core)
  ament_add_gtest(mowbite_stats_incidents_test test/test_incidents.cpp)
  target_link_libraries(mowbite_stats_incidents_test mowbite_stats_incidents)
endif()

ament_export_targets(export_mowbite_stats HAS_LIBRARY_TARGET)
ament_package()
