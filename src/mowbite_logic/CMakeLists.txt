cmake_minimum_required(VERSION 3.8)
project(mowbite_logic)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mowbite_component REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mowbite_msgs REQUIRED)
find_package(mowbite_bridge REQUIRED)
find_package(mowbite_planner_interfaces REQUIRED)
find_package(mowbite_planner_os_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(mowbite_hold REQUIRED)
find_package(mowbite_api REQUIRED)
find_package(mowbite_journal REQUIRED)
find_package(mowbite_watch REQUIRED)
find_package(mowbite_time REQUIRED)
find_package(sensor_msgs REQUIRED)

# States, actions and sensors without ROS, testable on any machine.
add_library(mowbite_logic_core STATIC src/reasons.cpp src/logic_core.cpp src/area_recorder.cpp src/mowing_plan.cpp src/mowing_job.cpp
  src/logic_params.cpp src/undocking.cpp src/docking.cpp src/geometry.cpp src/home_watch.cpp
  src/storage.cpp src/logic_dock.cpp src/logic_undock.cpp src/logic_home.cpp src/logic_job.cpp src/direct_route.cpp
  src/logic_recording.cpp src/logic_run.cpp src/logic_calibration.cpp src/logic_setup.cpp src/logic_charge.cpp
  src/marker_dock.cpp src/logic_marker.cpp src/logic_release.cpp src/core_access.cpp src/logic_parts.cpp
  src/logic_schedule.cpp src/logic_going_home.cpp
  src/logic_undocking.cpp src/logic_docking.cpp src/logic_mowing.cpp src/logic_obstacle.cpp src/logic_sensors.cpp)
target_include_directories(mowbite_logic_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowbite_logic_core mowbite_api::mowbite_api_core mowbite_time::mowbite_time_core
  nlohmann_json::nlohmann_json)
set_target_properties(mowbite_logic_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(mowbite_logic_component SHARED src/logic_node.cpp src/logic_node_blade.cpp src/logic_node_run.cpp src/logic_node_dock.cpp src/logic_node_obstacles.cpp src/logic_node_hold.cpp src/logic_node_params.cpp src/logic_messages.cpp src/logic_config.cpp)
target_link_libraries(mowbite_logic_component mowbite_logic_core mowbite_bridge::mowbite_bridge_core
  mowbite_hold::mowbite_hold_core mowbite_journal::mowbite_journal mowbite_watch::mowbite_watch)
ament_target_dependencies(mowbite_logic_component rclcpp rclcpp_components rclcpp_action geometry_msgs std_msgs
  sensor_msgs std_srvs mowbite_msgs mowbite_planner_interfaces mowbite_planner_os_interfaces rcl_interfaces)
mowbite_component_register(mowbite_logic_component
  PLUGIN "mowbite_logic::LogicNode"
  EXECUTABLE mowbite_logic_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_logic_core mowbite_logic_component
  EXPORT export_mowbite_logic
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_logic_test test/test_logic_core.cpp test/test_area_recorder.cpp test/test_mowing_plan.cpp
    test/test_mowing_job.cpp test/test_logic_mowing.cpp test/test_reasons.cpp test/test_logic_params.cpp
    test/test_logic_dock.cpp test/test_logic_undock.cpp test/test_logic_docking.cpp test/test_geometry.cpp
    test/test_logic_hold.cpp test/test_logic_calibration.cpp test/test_logic_setup.cpp test/test_event_catalog.cpp
    test/test_logic_commands.cpp test/test_home_watch.cpp test/test_logic_home.cpp
    test/test_logic_restart.cpp test/test_logic_effects.cpp test/test_direct_route.cpp test/test_logic_run.cpp
    test/test_marker_dock.cpp test/test_logic_marker.cpp test/test_logic_marker_guide.cpp test/test_logic_obstacle.cpp)
  target_link_libraries(mowbite_logic_test mowbite_logic_core)
endif()

ament_export_targets(export_mowbite_logic HAS_LIBRARY_TARGET)
ament_export_dependencies(mowbite_api mowbite_time)
ament_package()
