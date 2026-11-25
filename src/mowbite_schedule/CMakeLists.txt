cmake_minimum_required(VERSION 3.8)
project(mowbite_schedule)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(mowbite_time REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mowbite_component REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mowbite_msgs REQUIRED)
find_package(mowbite_hold REQUIRED)
find_package(mowbite_journal REQUIRED)
find_package(mowbite_watch REQUIRED)
find_package(mowbite_logic REQUIRED)
find_package(mowbite_bridge REQUIRED)

# The schedule without clock, files or ROS: time zones, the schedule and its checks, the decision, the answers of
# schedule.*.
add_library(mowbite_schedule_core STATIC src/tz.cpp src/model.cpp src/scheduler.cpp src/schedule_rpc.cpp)
target_include_directories(mowbite_schedule_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowbite_schedule_core PUBLIC nlohmann_json::nlohmann_json PRIVATE mowbite_time::mowbite_time_core)
set_target_properties(mowbite_schedule_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(mowbite_schedule_component SHARED src/schedule_node.cpp)
target_link_libraries(mowbite_schedule_component mowbite_schedule_core mowbite_logic::mowbite_logic_core
  mowbite_hold::mowbite_hold_core mowbite_journal::mowbite_journal mowbite_watch::mowbite_watch
  mowbite_bridge::mowbite_bridge_core)
ament_target_dependencies(mowbite_schedule_component rclcpp rclcpp_components std_msgs sensor_msgs mowbite_msgs)
mowbite_component_register(mowbite_schedule_component
  PLUGIN "mowbite_schedule::ScheduleNode"
  EXECUTABLE mowbite_schedule_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_schedule_core mowbite_schedule_component
  EXPORT export_mowbite_schedule
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(mowbite_api REQUIRED)
  ament_add_gtest(mowbite_schedule_test test/test_tz.cpp test/test_model.cpp test/test_scheduler.cpp
    test/test_schedule_rpc.cpp)
  target_link_libraries(mowbite_schedule_test mowbite_schedule_core mowbite_time::mowbite_time_core
    mowbite_api::mowbite_api_core)
  add_test(NAME test_topics COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_topics.py)
  target_compile_definitions(mowbite_schedule_test PRIVATE
    SCHEDULER_SOURCE="${CMAKE_CURRENT_SOURCE_DIR}/src/scheduler.cpp")
endif()

ament_export_targets(export_mowbite_schedule HAS_LIBRARY_TARGET)
ament_export_dependencies(nlohmann_json mowbite_time)
ament_package()
