cmake_minimum_required(VERSION 3.8)
project(mowbite_logic)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mowbite_msgs REQUIRED)
find_package(mowbite_bridge REQUIRED)
find_package(mowbite_planner_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nlohmann_json REQUIRED)

# States, actions and sensors without ROS, testable on any machine.
add_library(mowbite_logic_core STATIC src/reasons.cpp src/logic_core.cpp src/area_recorder.cpp src/mowing_plan.cpp src/mowing_job.cpp)
target_include_directories(mowbite_logic_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowbite_logic_core nlohmann_json::nlohmann_json)
set_target_properties(mowbite_logic_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(mowbite_logic_component SHARED src/logic_node.cpp)
target_link_libraries(mowbite_logic_component mowbite_logic_core mowbite_bridge::mowbite_bridge_core)
ament_target_dependencies(mowbite_logic_component rclcpp rclcpp_components rclcpp_action geometry_msgs std_msgs
  std_srvs mowbite_msgs mowbite_planner_interfaces rcl_interfaces)
rclcpp_components_register_node(mowbite_logic_component
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
    test/test_mowing_job.cpp test/test_logic_mowing.cpp test/test_reasons.cpp)
  target_link_libraries(mowbite_logic_test mowbite_logic_core)
endif()

ament_export_targets(export_mowbite_logic HAS_LIBRARY_TARGET)
ament_package()
