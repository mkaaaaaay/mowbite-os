cmake_minimum_required(VERSION 3.8)
project(mowbite_calibration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ZLIB REQUIRED)
find_package(mowbite_safety REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mowbite_component REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mowbite_msgs REQUIRED)
find_package(mowbite_watch REQUIRED)

# The calibration drive, the room it needs, its recording and result, without ROS.
add_library(mowbite_calibration_core STATIC src/lane.cpp src/program.cpp src/recording.cpp src/result.cpp)
target_include_directories(mowbite_calibration_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowbite_calibration_core mowbite_safety::mowbite_safety_core nlohmann_json::nlohmann_json
  ZLIB::ZLIB)
set_target_properties(mowbite_calibration_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(mowbite_calibration_component SHARED src/calibration_node.cpp)
target_link_libraries(mowbite_calibration_component mowbite_calibration_core mowbite_watch::mowbite_watch)
ament_target_dependencies(mowbite_calibration_component rclcpp rclcpp_action rclcpp_components geometry_msgs
  sensor_msgs std_msgs mowbite_msgs)
mowbite_component_register(mowbite_calibration_component
  PLUGIN "mowbite_calibration::CalibrationNode"
  EXECUTABLE mowbite_calibration_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_calibration_core mowbite_calibration_component
  EXPORT export_mowbite_calibration
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_calibration_test test/test_lane.cpp test/test_program.cpp test/test_recording.cpp
    test/test_result.cpp)
  target_link_libraries(mowbite_calibration_test mowbite_calibration_core)
endif()

ament_export_targets(export_mowbite_calibration HAS_LIBRARY_TARGET)
ament_export_dependencies(mowbite_safety nlohmann_json ZLIB)
ament_package()
