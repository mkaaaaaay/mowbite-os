cmake_minimum_required(VERSION 3.8)
project(mowbite_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mowbite_component REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mowbite_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(mowbite_journal REQUIRED)
find_package(mowbite_watch REQUIRED)

# Filter, gyro offset, GNSS gate and heading from the course without ROS, testable on any machine.
add_library(mowbite_localization_core STATIC src/gyro_bias.cpp src/pose_filter.cpp src/fix_gate.cpp
  src/course_alignment.cpp src/estimator.cpp)
target_include_directories(mowbite_localization_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowbite_localization_core PUBLIC Eigen3::Eigen)
set_target_properties(mowbite_localization_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(mowbite_localization_component SHARED src/localization_component.cpp)
target_link_libraries(mowbite_localization_component mowbite_localization_core nlohmann_json::nlohmann_json
  mowbite_journal::mowbite_journal mowbite_watch::mowbite_watch)
ament_target_dependencies(mowbite_localization_component rclcpp rclcpp_components geometry_msgs nav_msgs sensor_msgs
  std_msgs std_srvs tf2_ros mowbite_msgs)
mowbite_component_register(mowbite_localization_component
  PLUGIN "mowbite_localization::LocalizationNode"
  EXECUTABLE mowbite_localization_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_localization_core mowbite_localization_component
  EXPORT export_mowbite_localization
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_localization_test test/test_gyro_bias.cpp test/test_pose_filter.cpp test/test_fix_gate.cpp
    test/test_course_alignment.cpp test/test_estimator.cpp)
  target_link_libraries(mowbite_localization_test mowbite_localization_core)
endif()

ament_export_targets(export_mowbite_localization HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_package()
