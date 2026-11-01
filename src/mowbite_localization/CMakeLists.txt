cmake_minimum_required(VERSION 3.8)
project(mowbite_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mowbite_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# Filter and gating without ROS, testable on any machine. The Kalman headers (MIT) come unchanged from
# xbot_positioning and are only used inside this package.
add_library(mowbite_localization_core STATIC src/localization.cpp)
target_include_directories(mowbite_localization_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories(mowbite_localization_core SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/kalman/include>)
target_link_libraries(mowbite_localization_core Eigen3::Eigen)
set_target_properties(mowbite_localization_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(mowbite_localization_component SHARED src/localization_node.cpp)
target_link_libraries(mowbite_localization_component mowbite_localization_core nlohmann_json::nlohmann_json)
ament_target_dependencies(mowbite_localization_component rclcpp rclcpp_components geometry_msgs nav_msgs sensor_msgs
  std_msgs std_srvs tf2_ros mowbite_msgs)
rclcpp_components_register_node(mowbite_localization_component
  PLUGIN "mowbite_localization::LocalizationNode"
  EXECUTABLE mowbite_localization_node)

install(FILES LICENSE.xbot_positioning DESTINATION share/${PROJECT_NAME})
install(TARGETS mowbite_localization_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_localization_test test/test_models.cpp test/test_localization.cpp)
  target_link_libraries(mowbite_localization_test mowbite_localization_core)
endif()

ament_package()
