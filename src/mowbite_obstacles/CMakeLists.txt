cmake_minimum_required(VERSION 3.8)
project(mowbite_obstacles)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)

# Where a box of a camera stands on the ground, from rays and the mount; for the platforms, without ROS.
add_library(mowbite_obstacles_ground STATIC src/ground.cpp)
target_include_directories(mowbite_obstacles_ground PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(mowbite_obstacles_ground PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_obstacles_ground
  EXPORT export_mowbite_obstacles
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_obstacles_ground_test test/test_ground.cpp)
  target_link_libraries(mowbite_obstacles_ground_test mowbite_obstacles_ground)
endif()

ament_export_targets(export_mowbite_obstacles HAS_LIBRARY_TARGET)
ament_package()
