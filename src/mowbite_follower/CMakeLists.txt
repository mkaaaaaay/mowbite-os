cmake_minimum_required(VERSION 3.8)
project(mowbite_follower)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)

# Path tracking and its checks without ROS, testable on any machine.
add_library(mowbite_follower_core STATIC src/path_tracker.cpp src/guard.cpp)
target_include_directories(mowbite_follower_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(mowbite_follower_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_follower_core
  EXPORT export_mowbite_follower
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_follower_test test/test_path_tracker.cpp test/test_guard.cpp)
  target_link_libraries(mowbite_follower_test mowbite_follower_core)
endif()

ament_export_targets(export_mowbite_follower HAS_LIBRARY_TARGET)
ament_package()
