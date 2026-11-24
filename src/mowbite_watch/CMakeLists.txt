cmake_minimum_required(VERSION 3.8)
project(mowbite_watch)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Threads REQUIRED)

# Shared, so that all components of a process have one watch; never unloaded, its thread runs until the process ends.
add_library(mowbite_watch SHARED src/watch_core.cpp src/watch.cpp src/attach.cpp)
target_include_directories(mowbite_watch PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowbite_watch Threads::Threads)
target_link_options(mowbite_watch PRIVATE -Wl,-z,nodelete)
ament_target_dependencies(mowbite_watch rclcpp)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_watch
  EXPORT export_mowbite_watch
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Core and runtime from their sources, without ROS: the test runs exactly this build of them.
  ament_add_gtest(mowbite_watch_test test/test_watch_core.cpp test/test_watch.cpp test/test_tick_gap.cpp
    src/watch_core.cpp src/watch.cpp)
  target_include_directories(mowbite_watch_test PRIVATE include)
  target_link_libraries(mowbite_watch_test Threads::Threads)
endif()

ament_export_targets(export_mowbite_watch HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp Threads)
ament_package()
