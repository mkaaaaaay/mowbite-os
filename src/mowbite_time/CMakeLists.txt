cmake_minimum_required(VERSION 3.8)
project(mowbite_time)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)

# The sun and the judgement of the clock, without ROS.
add_library(mowbite_time_core STATIC src/sun.cpp src/clock_guard.cpp)
target_include_directories(mowbite_time_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(mowbite_time_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_time_core
  EXPORT export_mowbite_time
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_time_test test/test_sun.cpp test/test_clock_guard.cpp)
  target_link_libraries(mowbite_time_test mowbite_time_core)
  # For the simulation tests; not in the image, which builds without tests.
  add_executable(mowbite_sun_times src/sun_times_main.cpp)
  target_link_libraries(mowbite_sun_times mowbite_time_core)
  install(TARGETS mowbite_sun_times DESTINATION lib/${PROJECT_NAME})
endif()

ament_export_targets(export_mowbite_time HAS_LIBRARY_TARGET)
ament_package()
