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

# The schedule without clock, files or ROS: time zones, the schedule and its checks, the decision.
add_library(mowbite_schedule_core STATIC src/tz.cpp src/model.cpp src/scheduler.cpp)
target_include_directories(mowbite_schedule_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowbite_schedule_core PUBLIC nlohmann_json::nlohmann_json PRIVATE mowbite_time::mowbite_time_core)
set_target_properties(mowbite_schedule_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_schedule_core
  EXPORT export_mowbite_schedule
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_schedule_test test/test_tz.cpp test/test_model.cpp test/test_scheduler.cpp)
  target_link_libraries(mowbite_schedule_test mowbite_schedule_core mowbite_time::mowbite_time_core)
endif()

ament_export_targets(export_mowbite_schedule HAS_LIBRARY_TARGET)
ament_export_dependencies(nlohmann_json mowbite_time)
ament_package()
