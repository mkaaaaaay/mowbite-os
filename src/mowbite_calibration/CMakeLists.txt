cmake_minimum_required(VERSION 3.8)
project(mowbite_calibration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(mowbite_safety REQUIRED)

# The calibration drive and the room it needs, without ROS.
add_library(mowbite_calibration_core STATIC src/lane.cpp src/program.cpp)
target_include_directories(mowbite_calibration_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowbite_calibration_core mowbite_safety::mowbite_safety_core)
set_target_properties(mowbite_calibration_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_calibration_core
  EXPORT export_mowbite_calibration
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_calibration_test test/test_lane.cpp test/test_program.cpp)
  target_link_libraries(mowbite_calibration_test mowbite_calibration_core)
endif()

ament_export_targets(export_mowbite_calibration HAS_LIBRARY_TARGET)
ament_export_dependencies(mowbite_safety nlohmann_json)
ament_package()
