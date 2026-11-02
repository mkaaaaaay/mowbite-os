cmake_minimum_required(VERSION 3.8)
project(xcore_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)

# Frames, CBOR, service descriptions and control logic without ROS, testable on any machine.
add_library(xcore_core STATIC src/wire.cpp src/cbor.cpp src/schema.cpp src/services.cpp src/odometry.cpp src/xcore_control.cpp src/udp.cpp src/drive_regulator.cpp src/input_config.cpp src/link.cpp)
target_include_directories(xcore_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(xcore_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS xcore_core
  EXPORT export_xcore_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(xcore_test test/test_wire.cpp test/test_cbor.cpp test/test_schema.cpp test/test_odometry.cpp test/test_xcore_control.cpp test/test_udp.cpp test/test_drive_regulator.cpp test/test_input_config.cpp test/test_link.cpp)
  target_link_libraries(xcore_test xcore_core)
endif()

ament_export_targets(export_xcore_driver HAS_LIBRARY_TARGET)
ament_package()
