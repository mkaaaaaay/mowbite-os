cmake_minimum_required(VERSION 3.8)
project(mowbite_serial)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(mowbite_hold REQUIRED)

# The port of a driver, the processes that hold a device, the serial devices of the machine and what a port sends,
# without ROS.
add_library(mowbite_serial STATIC src/serial_port.cpp src/ports.cpp src/recognize.cpp)
target_include_directories(mowbite_serial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowbite_serial mowbite_hold::mowbite_hold_core)
set_target_properties(mowbite_serial PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_serial
  EXPORT export_mowbite_serial
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_serial_test test/test_serial_port.cpp)
  target_link_libraries(mowbite_serial_test mowbite_serial util)
  find_package(mowbite_gnss REQUIRED)
  find_package(mowbite_ntrip REQUIRED)
  ament_add_gtest(mowbite_serial_ports_test test/test_ports.cpp test/test_recognize.cpp)
  target_link_libraries(mowbite_serial_ports_test mowbite_serial util mowbite_gnss::mowbite_gnss_core
    mowbite_ntrip::mowbite_ntrip_core)
endif()

ament_export_targets(export_mowbite_serial HAS_LIBRARY_TARGET)
ament_export_dependencies(mowbite_hold)
ament_package()
