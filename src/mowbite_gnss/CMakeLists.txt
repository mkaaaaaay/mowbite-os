cmake_minimum_required(VERSION 3.8)
project(mowbite_gnss)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)

# Geodesy and NMEA sentences shared by the GNSS drivers of all platforms, without ROS.
add_library(mowbite_gnss_core STATIC src/geodesy.cpp src/gga.cpp)
target_include_directories(mowbite_gnss_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(mowbite_gnss_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_gnss_core
  EXPORT export_mowbite_gnss
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_gnss_test test/test_geodesy.cpp test/test_gga.cpp)
  target_link_libraries(mowbite_gnss_test mowbite_gnss_core)
endif()

ament_export_targets(export_mowbite_gnss HAS_LIBRARY_TARGET)
ament_package()
