cmake_minimum_required(VERSION 3.8)
project(mowbite_hold)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)

# The hold of an update (the file, its time limit and the watch of the drivers) and the restart hold, without ROS.
add_library(mowbite_hold_core STATIC src/file_io.cpp src/hold.cpp src/restart.cpp)
target_include_directories(mowbite_hold_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowbite_hold_core PRIVATE nlohmann_json::nlohmann_json)
set_target_properties(mowbite_hold_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_hold_core
  EXPORT export_mowbite_hold
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_hold_test test/test_hold.cpp test/test_restart.cpp)
  target_link_libraries(mowbite_hold_test mowbite_hold_core nlohmann_json::nlohmann_json)
endif()

ament_export_targets(export_mowbite_hold HAS_LIBRARY_TARGET)
ament_export_dependencies(nlohmann_json)
ament_package()
