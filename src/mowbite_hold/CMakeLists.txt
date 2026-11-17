cmake_minimum_required(VERSION 3.8)
project(mowbite_hold)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Threads REQUIRED)

# The hold of an update (the file, its time limit and the watch of the drivers), the restart hold, the safety lock
# of the setup program and the writer of small files in a thread of its own, without ROS.
add_library(mowbite_hold_core STATIC src/file_io.cpp src/file_writer.cpp src/hold.cpp src/lock.cpp src/restart.cpp)
target_include_directories(mowbite_hold_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowbite_hold_core PRIVATE nlohmann_json::nlohmann_json PUBLIC Threads::Threads)
set_target_properties(mowbite_hold_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_hold_core
  EXPORT export_mowbite_hold
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_hold_test test/test_file_writer.cpp test/test_hold.cpp test/test_lock.cpp
    test/test_restart.cpp)
  target_link_libraries(mowbite_hold_test mowbite_hold_core nlohmann_json::nlohmann_json)
endif()

ament_export_targets(export_mowbite_hold HAS_LIBRARY_TARGET)
ament_export_dependencies(nlohmann_json Threads)
ament_package()
