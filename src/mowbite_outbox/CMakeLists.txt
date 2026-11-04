cmake_minimum_required(VERSION 3.8)
project(mowbite_outbox)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

# Rings, latest values, counters and the wakeup between the I/O thread of a driver and its output thread.
add_library(mowbite_outbox_core STATIC src/outbox.cpp)
target_include_directories(mowbite_outbox_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowbite_outbox_core PUBLIC Threads::Threads)
set_target_properties(mowbite_outbox_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_outbox_core
  EXPORT export_mowbite_outbox
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_outbox_test test/test_outbox.cpp)
  target_link_libraries(mowbite_outbox_test mowbite_outbox_core)
endif()

ament_export_targets(export_mowbite_outbox HAS_LIBRARY_TARGET)
ament_export_dependencies(Threads)
ament_package()
