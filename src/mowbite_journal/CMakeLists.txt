cmake_minimum_required(VERSION 3.8)
project(mowbite_journal)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZSTD REQUIRED IMPORTED_TARGET libzstd)

# Shared, so that all components of a process write one journal.
add_library(mowbite_journal SHARED src/journal.cpp src/reader.cpp src/replay.cpp)
target_include_directories(mowbite_journal PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowbite_journal PUBLIC nlohmann_json::nlohmann_json Threads::Threads PRIVATE PkgConfig::ZSTD)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_journal
  EXPORT export_mowbite_journal
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_journal_test test/test_journal.cpp)
  target_link_libraries(mowbite_journal_test mowbite_journal)
endif()

ament_export_targets(export_mowbite_journal HAS_LIBRARY_TARGET)
ament_export_dependencies(nlohmann_json Threads)
ament_package()
