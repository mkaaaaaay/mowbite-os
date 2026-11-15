cmake_minimum_required(VERSION 3.8)
project(mowbite_replay)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(mowbite_journal REQUIRED)
find_package(mowbite_localization REQUIRED)
find_package(mowbite_logic REQUIRED)
find_package(mowbite_safety REQUIRED)
find_package(mowbite_schedule REQUIRED)
find_package(tron_mcu REQUIRED)

# The cores this tool knows, also for the tests and for nodes that anonymize a run.
add_library(mowbite_replay_cores STATIC src/cores.cpp)
set_target_properties(mowbite_replay_cores PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(mowbite_replay_cores PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowbite_replay_cores PUBLIC
  mowbite_journal::mowbite_journal
  mowbite_localization::mowbite_localization_core
  mowbite_logic::mowbite_logic_core
  mowbite_safety::mowbite_safety_core
  mowbite_schedule::mowbite_schedule_core
  tron_mcu::tron_mcu_core)

add_executable(mowbite_replay src/main.cpp)
target_link_libraries(mowbite_replay mowbite_replay_cores)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_replay_cores
  EXPORT export_mowbite_replay
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS mowbite_replay DESTINATION lib/${PROJECT_NAME})
# On the PATH of the image as well: docker exec <container> mowbite_replay check /journal
install(TARGETS mowbite_replay DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_replay_test test/test_cores.cpp)
  target_link_libraries(mowbite_replay_test mowbite_replay_cores)
  add_test(NAME test_core_rules COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_core_rules.py)
  add_test(NAME test_place_tables COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_place_tables.py)
endif()

ament_export_targets(export_mowbite_replay HAS_LIBRARY_TARGET)
ament_export_dependencies(mowbite_journal mowbite_localization mowbite_logic mowbite_safety mowbite_schedule tron_mcu)
ament_package()
