cmake_minimum_required(VERSION 3.8)
project(mowbite_api)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)

# The planner's part of the schema, read from the installed planner at build time; none in a planner without it.
set(MOWBITE_PLANNER_SCHEMA_FILE "")
set(MOWBITE_PLANNER_SCHEMA_TEXT "")
string(REPLACE ":" ";" _prefixes "$ENV{AMENT_PREFIX_PATH}")
foreach(_prefix ${_prefixes} ${CMAKE_PREFIX_PATH})
  if(NOT MOWBITE_PLANNER_SCHEMA_FILE AND EXISTS "${_prefix}/share/mowbite_planner_os/api/planner.schema.json")
    set(MOWBITE_PLANNER_SCHEMA_FILE "${_prefix}/share/mowbite_planner_os/api/planner.schema.json")
  endif()
endforeach()
if(MOWBITE_PLANNER_SCHEMA_FILE)
  file(READ "${MOWBITE_PLANNER_SCHEMA_FILE}" MOWBITE_PLANNER_SCHEMA_TEXT)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${MOWBITE_PLANNER_SCHEMA_FILE}")
  message(STATUS "mowbite_api: planner schema ${MOWBITE_PLANNER_SCHEMA_FILE}")
else()
  message(STATUS "mowbite_api: no planner schema, planner events count as unknown types")
endif()
configure_file(src/planner_schema_text.h.in ${CMAKE_CURRENT_BINARY_DIR}/planner_schema_text.h @ONLY)

# The catalog of the app interface, its JSON Schema and the check of events against it, without ROS.
add_library(mowbite_api_core STATIC src/catalog.cpp src/events.cpp src/schema.cpp src/check.cpp src/planner.cpp
  src/setup.cpp)
target_include_directories(mowbite_api_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(mowbite_api_core PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(mowbite_api_core PUBLIC nlohmann_json::nlohmann_json)
set_target_properties(mowbite_api_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Prints the schema, or checks or writes api.schema.json.
add_executable(mowbite_api_schema src/schema_main.cpp)
target_link_libraries(mowbite_api_schema mowbite_api_core)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_api_core
  EXPORT export_mowbite_api
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS mowbite_api_schema DESTINATION lib/${PROJECT_NAME})
install(FILES api.schema.json DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_api_test test/test_catalog.cpp test/test_schema.cpp test/test_check.cpp TIMEOUT 300)
  target_link_libraries(mowbite_api_test mowbite_api_core)
  target_compile_definitions(mowbite_api_test PRIVATE
    MOWBITE_API_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    MOWBITE_SOURCE_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/..")
endif()

ament_export_targets(export_mowbite_api HAS_LIBRARY_TARGET)
ament_export_dependencies(nlohmann_json)
ament_package()
