cmake_minimum_required(VERSION 3.8)
project(mowbite_map)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)

# Map model, storage and RPC handling without ROS, testable on any machine.
add_library(mowbite_map_core STATIC src/map_model.cpp)
target_include_directories(mowbite_map_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowbite_map_core nlohmann_json::nlohmann_json)
set_target_properties(mowbite_map_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_map_core
  EXPORT export_mowbite_map
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_map_test test/test_map_model.cpp)
  target_link_libraries(mowbite_map_test mowbite_map_core)
endif()

ament_export_targets(export_mowbite_map HAS_LIBRARY_TARGET)
ament_package()
