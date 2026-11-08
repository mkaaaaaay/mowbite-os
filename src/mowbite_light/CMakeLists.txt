cmake_minimum_required(VERSION 3.8)
project(mowbite_light)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)

# Effects, profiles and the WS2812 frames without ROS, testable on any machine.
add_library(mowbite_light_core STATIC src/ws2812.cpp src/effect.cpp)
target_include_directories(mowbite_light_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowbite_light_core nlohmann_json::nlohmann_json)
set_target_properties(mowbite_light_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_light_core
  EXPORT export_mowbite_light
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_light_test test/test_light.cpp)
  target_link_libraries(mowbite_light_test mowbite_light_core)
endif()

ament_export_targets(export_mowbite_light HAS_LIBRARY_TARGET)
ament_package()
