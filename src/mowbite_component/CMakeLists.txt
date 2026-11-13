cmake_minimum_required(VERSION 3.8)
project(mowbite_component)

find_package(ament_cmake REQUIRED)

add_library(mowbite_component INTERFACE)
target_include_directories(mowbite_component INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_component EXPORT export_mowbite_component)

ament_export_targets(export_mowbite_component)
ament_package(CONFIG_EXTRAS cmake/mowbite_component-extras.cmake)
