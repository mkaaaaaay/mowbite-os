cmake_minimum_required(VERSION 3.8)
project(mowbite_sound)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ALSA REQUIRED)
find_package(mowbite_api REQUIRED)
find_package(mowbite_hold REQUIRED)
find_package(mowbite_bridge REQUIRED)
find_package(mowbite_map REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mowbite_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(mowbite_sound_wav STATIC src/wav.cpp)
target_include_directories(mowbite_sound_wav PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(mowbite_sound_wav PROPERTIES POSITION_INDEPENDENT_CODE ON)

# minimp3 (CC0-1.0) from github.com/lieff/minimp3 at ea99364f61c14656440e8d77e9c233ccf3124633, only in the converter.
add_library(mowbite_sound_decode STATIC src/decode_mp3.cpp)
target_link_libraries(mowbite_sound_decode mowbite_sound_wav)
target_include_directories(mowbite_sound_decode SYSTEM PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/minimp3)

add_executable(mowbite_sound_convert src/convert_main.cpp)
target_link_libraries(mowbite_sound_convert mowbite_sound_decode mowbite_hold::mowbite_hold_core)

# Situations, the RPC methods, sounds.json, the import and the player without ROS.
add_library(mowbite_sound_core STATIC src/sound_service.cpp src/run_program.cpp src/player.cpp
  src/alsa_output.cpp)
target_link_libraries(mowbite_sound_core mowbite_sound_wav nlohmann_json::nlohmann_json ALSA::ALSA
  mowbite_api::mowbite_api_core mowbite_hold::mowbite_hold_core mowbite_bridge::mowbite_bridge_core
  mowbite_map::mowbite_map_core)
set_target_properties(mowbite_sound_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(mowbite_sound_component SHARED src/sound_node.cpp)
target_link_libraries(mowbite_sound_component mowbite_sound_core)
ament_target_dependencies(mowbite_sound_component ament_index_cpp rclcpp rclcpp_components mowbite_msgs std_msgs)
rclcpp_components_register_node(mowbite_sound_component
  PLUGIN "mowbite_sound::SoundNode"
  EXECUTABLE mowbite_sound_node)

install(TARGETS mowbite_sound_convert DESTINATION lib/${PROJECT_NAME})
install(TARGETS mowbite_sound_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(FILES third_party/minimp3/LICENSE DESTINATION share/${PROJECT_NAME}/minimp3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_sound_test test/test_sound_service.cpp test/test_convert.cpp)
  target_link_libraries(mowbite_sound_test mowbite_sound_core mowbite_sound_decode)
  target_compile_definitions(mowbite_sound_test PRIVATE
    TEST_DATA="${CMAKE_CURRENT_SOURCE_DIR}/test/data"
    CONVERTER="$<TARGET_FILE:mowbite_sound_convert>")
  # For the launch test of mowbite_sim; not in the image, which builds without tests.
  install(FILES test/data/tone.mp3 DESTINATION share/${PROJECT_NAME}/test_data)
endif()

ament_package()
