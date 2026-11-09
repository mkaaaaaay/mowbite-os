cmake_minimum_required(VERSION 3.8)
project(tron_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mowbite_msgs REQUIRED)
find_package(tron_mcu REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TURBOJPEG REQUIRED IMPORTED_TARGET libturbojpeg)

# Camera, exposure, NPU model, detection decoding and pictures without ROS. The NPU runtime is loaded at run time, never linked.
add_library(tron_vision_core STATIC src/npu_model.cpp src/model_check.cpp src/yolo_decode.cpp src/frame_convert.cpp
  src/exposure.cpp src/v4l2_camera.cpp src/crash_guard.cpp src/budget.cpp src/vision_loop.cpp src/tron_camera.cpp
  src/text_file.cpp src/preview.cpp)
target_include_directories(tron_vision_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(tron_vision_core ${CMAKE_DL_LIBS} PkgConfig::TURBOJPEG)
set_target_properties(tron_vision_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The node, a process of its own outside the container: a crash of the NPU runtime takes nothing else along.
add_library(tron_vision_node_lib STATIC src/vision_node.cpp)
target_link_libraries(tron_vision_node_lib tron_vision_core tron_mcu::tron_mcu_core)
ament_target_dependencies(tron_vision_node_lib rclcpp mowbite_msgs sensor_msgs)
add_executable(tron_vision_node src/vision_node_main.cpp)
target_link_libraries(tron_vision_node tron_vision_node_lib)
install(TARGETS tron_vision_node RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS tron_vision_core
  EXPORT export_tron_vision
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Stand-in for the runtime library, for the tests only.
  add_library(tron_vision_fake_npu SHARED test/fake_npu_runtime.cpp)
  target_include_directories(tron_vision_fake_npu PRIVATE include)
  ament_add_gtest(tron_vision_test test/test_model_check.cpp test/test_yolo_decode.cpp test/test_frame_convert.cpp
                  test/test_exposure.cpp test/test_npu_model.cpp test/test_crash_guard.cpp test/test_budget.cpp
                  test/test_vision_loop.cpp test/test_tron_camera.cpp test/test_preview.cpp)
  target_link_libraries(tron_vision_test tron_vision_core)
  target_compile_definitions(tron_vision_test PRIVATE FAKE_NPU_LIB="$<TARGET_FILE:tron_vision_fake_npu>")
  add_dependencies(tron_vision_test tron_vision_fake_npu)
  ament_add_gtest(tron_vision_node_test test/test_vision_node.cpp)
  target_link_libraries(tron_vision_node_test tron_vision_node_lib)
endif()

ament_export_targets(export_tron_vision HAS_LIBRARY_TARGET)
ament_package()
