cmake_minimum_required(VERSION 3.8)
project(tron_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)

# Camera, exposure, NPU model and detection decoding without ROS. The NPU runtime is loaded at run time, never linked.
add_library(tron_vision_core STATIC src/npu_model.cpp src/model_check.cpp src/yolo_decode.cpp src/frame_convert.cpp
  src/exposure.cpp src/v4l2_camera.cpp src/crash_guard.cpp src/budget.cpp src/vision_loop.cpp src/tron_camera.cpp)
target_include_directories(tron_vision_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(tron_vision_core ${CMAKE_DL_LIBS})
set_target_properties(tron_vision_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

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
                  test/test_vision_loop.cpp test/test_tron_camera.cpp)
  target_link_libraries(tron_vision_test tron_vision_core)
  target_compile_definitions(tron_vision_test PRIVATE FAKE_NPU_LIB="$<TARGET_FILE:tron_vision_fake_npu>")
  add_dependencies(tron_vision_test tron_vision_fake_npu)
endif()

ament_export_targets(export_tron_vision HAS_LIBRARY_TARGET)
ament_package()
