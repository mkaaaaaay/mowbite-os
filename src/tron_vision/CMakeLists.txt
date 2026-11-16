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
find_package(mowbite_watch REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TURBOJPEG REQUIRED IMPORTED_TARGET libturbojpeg)

# Cameras, exposure, NPU model, detection decoding, pictures, the H.264 stream and the marker of the docking station
# without ROS. The NPU runtime and the media library of the encoder are loaded at run time, never linked.
add_library(tron_vision_core STATIC src/npu_model.cpp src/model_check.cpp src/yolo_decode.cpp src/frame_convert.cpp
  src/exposure.cpp src/v4l2_camera.cpp src/crash_guard.cpp src/budget.cpp src/vision_loop.cpp src/tron_camera.cpp
  src/text_file.cpp src/preview.cpp src/h264_rtp.cpp src/rtsp_server.cpp src/h264_encoder.cpp src/h264_stream.cpp
  src/stereo_camera.cpp src/stereo_loop.cpp src/lens_model.cpp src/dock_marker.cpp src/dock_pose.cpp
  src/dock_exposure.cpp src/jpeg_luma.cpp src/rear_camera.cpp src/rear_loop.cpp src/preview_loop.cpp src/inference_worker.cpp
  src/video_feed.cpp src/dma_frames.cpp src/frame_recorder.cpp src/gain_level.cpp)
target_include_directories(tron_vision_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(tron_vision_core ${CMAKE_DL_LIBS} PkgConfig::TURBOJPEG)
set_target_properties(tron_vision_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The node, a process of its own outside the container: a crash of the NPU runtime takes nothing else along.
# The H.264 stream of the camera nodes with its parameters.
add_library(tron_video_out_lib STATIC src/video_out.cpp)
target_link_libraries(tron_video_out_lib tron_vision_core)
ament_target_dependencies(tron_video_out_lib rclcpp mowbite_msgs ament_index_cpp)

add_library(tron_vision_node_lib STATIC src/vision_node.cpp)
target_link_libraries(tron_vision_node_lib tron_vision_core tron_video_out_lib tron_mcu::tron_mcu_core)
ament_target_dependencies(tron_vision_node_lib rclcpp mowbite_msgs sensor_msgs)
add_executable(tron_vision_node src/vision_node_main.cpp)
target_link_libraries(tron_vision_node tron_vision_node_lib)
install(TARGETS tron_vision_node RUNTIME DESTINATION lib/${PROJECT_NAME})

# The live picture of the stereo camera, a process of its own: a crash in the media library takes nothing else along.
add_library(tron_stereo_node_lib STATIC src/stereo_node.cpp)
target_link_libraries(tron_stereo_node_lib tron_vision_core tron_mcu::tron_mcu_core mowbite_watch::mowbite_watch)
ament_target_dependencies(tron_stereo_node_lib rclcpp mowbite_msgs ament_index_cpp)
add_executable(tron_stereo_node src/stereo_node_main.cpp)
target_link_libraries(tron_stereo_node tron_stereo_node_lib)
install(TARGETS tron_stereo_node RUNTIME DESTINATION lib/${PROJECT_NAME})

# The live picture of the right side camera, a process of its own beside the camera AI on the left one.
add_library(tron_side_node_lib STATIC src/side_node.cpp)
target_link_libraries(tron_side_node_lib tron_vision_core tron_video_out_lib tron_mcu::tron_mcu_core)
ament_target_dependencies(tron_side_node_lib rclcpp mowbite_msgs sensor_msgs ament_index_cpp)
add_executable(tron_side_node src/side_node_main.cpp)
target_link_libraries(tron_side_node tron_side_node_lib)
install(TARGETS tron_side_node RUNTIME DESTINATION lib/${PROJECT_NAME})

# The dock marker seen by the rear camera, a process of its own: a crash in the USB camera takes nothing else along.
add_library(tron_rear_node_lib STATIC src/rear_node.cpp)
target_link_libraries(tron_rear_node_lib tron_vision_core tron_video_out_lib tron_mcu::tron_mcu_core
  mowbite_watch::mowbite_watch)
ament_target_dependencies(tron_rear_node_lib rclcpp mowbite_msgs sensor_msgs std_msgs diagnostic_msgs)
add_executable(tron_rear_node src/rear_node_main.cpp)
target_link_libraries(tron_rear_node tron_rear_node_lib)
install(TARGETS tron_rear_node RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS tron_vision_core
  EXPORT export_tron_vision
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# With the headers of the media library at hand (TRON_VISION_MPP_INCLUDE as option or in the environment), the
# declarations in mpp_abi.h are compared with them, also in the image build.
find_path(TRON_VISION_MPP_INCLUDE rk_mpi.h HINTS ENV TRON_VISION_MPP_INCLUDE NO_DEFAULT_PATH)
if(TRON_VISION_MPP_INCLUDE)
  add_library(tron_vision_check_mpp_abi OBJECT test/check_mpp_abi.cpp)
  target_include_directories(tron_vision_check_mpp_abi PRIVATE include)
  target_include_directories(tron_vision_check_mpp_abi SYSTEM PRIVATE ${TRON_VISION_MPP_INCLUDE})
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Stand-in for the runtime library, for the tests only.
  add_library(tron_vision_fake_npu SHARED test/fake_npu_runtime.cpp)
  target_include_directories(tron_vision_fake_npu PRIVATE include)
  # Stand-in for the media library, for the tests only.
  add_library(tron_vision_fake_mpp SHARED test/fake_mpp.cpp src/h264_rtp.cpp)
  target_include_directories(tron_vision_fake_mpp PRIVATE include)
  ament_add_gtest(tron_vision_test test/test_model_check.cpp test/test_yolo_decode.cpp test/test_frame_convert.cpp
                  test/test_exposure.cpp test/test_npu_model.cpp test/test_crash_guard.cpp test/test_budget.cpp
                  test/test_vision_loop.cpp test/test_tron_camera.cpp test/test_preview.cpp
                  test/test_v4l2_capture.cpp test/test_h264_rtp.cpp test/test_rtsp_server.cpp
                  test/test_h264_encoder.cpp test/test_stereo_camera.cpp test/test_stereo_loop.cpp
                  test/test_rear_camera.cpp test/test_preview_loop.cpp test/test_dma_frames.cpp
                  test/test_gain_level.cpp
                  # The loops are tested in real time, about a minute in all.
                  TIMEOUT 180)
  target_link_libraries(tron_vision_test tron_vision_core)
  target_compile_definitions(tron_vision_test PRIVATE FAKE_NPU_LIB="$<TARGET_FILE:tron_vision_fake_npu>"
                             FAKE_MPP_LIB="$<TARGET_FILE:tron_vision_fake_mpp>")
  add_dependencies(tron_vision_test tron_vision_fake_npu tron_vision_fake_mpp)
  # The marker of the docking station on drawn images; a test of its own, it takes a while with the sanitizers.
  ament_add_gtest(tron_vision_dock_test test/test_lens_model.cpp test/test_dock_marker.cpp test/test_dock_pose.cpp
                  test/test_dock_exposure.cpp test/test_jpeg_luma.cpp test/test_rear_loop.cpp
                  test/test_frame_recorder.cpp
                  TIMEOUT 300)
  target_link_libraries(tron_vision_dock_test tron_vision_core)
  # The stream without the Tron, for a trial by hand against a real RTSP client.
  add_executable(tron_stereo_trial test/stereo_trial.cpp)
  target_link_libraries(tron_stereo_trial tron_vision_core)
  ament_add_gtest(tron_vision_node_test test/test_vision_node.cpp)
  target_link_libraries(tron_vision_node_test tron_vision_node_lib)
  target_compile_definitions(tron_vision_node_test PRIVATE FAKE_NPU_LIB="$<TARGET_FILE:tron_vision_fake_npu>")
  add_dependencies(tron_vision_node_test tron_vision_fake_npu)
  ament_add_gtest(tron_side_node_test test/test_side_node.cpp)
  target_link_libraries(tron_side_node_test tron_side_node_lib)
  target_compile_definitions(tron_side_node_test PRIVATE FAKE_MPP_LIB="$<TARGET_FILE:tron_vision_fake_mpp>")
  add_dependencies(tron_side_node_test tron_vision_fake_mpp)
  ament_add_gtest(tron_stereo_node_test test/test_stereo_node.cpp)
  target_link_libraries(tron_stereo_node_test tron_stereo_node_lib)
  target_compile_definitions(tron_stereo_node_test PRIVATE FAKE_MPP_LIB="$<TARGET_FILE:tron_vision_fake_mpp>")
  add_dependencies(tron_stereo_node_test tron_vision_fake_mpp)
  ament_add_gtest(tron_rear_node_test test/test_rear_node.cpp TIMEOUT 120)
  target_link_libraries(tron_rear_node_test tron_rear_node_lib)
  target_compile_definitions(tron_rear_node_test PRIVATE FAKE_MPP_LIB="$<TARGET_FILE:tron_vision_fake_mpp>")
  add_dependencies(tron_rear_node_test tron_vision_fake_mpp)
endif()

ament_export_targets(export_tron_vision HAS_LIBRARY_TARGET)
ament_package()
