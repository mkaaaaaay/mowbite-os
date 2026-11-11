cmake_minimum_required(VERSION 3.8)
project(mowbite_camera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Threads REQUIRED)
find_package(mowbite_api REQUIRED)
find_package(mowbite_bridge REQUIRED)
find_package(mowbite_msgs REQUIRED)
find_package(mowbite_watch REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# The list, the places, the HTTP server, the methods and the supervision of the stream server without ROS.
add_library(mowbite_camera_core STATIC src/seats.cpp src/camera_list.cpp src/http_server.cpp src/route.cpp
  src/camera_service.cpp src/stream_config.cpp src/child.cpp src/stream_supervisor.cpp src/http_client.cpp
  src/stream_server.cpp)
target_include_directories(mowbite_camera_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowbite_camera_core nlohmann_json::nlohmann_json Threads::Threads
  mowbite_api::mowbite_api_core mowbite_bridge::mowbite_bridge_core)

add_executable(mowbite_camera_node src/camera_node.cpp)
target_link_libraries(mowbite_camera_node mowbite_camera_core mowbite_watch::mowbite_watch)
ament_target_dependencies(mowbite_camera_node ament_index_cpp rclcpp mowbite_msgs sensor_msgs)

install(TARGETS mowbite_camera_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_camera_test test/test_seats.cpp test/test_camera_list.cpp test/test_http_server.cpp
    test/test_camera_service.cpp test/test_stream_config.cpp test/test_child.cpp test/test_stream_supervisor.cpp
    test/test_http_client.cpp test/test_stream_server.cpp test/test_stream_smoke.cpp)
  target_link_libraries(mowbite_camera_test mowbite_camera_core)
endif()

ament_package()
