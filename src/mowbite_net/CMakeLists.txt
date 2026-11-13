cmake_minimum_required(VERSION 3.8)
project(mowbite_net)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(CURL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mowbite_component REQUIRED)
find_package(mowbite_msgs REQUIRED)
find_package(mowbite_api REQUIRED)
find_package(mowbite_hold REQUIRED)
find_package(mowbite_watch REQUIRED)

# Settings, notifier, outbox and weather without network, clock, files or ROS.
add_library(mowbite_net_core STATIC src/texts.cpp src/notify_config.cpp src/notifier.cpp src/outbox.cpp
  src/weather.cpp)
target_include_directories(mowbite_net_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowbite_net_core PUBLIC nlohmann_json::nlohmann_json mowbite_api::mowbite_api_core)
set_target_properties(mowbite_net_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(mowbite_net_component SHARED src/net_node.cpp src/curl_http.cpp)
target_link_libraries(mowbite_net_component mowbite_net_core CURL::libcurl mowbite_hold::mowbite_hold_core
  mowbite_watch::mowbite_watch)
ament_target_dependencies(mowbite_net_component rclcpp rclcpp_components mowbite_msgs)
mowbite_component_register(mowbite_net_component
  PLUGIN "mowbite_net::NetNode"
  EXECUTABLE mowbite_net_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_net_core mowbite_net_component
  EXPORT export_mowbite_net
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_net_test test/test_notify_config.cpp test/test_notifier.cpp test/test_outbox.cpp
    test/test_weather.cpp)
  target_link_libraries(mowbite_net_test mowbite_net_core)
  add_test(NAME test_net_topics COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_topics.py)
endif()

ament_export_targets(export_mowbite_net HAS_LIBRARY_TARGET)
ament_package()
