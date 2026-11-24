cmake_minimum_required(VERSION 3.8)
project(mowbite_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# No fused multiply-add, so amd64 and arm64 compute the same results
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mowbite_component REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mowbite_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(PahoMqttCpp REQUIRED)
find_package(mowbite_api REQUIRED)
find_package(mowbite_watch REQUIRED)

# The schema of data_dir this image writes: one file for system.info here and for the start of the stack, installed with
# the package.
file(READ data_schema.json DATA_SCHEMA_JSON)
string(JSON DATA_SCHEMA GET "${DATA_SCHEMA_JSON}" schema)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS data_schema.json)

# Everything of the bridge without ROS and without an MQTT client, testable on any machine; the nodes that send
# events link it for BuildEvent.
add_library(mowbite_bridge_core STATIC src/text.cpp src/json_rpc.cpp src/method_table.cpp src/app_json.cpp
  src/param_table.cpp src/event_builder.cpp src/warn_log.cpp src/bridge_core.cpp src/event_log.cpp src/track.cpp
  src/pose_window.cpp src/health.cpp src/system_stats.cpp src/compat.cpp)
target_include_directories(mowbite_bridge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowbite_bridge_core mowbite_api::mowbite_api_core nlohmann_json::nlohmann_json)
target_compile_definitions(mowbite_bridge_core PRIVATE MOWBITE_DATA_SCHEMA=${DATA_SCHEMA})
set_target_properties(mowbite_bridge_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# HEALTHCHECK of the image, without ROS.
add_executable(mowbite_health src/health_main.cpp)
target_link_libraries(mowbite_health mowbite_bridge_core PahoMqttCpp::paho-mqttpp3)

add_library(mowbite_bridge_component SHARED src/bridge_component.cpp)
target_link_libraries(mowbite_bridge_component mowbite_bridge_core PahoMqttCpp::paho-mqttpp3
  mowbite_watch::mowbite_watch)
ament_target_dependencies(mowbite_bridge_component rclcpp rclcpp_components rcl_interfaces geometry_msgs std_msgs mowbite_msgs)
mowbite_component_register(mowbite_bridge_component
  PLUGIN "mowbite_bridge::BridgeNode"
  EXECUTABLE mowbite_bridge_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mowbite_bridge_core mowbite_bridge_component
  EXPORT export_mowbite_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS mowbite_health DESTINATION lib/${PROJECT_NAME})
install(FILES data_schema.json DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mowbite_bridge_test test/test_text.cpp test/test_json_rpc.cpp test/test_app_json.cpp
    test/test_param_table.cpp test/test_warn_log.cpp test/test_bridge_core.cpp test/test_bridge_fuzz.cpp
    test/test_pose_window.cpp test/test_event_log.cpp test/test_track.cpp test/test_health.cpp
    test/test_system_stats.cpp test/test_compat.cpp TIMEOUT 300)
  target_link_libraries(mowbite_bridge_test mowbite_bridge_core)
endif()

ament_export_targets(export_mowbite_bridge HAS_LIBRARY_TARGET)
ament_export_dependencies(mowbite_api)
ament_package()
