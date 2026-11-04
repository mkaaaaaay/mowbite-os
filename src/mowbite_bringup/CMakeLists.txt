cmake_minimum_required(VERSION 3.8)
project(mowbite_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_test(NAME test_data_store COMMAND python3 -m pytest -q -p no:cacheprovider
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_data_store.py)
endif()

ament_package()
