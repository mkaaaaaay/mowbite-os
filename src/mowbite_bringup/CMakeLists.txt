cmake_minimum_required(VERSION 3.8)
project(mowbite_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # The tests of the platform packages build their start with it.
  install(FILES test/stack_probe.py DESTINATION share/${PROJECT_NAME}/test)
  add_test(NAME test_data_store COMMAND python3 -m pytest -q -p no:cacheprovider
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_data_store.py)
  add_test(NAME test_migrate COMMAND python3 -m pytest -q -p no:cacheprovider
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_migrate.py)
  add_test(NAME test_layers COMMAND python3 -m pytest -q -p no:cacheprovider
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_layers.py)
  add_test(NAME test_stack COMMAND python3 -m pytest -q -p no:cacheprovider
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_stack.py)
endif()

ament_package()
