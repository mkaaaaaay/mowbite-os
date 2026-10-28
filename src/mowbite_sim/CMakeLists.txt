cmake_minimum_required(VERSION 3.8)
project(mowbite_sim)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/fake_tron DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_stack_smoke.py TIMEOUT 120)
  add_launch_test(test/test_hw_contract.py TIMEOUT 120)
  add_launch_test(test/test_map.py TIMEOUT 180)
endif()

ament_package()
