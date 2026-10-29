cmake_minimum_required(VERSION 3.8)
project(mowbite_sim)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/fake_tron scripts/sim_control DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_stack_smoke.py TIMEOUT 120)
  add_launch_test(test/test_hw_contract.py TIMEOUT 120)
  add_launch_test(test/test_map.py TIMEOUT 180)
  add_launch_test(test/test_recording.py TIMEOUT 120)
  add_launch_test(test/test_follower.py TIMEOUT 240)
  add_launch_test(test/test_drive.py TIMEOUT 400)
  add_launch_test(test/test_mowing.py TIMEOUT 900)
  add_launch_test(test/test_follower_stall.py TIMEOUT 240)
  add_launch_test(test/test_safety_inputs.py TIMEOUT 120)
endif()

ament_package()
