cmake_minimum_required(VERSION 3.8)
project(mowbite_sim)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/fake_mcu DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
