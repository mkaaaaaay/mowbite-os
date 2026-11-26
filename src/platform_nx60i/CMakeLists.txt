cmake_minimum_required(VERSION 3.8)
project(platform_nx60i)

find_package(ament_cmake REQUIRED)

install(FILES platform.py platform.yaml geometry.yaml DESTINATION share/${PROJECT_NAME})
# The mowers with this board whose values differ (step model of the setup program).
install(DIRECTORY profiles DESTINATION share/${PROJECT_NAME})
# The steps of the setup program and their images.
install(DIRECTORY setup DESTINATION share/${PROJECT_NAME})
# The name mowbite.launch.py finds this platform by (platform:=nx60i).
ament_index_register_resource(mowbite_platform PACKAGE_NAME nx60i CONTENT ${PROJECT_NAME})

if(BUILD_TESTING)
  add_test(NAME test_platform COMMAND python3 -m pytest -q -p no:cacheprovider
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_platform.py)
endif()

ament_package()
