cmake_minimum_required(VERSION 3.8)
project(amr_mission)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS ${PROJECT_NAME}/transport_task.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME transport_task)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_stations test/test_stations.py)
endif()

ament_package()
