cmake_minimum_required(VERSION 3.8)
project(amr_common)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_pose test/test_pose.py)
  ament_add_pytest_test(test_topics test/test_topics.py)
  ament_add_pytest_test(test_no_second_implementation test/test_no_second_implementation.py)
endif()

ament_package()
