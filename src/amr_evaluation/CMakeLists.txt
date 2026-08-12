cmake_minimum_required(VERSION 3.8)
project(amr_evaluation)
find_package(ament_cmake REQUIRED)
install(DIRECTORY tools results
        DESTINATION share/${PROJECT_NAME}
        PATTERN "__pycache__" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_experiment test/test_experiment.py)
endif()

ament_package()
