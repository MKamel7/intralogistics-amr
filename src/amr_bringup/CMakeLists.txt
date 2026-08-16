cmake_minimum_required(VERSION 3.8)
project(amr_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config rviz
        DESTINATION share/${PROJECT_NAME}
        PATTERN "__pycache__" EXCLUDE)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # EVERY test file in test/ is registered here, and that is checked by
  # test_every_test_file_is_registered in amr_evaluation.
  ament_add_pytest_test(test_pod_params test/test_pod_params.py)
endif()

ament_package()
