cmake_minimum_required(VERSION 3.8)
project(amr_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS ${PROJECT_NAME}/survey_runner.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME survey_runner)

install(DIRECTORY config launch maps tools DESTINATION share/${PROJECT_NAME}
        PATTERN "__pycache__" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # EVERY test file in test/ is registered here, and that is checked by
  # test_every_test_file_is_registered. Eleven files across three packages
  # were never registered and therefore never ran under `colcon test` or in
  # the container; they only ran when somebody invoked pytest over src/ by
  # hand. They were passing, which is why it went unnoticed for the whole
  # project, and is exactly why the gap is worth closing: a test nothing
  # runs is a test nothing can fail.
  ament_add_pytest_test(test_nav2_config test/test_nav2_config.py)
  ament_add_pytest_test(test_survey_exit test/test_survey_exit.py)
endif()

ament_package()
