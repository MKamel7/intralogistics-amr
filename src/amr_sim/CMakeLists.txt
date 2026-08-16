cmake_minimum_required(VERSION 3.8)
project(amr_sim)

find_package(ament_cmake REQUIRED)

# Python nodes live under amr_sim/ and are installed as executables. The
# package stays ament_cmake because its main payload is world and model assets.
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS ${PROJECT_NAME}/battery_model.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME battery_model)
install(PROGRAMS ${PROJECT_NAME}/pedestrian_driver.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME pedestrian_driver)
install(PROGRAMS ${PROJECT_NAME}/ground_truth_publisher.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME ground_truth_publisher)

install(PROGRAMS ${PROJECT_NAME}/truth_map_publisher.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME truth_map_publisher)

install(DIRECTORY models worlds tools scenarios launch maps
        DESTINATION share/${PROJECT_NAME}
        PATTERN "__pycache__" EXCLUDE)

# Gazebo resolves model:// against GZ_SIM_RESOURCE_PATH. Export the installed
# models directory so a sourced workspace can load the world without the caller
# setting anything by hand.
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # EVERY test file in test/ is registered here, and that is checked by
  # test_every_test_file_is_registered. Eleven files across three packages
  # were never registered and therefore never ran under `colcon test` or in
  # the container; they only ran when somebody invoked pytest over src/ by
  # hand. They were passing, which is why it went unnoticed for the whole
  # project, and is exactly why the gap is worth closing: a test nothing
  # runs is a test nothing can fail.
  ament_add_pytest_test(test_battery_model test/test_battery_model.py)
  ament_add_pytest_test(test_test_track test/test_test_track.py)
  ament_add_pytest_test(test_worlds test/test_worlds.py)
  ament_add_pytest_test(test_pedestrian_behaviours test/test_pedestrian_behaviours.py)
  ament_add_pytest_test(test_pedestrian_recovery test/test_pedestrian_recovery.py)
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
