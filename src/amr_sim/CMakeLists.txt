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

install(DIRECTORY models worlds tools scenarios launch
        DESTINATION share/${PROJECT_NAME}
        PATTERN "__pycache__" EXCLUDE)

# Gazebo resolves model:// against GZ_SIM_RESOURCE_PATH. Export the installed
# models directory so a sourced workspace can load the world without the caller
# setting anything by hand.
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_battery_model test/test_battery_model.py)
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
