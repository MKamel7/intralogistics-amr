cmake_minimum_required(VERSION 3.8)
project(amr_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY models worlds tools
        DESTINATION share/${PROJECT_NAME}
        PATTERN "__pycache__" EXCLUDE)

# Gazebo resolves model:// against GZ_SIM_RESOURCE_PATH. Export the installed
# models directory so a sourced workspace can load the world without the caller
# setting anything by hand.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
