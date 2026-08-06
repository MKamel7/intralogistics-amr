cmake_minimum_required(VERSION 3.8)
project(amr_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config rviz
        DESTINATION share/${PROJECT_NAME}
        PATTERN "__pycache__" EXCLUDE)
ament_package()
