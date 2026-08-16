cmake_minimum_required(VERSION 3.8)
project(amr_vda5050)

find_package(ament_cmake REQUIRED)

# ament_cmake rather than ament_python, matching the other eight packages. The
# package was ament_python while it was amr_fleet and its tests were silently
# not collected by `colcon test`: pytest ran in the build directory, found
# nothing, and exited 5, which colcon reports as a package failure. The 19
# protocol tests were passing the whole time under a direct pytest run, which
# is exactly the sort of gap that only shows up when something else fails.
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS ${PROJECT_NAME}/vda5050_bridge.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME vda5050_bridge)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_vda5050 test/test_vda5050.py)
endif()

ament_package()
