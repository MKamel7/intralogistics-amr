cmake_minimum_required(VERSION 3.8)
project(amr_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# The proxemic costmap layer. C++ because nav2 loads costmap layers through
# pluginlib and there is no Python plugin interface for them.
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

add_library(amr_proxemic_layer SHARED src/proxemic_layer.cpp)
target_include_directories(amr_proxemic_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(amr_proxemic_layer
  nav2_costmap_2d pluginlib rclcpp tf2_geometry_msgs vision_msgs)
pluginlib_export_plugin_description_file(nav2_costmap_2d proxemic_layer.xml)

install(TARGETS amr_proxemic_layer
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

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
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_proxemic_core test/test_proxemic_core.cpp)
  target_include_directories(test_proxemic_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
endif()

ament_export_include_directories(include)
ament_export_libraries(amr_proxemic_layer)

ament_package()
