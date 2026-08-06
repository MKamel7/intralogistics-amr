cmake_minimum_required(VERSION 3.8)
project(amr_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(amr_perception_core SHARED src/leg_detector_core.cpp)
target_include_directories(amr_perception_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(scan_merger src/scan_merger_node.cpp)
target_include_directories(scan_merger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(scan_merger rclcpp sensor_msgs tf2 tf2_ros)

add_executable(leg_detector src/leg_detector_node.cpp)
target_include_directories(leg_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(leg_detector amr_perception_core)
ament_target_dependencies(leg_detector rclcpp sensor_msgs vision_msgs geometry_msgs)

install(TARGETS amr_perception_core
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS scan_merger leg_detector DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # The merge geometry is deliberately free of ROS so it can be tested here in
  # milliseconds rather than against a running simulator.
  ament_add_gtest(test_scan_merge_core test/test_scan_merge_core.cpp)
  target_include_directories(test_scan_merge_core PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

  ament_add_gtest(test_leg_detector_core test/test_leg_detector_core.cpp)
  target_link_libraries(test_leg_detector_core amr_perception_core)
endif()

ament_package()
