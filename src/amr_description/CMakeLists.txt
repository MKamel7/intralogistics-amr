cmake_minimum_required(VERSION 3.8)
project(amr_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config urdf
        DESTINATION share/${PROJECT_NAME}
        PATTERN "__pycache__" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Provenance gate: every platform constant must trace to a recorded source.
  ament_add_pytest_test(test_platform_spec test/test_platform_spec.py)
  ament_add_pytest_test(test_description test/test_description.py)
  ament_add_pytest_test(test_payload test/test_payload.py)
endif()

ament_package()
