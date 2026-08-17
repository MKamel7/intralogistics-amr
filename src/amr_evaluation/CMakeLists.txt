cmake_minimum_required(VERSION 3.8)
project(amr_evaluation)
find_package(ament_cmake REQUIRED)
install(DIRECTORY tools results
        DESTINATION share/${PROJECT_NAME}
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
  ament_add_pytest_test(test_demo_cut test/test_demo_cut.py)
  ament_add_pytest_test(test_film test/test_film.py)
  ament_add_pytest_test(test_experiment test/test_experiment.py)
  ament_add_pytest_test(test_container test/test_container.py)
  ament_add_pytest_test(test_measure_contacts test/test_measure_contacts.py)
  ament_add_pytest_test(test_measure_localisation test/test_measure_localisation.py)
  ament_add_pytest_test(test_measure_path_efficiency test/test_measure_path_efficiency.py)
  ament_add_pytest_test(test_measure_slip test/test_measure_slip.py)
  ament_add_pytest_test(test_measure_social test/test_measure_social.py)
  ament_add_pytest_test(test_occlusion test/test_occlusion.py)
  ament_add_pytest_test(test_run_stack test/test_run_stack.py)
  ament_add_pytest_test(test_measure_control_latency test/test_measure_control_latency.py)
  ament_add_pytest_test(test_measure_braking test/test_measure_braking.py)
  ament_add_pytest_test(test_measure_load test/test_measure_load.py)
  ament_add_pytest_test(test_measure_docking test/test_measure_docking.py)
  ament_add_pytest_test(test_probe_clocks test/test_probe_clocks.py)
  ament_add_pytest_test(test_registration test/test_registration.py)
endif()

ament_package()
