cmake_minimum_required(VERSION 3.8)
project(sim_tests)

########
# test #
########
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  # Simulator tests
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()

  add_ros_isolated_launch_test(
    test/test_simulation.py
    TIMEOUT 60
  )
  # add_ros_isolated_launch_test(
  #   test/test_simulation.py
  #   ARGS "map:=levine" "algorithm:=disparity_extender"
  # )
  # add_ros_isolated_launch_test(
  #   test/test_simulation.py
  #   ARGS "map:=levine" "algorithm:=pure_pursuit"
  # )
  # add_ros_isolated_launch_test(
  #   test/test_simulation.py
  #   ARGS "map:=levine" "algorithm:=reactive"
  # )
endif()

ament_package()