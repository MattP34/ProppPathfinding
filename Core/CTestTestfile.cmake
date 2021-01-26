# CMake generated Testfile for 
# Source directory: /Users/matthewpropp/Documents/Robotics/Season2021/Pathfinding/Core
# Build directory: /Users/matthewpropp/Documents/Robotics/Season2021/Pathfinding/Core
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(QuinticTest "quintic_test")
set_tests_properties(QuinticTest PROPERTIES  FAIL_REGULAR_EXPRESSION "Test failed" PASS_REGULAR_EXPRESSION "Test passed")
add_test(SplineTest "spline_test")
set_tests_properties(SplineTest PROPERTIES  FAIL_REGULAR_EXPRESSION "Test failed" PASS_REGULAR_EXPRESSION "Test passed")
