# CMake generated Testfile for 
# Source directory: /Users/matthewpropp/Documents/Robotics/Pathfinding/Core
# Build directory: /Users/matthewpropp/Documents/Robotics/Pathfinding/Core
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(quintic_test "quintic_test")
set_tests_properties(quintic_test PROPERTIES  FAIL_REGULAR_EXPRESSION "(Exception|Test failed)" PASS_REGULAR_EXPRESSION "Test passed" TIMEOUT "120" _BACKTRACE_TRIPLES "/Users/matthewpropp/Documents/Robotics/Pathfinding/Core/CMakeLists.txt;15;add_test;/Users/matthewpropp/Documents/Robotics/Pathfinding/Core/CMakeLists.txt;0;")
add_test(spline_constructor_test "spline_constructor_test")
set_tests_properties(spline_constructor_test PROPERTIES  FAIL_REGULAR_EXPRESSION "(Exception|Test failed)" PASS_REGULAR_EXPRESSION "Test passed" TIMEOUT "120" _BACKTRACE_TRIPLES "/Users/matthewpropp/Documents/Robotics/Pathfinding/Core/CMakeLists.txt;15;add_test;/Users/matthewpropp/Documents/Robotics/Pathfinding/Core/CMakeLists.txt;0;")
add_test(spline_math_test "spline_math_test")
set_tests_properties(spline_math_test PROPERTIES  FAIL_REGULAR_EXPRESSION "(Exception|Test failed)" PASS_REGULAR_EXPRESSION "Test passed" TIMEOUT "120" _BACKTRACE_TRIPLES "/Users/matthewpropp/Documents/Robotics/Pathfinding/Core/CMakeLists.txt;15;add_test;/Users/matthewpropp/Documents/Robotics/Pathfinding/Core/CMakeLists.txt;0;")
add_test(trajectory_test "trajectory_test")
set_tests_properties(trajectory_test PROPERTIES  FAIL_REGULAR_EXPRESSION "(Exception|Test failed)" PASS_REGULAR_EXPRESSION "Test passed" TIMEOUT "120" _BACKTRACE_TRIPLES "/Users/matthewpropp/Documents/Robotics/Pathfinding/Core/CMakeLists.txt;15;add_test;/Users/matthewpropp/Documents/Robotics/Pathfinding/Core/CMakeLists.txt;0;")
