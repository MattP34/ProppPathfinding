cmake_minimum_required(VERSION 3.10)

enable_testing()

# set the project name
project(Pathfinding)

file(GLOB files "test/*.cpp")
foreach(file ${files})
	string(REGEX REPLACE "(^.*/|\\.[^.]*$)" "" file_without_ext ${file})
	add_executable(${file_without_ext} ${file} src/spline.cpp src/trajectory.cpp)
	target_include_directories(${file_without_ext} PUBLIC 
							"include"
							)
	add_test(${file_without_ext} ${file_without_ext})
	set_tests_properties(${file_without_ext}
		PROPERTIES
		PASS_REGULAR_EXPRESSION "Test passed")
	set_tests_properties(${file_without_ext}
		PROPERTIES
		FAIL_REGULAR_EXPRESSION "(Exception|Test failed)")
	set_tests_properties(${file_without_ext}
		PROPERTIES
		TIMEOUT 120)
endforeach()

#[[add_executable(quintic_test test/quintic_test.cpp src/spline.cpp)

target_include_directories(quintic_test PUBLIC
                           "include"
                          )

add_test(QuinticTest quintic_test)
set_tests_properties(QuinticTest
	PROPERTIES
	PASS_REGULAR_EXPRESSION "Test passed")
set_tests_properties(QuinticTest
	PROPERTIES
	FAIL_REGULAR_EXPRESSION "Test failed")

add_executable(spline_test test/spline_test.cpp src/spline.cpp)

target_include_directories(spline_test PUBLIC
                           "include"
                           )

add_test(SplineTest spline_test)
set_tests_properties(SplineTest
	PROPERTIES
	PASS_REGULAR_EXPRESSION "Test passed")
set_tests_properties(SplineTest
	PROPERTIES
	FAIL_REGULAR_EXPRESSION "Test failed")]]