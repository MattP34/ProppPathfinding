cmake_minimum_required(VERSION 3.10)

enable_testing()

# set the project name
project(Pathfinding)

file(GLOB files "test/*.cpp")
foreach(file ${files})
	string(REGEX REPLACE "(^.*/|\\.[^.]*$)" "" file_without_ext ${file})
	add_executable(${file_without_ext} ${file} src/spline.cpp src/trajectory.cpp src/math_util.cpp src/io_helpers.cpp)
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

add_executable(pathfinding src/main.cpp src/spline.cpp src/trajectory.cpp src/math_util.cpp src/io_helpers.cpp)

target_include_directories(pathfinding PUBLIC 
							"include"
							)