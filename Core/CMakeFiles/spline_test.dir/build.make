# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /Applications/CMake.app/Contents/bin/cmake

# The command to remove a file.
RM = /Applications/CMake.app/Contents/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/matthewpropp/Documents/Robotics/Season2021/Pathfinding/Core

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/matthewpropp/Documents/Robotics/Season2021/Pathfinding/Core

# Include any dependencies generated for this target.
include CMakeFiles/spline_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/spline_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/spline_test.dir/flags.make

CMakeFiles/spline_test.dir/test/spline_test.cpp.o: CMakeFiles/spline_test.dir/flags.make
CMakeFiles/spline_test.dir/test/spline_test.cpp.o: test/spline_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/matthewpropp/Documents/Robotics/Season2021/Pathfinding/Core/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/spline_test.dir/test/spline_test.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/spline_test.dir/test/spline_test.cpp.o -c /Users/matthewpropp/Documents/Robotics/Season2021/Pathfinding/Core/test/spline_test.cpp

CMakeFiles/spline_test.dir/test/spline_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/spline_test.dir/test/spline_test.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/matthewpropp/Documents/Robotics/Season2021/Pathfinding/Core/test/spline_test.cpp > CMakeFiles/spline_test.dir/test/spline_test.cpp.i

CMakeFiles/spline_test.dir/test/spline_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/spline_test.dir/test/spline_test.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/matthewpropp/Documents/Robotics/Season2021/Pathfinding/Core/test/spline_test.cpp -o CMakeFiles/spline_test.dir/test/spline_test.cpp.s

CMakeFiles/spline_test.dir/src/spline.cpp.o: CMakeFiles/spline_test.dir/flags.make
CMakeFiles/spline_test.dir/src/spline.cpp.o: src/spline.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/matthewpropp/Documents/Robotics/Season2021/Pathfinding/Core/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/spline_test.dir/src/spline.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/spline_test.dir/src/spline.cpp.o -c /Users/matthewpropp/Documents/Robotics/Season2021/Pathfinding/Core/src/spline.cpp

CMakeFiles/spline_test.dir/src/spline.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/spline_test.dir/src/spline.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/matthewpropp/Documents/Robotics/Season2021/Pathfinding/Core/src/spline.cpp > CMakeFiles/spline_test.dir/src/spline.cpp.i

CMakeFiles/spline_test.dir/src/spline.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/spline_test.dir/src/spline.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/matthewpropp/Documents/Robotics/Season2021/Pathfinding/Core/src/spline.cpp -o CMakeFiles/spline_test.dir/src/spline.cpp.s

# Object files for target spline_test
spline_test_OBJECTS = \
"CMakeFiles/spline_test.dir/test/spline_test.cpp.o" \
"CMakeFiles/spline_test.dir/src/spline.cpp.o"

# External object files for target spline_test
spline_test_EXTERNAL_OBJECTS =

spline_test: CMakeFiles/spline_test.dir/test/spline_test.cpp.o
spline_test: CMakeFiles/spline_test.dir/src/spline.cpp.o
spline_test: CMakeFiles/spline_test.dir/build.make
spline_test: CMakeFiles/spline_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/matthewpropp/Documents/Robotics/Season2021/Pathfinding/Core/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable spline_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/spline_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/spline_test.dir/build: spline_test

.PHONY : CMakeFiles/spline_test.dir/build

CMakeFiles/spline_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/spline_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/spline_test.dir/clean

CMakeFiles/spline_test.dir/depend:
	cd /Users/matthewpropp/Documents/Robotics/Season2021/Pathfinding/Core && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/matthewpropp/Documents/Robotics/Season2021/Pathfinding/Core /Users/matthewpropp/Documents/Robotics/Season2021/Pathfinding/Core /Users/matthewpropp/Documents/Robotics/Season2021/Pathfinding/Core /Users/matthewpropp/Documents/Robotics/Season2021/Pathfinding/Core /Users/matthewpropp/Documents/Robotics/Season2021/Pathfinding/Core/CMakeFiles/spline_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/spline_test.dir/depend

