# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.21

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /Users/alexlatz/.brew/Cellar/cmake/3.21.2/bin/cmake

# The command to remove a file.
RM = /Users/alexlatz/.brew/Cellar/cmake/3.21.2/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/alexlatz/Documents/Pathfinding/Core

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/alexlatz/Documents/Pathfinding/Core/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/pathfinding.dir/depend.make
# Include the progress variables for this target.
include CMakeFiles/pathfinding.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pathfinding.dir/flags.make

CMakeFiles/pathfinding.dir/src/main.cpp.o: CMakeFiles/pathfinding.dir/flags.make
CMakeFiles/pathfinding.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/alexlatz/Documents/Pathfinding/Core/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/pathfinding.dir/src/main.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pathfinding.dir/src/main.cpp.o -c /Users/alexlatz/Documents/Pathfinding/Core/src/main.cpp

CMakeFiles/pathfinding.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pathfinding.dir/src/main.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/alexlatz/Documents/Pathfinding/Core/src/main.cpp > CMakeFiles/pathfinding.dir/src/main.cpp.i

CMakeFiles/pathfinding.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pathfinding.dir/src/main.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/alexlatz/Documents/Pathfinding/Core/src/main.cpp -o CMakeFiles/pathfinding.dir/src/main.cpp.s

CMakeFiles/pathfinding.dir/src/spline.cpp.o: CMakeFiles/pathfinding.dir/flags.make
CMakeFiles/pathfinding.dir/src/spline.cpp.o: ../src/spline.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/alexlatz/Documents/Pathfinding/Core/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/pathfinding.dir/src/spline.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pathfinding.dir/src/spline.cpp.o -c /Users/alexlatz/Documents/Pathfinding/Core/src/spline.cpp

CMakeFiles/pathfinding.dir/src/spline.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pathfinding.dir/src/spline.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/alexlatz/Documents/Pathfinding/Core/src/spline.cpp > CMakeFiles/pathfinding.dir/src/spline.cpp.i

CMakeFiles/pathfinding.dir/src/spline.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pathfinding.dir/src/spline.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/alexlatz/Documents/Pathfinding/Core/src/spline.cpp -o CMakeFiles/pathfinding.dir/src/spline.cpp.s

CMakeFiles/pathfinding.dir/src/trajectory.cpp.o: CMakeFiles/pathfinding.dir/flags.make
CMakeFiles/pathfinding.dir/src/trajectory.cpp.o: ../src/trajectory.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/alexlatz/Documents/Pathfinding/Core/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/pathfinding.dir/src/trajectory.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pathfinding.dir/src/trajectory.cpp.o -c /Users/alexlatz/Documents/Pathfinding/Core/src/trajectory.cpp

CMakeFiles/pathfinding.dir/src/trajectory.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pathfinding.dir/src/trajectory.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/alexlatz/Documents/Pathfinding/Core/src/trajectory.cpp > CMakeFiles/pathfinding.dir/src/trajectory.cpp.i

CMakeFiles/pathfinding.dir/src/trajectory.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pathfinding.dir/src/trajectory.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/alexlatz/Documents/Pathfinding/Core/src/trajectory.cpp -o CMakeFiles/pathfinding.dir/src/trajectory.cpp.s

CMakeFiles/pathfinding.dir/src/math_util.cpp.o: CMakeFiles/pathfinding.dir/flags.make
CMakeFiles/pathfinding.dir/src/math_util.cpp.o: ../src/math_util.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/alexlatz/Documents/Pathfinding/Core/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/pathfinding.dir/src/math_util.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pathfinding.dir/src/math_util.cpp.o -c /Users/alexlatz/Documents/Pathfinding/Core/src/math_util.cpp

CMakeFiles/pathfinding.dir/src/math_util.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pathfinding.dir/src/math_util.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/alexlatz/Documents/Pathfinding/Core/src/math_util.cpp > CMakeFiles/pathfinding.dir/src/math_util.cpp.i

CMakeFiles/pathfinding.dir/src/math_util.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pathfinding.dir/src/math_util.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/alexlatz/Documents/Pathfinding/Core/src/math_util.cpp -o CMakeFiles/pathfinding.dir/src/math_util.cpp.s

CMakeFiles/pathfinding.dir/src/io_helpers.cpp.o: CMakeFiles/pathfinding.dir/flags.make
CMakeFiles/pathfinding.dir/src/io_helpers.cpp.o: ../src/io_helpers.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/alexlatz/Documents/Pathfinding/Core/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/pathfinding.dir/src/io_helpers.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pathfinding.dir/src/io_helpers.cpp.o -c /Users/alexlatz/Documents/Pathfinding/Core/src/io_helpers.cpp

CMakeFiles/pathfinding.dir/src/io_helpers.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pathfinding.dir/src/io_helpers.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/alexlatz/Documents/Pathfinding/Core/src/io_helpers.cpp > CMakeFiles/pathfinding.dir/src/io_helpers.cpp.i

CMakeFiles/pathfinding.dir/src/io_helpers.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pathfinding.dir/src/io_helpers.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/alexlatz/Documents/Pathfinding/Core/src/io_helpers.cpp -o CMakeFiles/pathfinding.dir/src/io_helpers.cpp.s

# Object files for target pathfinding
pathfinding_OBJECTS = \
"CMakeFiles/pathfinding.dir/src/main.cpp.o" \
"CMakeFiles/pathfinding.dir/src/spline.cpp.o" \
"CMakeFiles/pathfinding.dir/src/trajectory.cpp.o" \
"CMakeFiles/pathfinding.dir/src/math_util.cpp.o" \
"CMakeFiles/pathfinding.dir/src/io_helpers.cpp.o"

# External object files for target pathfinding
pathfinding_EXTERNAL_OBJECTS =

pathfinding: CMakeFiles/pathfinding.dir/src/main.cpp.o
pathfinding: CMakeFiles/pathfinding.dir/src/spline.cpp.o
pathfinding: CMakeFiles/pathfinding.dir/src/trajectory.cpp.o
pathfinding: CMakeFiles/pathfinding.dir/src/math_util.cpp.o
pathfinding: CMakeFiles/pathfinding.dir/src/io_helpers.cpp.o
pathfinding: CMakeFiles/pathfinding.dir/build.make
pathfinding: CMakeFiles/pathfinding.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/alexlatz/Documents/Pathfinding/Core/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable pathfinding"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pathfinding.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pathfinding.dir/build: pathfinding
.PHONY : CMakeFiles/pathfinding.dir/build

CMakeFiles/pathfinding.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pathfinding.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pathfinding.dir/clean

CMakeFiles/pathfinding.dir/depend:
	cd /Users/alexlatz/Documents/Pathfinding/Core/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/alexlatz/Documents/Pathfinding/Core /Users/alexlatz/Documents/Pathfinding/Core /Users/alexlatz/Documents/Pathfinding/Core/cmake-build-debug /Users/alexlatz/Documents/Pathfinding/Core/cmake-build-debug /Users/alexlatz/Documents/Pathfinding/Core/cmake-build-debug/CMakeFiles/pathfinding.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pathfinding.dir/depend

