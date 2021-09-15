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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/riki/Test/rgbdSlam/RGBD-SLAM/sophus_test/sophus_test_cpp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/riki/Test/rgbdSlam/RGBD-SLAM/sophus_test/sophus_test_cpp/build

# Include any dependencies generated for this target.
include CMakeFiles/useSophus.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/useSophus.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/useSophus.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/useSophus.dir/flags.make

CMakeFiles/useSophus.dir/test.cpp.o: CMakeFiles/useSophus.dir/flags.make
CMakeFiles/useSophus.dir/test.cpp.o: ../test.cpp
CMakeFiles/useSophus.dir/test.cpp.o: CMakeFiles/useSophus.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/riki/Test/rgbdSlam/RGBD-SLAM/sophus_test/sophus_test_cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/useSophus.dir/test.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/useSophus.dir/test.cpp.o -MF CMakeFiles/useSophus.dir/test.cpp.o.d -o CMakeFiles/useSophus.dir/test.cpp.o -c /home/riki/Test/rgbdSlam/RGBD-SLAM/sophus_test/sophus_test_cpp/test.cpp

CMakeFiles/useSophus.dir/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/useSophus.dir/test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/riki/Test/rgbdSlam/RGBD-SLAM/sophus_test/sophus_test_cpp/test.cpp > CMakeFiles/useSophus.dir/test.cpp.i

CMakeFiles/useSophus.dir/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/useSophus.dir/test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/riki/Test/rgbdSlam/RGBD-SLAM/sophus_test/sophus_test_cpp/test.cpp -o CMakeFiles/useSophus.dir/test.cpp.s

# Object files for target useSophus
useSophus_OBJECTS = \
"CMakeFiles/useSophus.dir/test.cpp.o"

# External object files for target useSophus
useSophus_EXTERNAL_OBJECTS =

useSophus: CMakeFiles/useSophus.dir/test.cpp.o
useSophus: CMakeFiles/useSophus.dir/build.make
useSophus: /home/riki/Test/sophus/Sophus/build/libSophus.so
useSophus: CMakeFiles/useSophus.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/riki/Test/rgbdSlam/RGBD-SLAM/sophus_test/sophus_test_cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable useSophus"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/useSophus.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/useSophus.dir/build: useSophus
.PHONY : CMakeFiles/useSophus.dir/build

CMakeFiles/useSophus.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/useSophus.dir/cmake_clean.cmake
.PHONY : CMakeFiles/useSophus.dir/clean

CMakeFiles/useSophus.dir/depend:
	cd /home/riki/Test/rgbdSlam/RGBD-SLAM/sophus_test/sophus_test_cpp/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/riki/Test/rgbdSlam/RGBD-SLAM/sophus_test/sophus_test_cpp /home/riki/Test/rgbdSlam/RGBD-SLAM/sophus_test/sophus_test_cpp /home/riki/Test/rgbdSlam/RGBD-SLAM/sophus_test/sophus_test_cpp/build /home/riki/Test/rgbdSlam/RGBD-SLAM/sophus_test/sophus_test_cpp/build /home/riki/Test/rgbdSlam/RGBD-SLAM/sophus_test/sophus_test_cpp/build/CMakeFiles/useSophus.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/useSophus.dir/depend

