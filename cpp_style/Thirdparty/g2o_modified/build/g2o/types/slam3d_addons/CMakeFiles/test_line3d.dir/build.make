# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build

# Include any dependencies generated for this target.
include g2o/types/slam3d_addons/CMakeFiles/test_line3d.dir/depend.make

# Include the progress variables for this target.
include g2o/types/slam3d_addons/CMakeFiles/test_line3d.dir/progress.make

# Include the compile flags for this target's objects.
include g2o/types/slam3d_addons/CMakeFiles/test_line3d.dir/flags.make

g2o/types/slam3d_addons/CMakeFiles/test_line3d.dir/line3d_test.cpp.o: g2o/types/slam3d_addons/CMakeFiles/test_line3d.dir/flags.make
g2o/types/slam3d_addons/CMakeFiles/test_line3d.dir/line3d_test.cpp.o: ../g2o/types/slam3d_addons/line3d_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object g2o/types/slam3d_addons/CMakeFiles/test_line3d.dir/line3d_test.cpp.o"
	cd /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build/g2o/types/slam3d_addons && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_line3d.dir/line3d_test.cpp.o -c /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/g2o/types/slam3d_addons/line3d_test.cpp

g2o/types/slam3d_addons/CMakeFiles/test_line3d.dir/line3d_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_line3d.dir/line3d_test.cpp.i"
	cd /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build/g2o/types/slam3d_addons && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/g2o/types/slam3d_addons/line3d_test.cpp > CMakeFiles/test_line3d.dir/line3d_test.cpp.i

g2o/types/slam3d_addons/CMakeFiles/test_line3d.dir/line3d_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_line3d.dir/line3d_test.cpp.s"
	cd /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build/g2o/types/slam3d_addons && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/g2o/types/slam3d_addons/line3d_test.cpp -o CMakeFiles/test_line3d.dir/line3d_test.cpp.s

# Object files for target test_line3d
test_line3d_OBJECTS = \
"CMakeFiles/test_line3d.dir/line3d_test.cpp.o"

# External object files for target test_line3d
test_line3d_EXTERNAL_OBJECTS =

../bin/test_line3d: g2o/types/slam3d_addons/CMakeFiles/test_line3d.dir/line3d_test.cpp.o
../bin/test_line3d: g2o/types/slam3d_addons/CMakeFiles/test_line3d.dir/build.make
../bin/test_line3d: ../lib/libg2o_types_slam3d_addons.so
../bin/test_line3d: ../lib/libg2o_types_slam3d.so
../bin/test_line3d: ../lib/libg2o_opengl_helper.so
../bin/test_line3d: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/test_line3d: ../lib/libg2o_core.so
../bin/test_line3d: ../lib/libg2o_stuff.so
../bin/test_line3d: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/test_line3d: g2o/types/slam3d_addons/CMakeFiles/test_line3d.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../../bin/test_line3d"
	cd /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build/g2o/types/slam3d_addons && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_line3d.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
g2o/types/slam3d_addons/CMakeFiles/test_line3d.dir/build: ../bin/test_line3d

.PHONY : g2o/types/slam3d_addons/CMakeFiles/test_line3d.dir/build

g2o/types/slam3d_addons/CMakeFiles/test_line3d.dir/clean:
	cd /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build/g2o/types/slam3d_addons && $(CMAKE_COMMAND) -P CMakeFiles/test_line3d.dir/cmake_clean.cmake
.PHONY : g2o/types/slam3d_addons/CMakeFiles/test_line3d.dir/clean

g2o/types/slam3d_addons/CMakeFiles/test_line3d.dir/depend:
	cd /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/g2o/types/slam3d_addons /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build/g2o/types/slam3d_addons /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build/g2o/types/slam3d_addons/CMakeFiles/test_line3d.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : g2o/types/slam3d_addons/CMakeFiles/test_line3d.dir/depend

