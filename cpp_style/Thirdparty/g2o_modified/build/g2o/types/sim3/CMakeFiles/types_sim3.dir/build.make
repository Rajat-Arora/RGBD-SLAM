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
include g2o/types/sim3/CMakeFiles/types_sim3.dir/depend.make

# Include the progress variables for this target.
include g2o/types/sim3/CMakeFiles/types_sim3.dir/progress.make

# Include the compile flags for this target's objects.
include g2o/types/sim3/CMakeFiles/types_sim3.dir/flags.make

g2o/types/sim3/CMakeFiles/types_sim3.dir/types_seven_dof_expmap.cpp.o: g2o/types/sim3/CMakeFiles/types_sim3.dir/flags.make
g2o/types/sim3/CMakeFiles/types_sim3.dir/types_seven_dof_expmap.cpp.o: ../g2o/types/sim3/types_seven_dof_expmap.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object g2o/types/sim3/CMakeFiles/types_sim3.dir/types_seven_dof_expmap.cpp.o"
	cd /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build/g2o/types/sim3 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/types_sim3.dir/types_seven_dof_expmap.cpp.o -c /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/g2o/types/sim3/types_seven_dof_expmap.cpp

g2o/types/sim3/CMakeFiles/types_sim3.dir/types_seven_dof_expmap.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/types_sim3.dir/types_seven_dof_expmap.cpp.i"
	cd /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build/g2o/types/sim3 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/g2o/types/sim3/types_seven_dof_expmap.cpp > CMakeFiles/types_sim3.dir/types_seven_dof_expmap.cpp.i

g2o/types/sim3/CMakeFiles/types_sim3.dir/types_seven_dof_expmap.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/types_sim3.dir/types_seven_dof_expmap.cpp.s"
	cd /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build/g2o/types/sim3 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/g2o/types/sim3/types_seven_dof_expmap.cpp -o CMakeFiles/types_sim3.dir/types_seven_dof_expmap.cpp.s

# Object files for target types_sim3
types_sim3_OBJECTS = \
"CMakeFiles/types_sim3.dir/types_seven_dof_expmap.cpp.o"

# External object files for target types_sim3
types_sim3_EXTERNAL_OBJECTS =

../lib/libg2o_types_sim3.so: g2o/types/sim3/CMakeFiles/types_sim3.dir/types_seven_dof_expmap.cpp.o
../lib/libg2o_types_sim3.so: g2o/types/sim3/CMakeFiles/types_sim3.dir/build.make
../lib/libg2o_types_sim3.so: ../lib/libg2o_types_sba.so
../lib/libg2o_types_sim3.so: ../lib/libg2o_types_slam3d.so
../lib/libg2o_types_sim3.so: ../lib/libg2o_core.so
../lib/libg2o_types_sim3.so: ../lib/libg2o_stuff.so
../lib/libg2o_types_sim3.so: ../lib/libg2o_opengl_helper.so
../lib/libg2o_types_sim3.so: /usr/lib/x86_64-linux-gnu/libGLU.so
../lib/libg2o_types_sim3.so: /usr/lib/x86_64-linux-gnu/libGL.so
../lib/libg2o_types_sim3.so: g2o/types/sim3/CMakeFiles/types_sim3.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library ../../../../lib/libg2o_types_sim3.so"
	cd /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build/g2o/types/sim3 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/types_sim3.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
g2o/types/sim3/CMakeFiles/types_sim3.dir/build: ../lib/libg2o_types_sim3.so

.PHONY : g2o/types/sim3/CMakeFiles/types_sim3.dir/build

g2o/types/sim3/CMakeFiles/types_sim3.dir/clean:
	cd /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build/g2o/types/sim3 && $(CMAKE_COMMAND) -P CMakeFiles/types_sim3.dir/cmake_clean.cmake
.PHONY : g2o/types/sim3/CMakeFiles/types_sim3.dir/clean

g2o/types/sim3/CMakeFiles/types_sim3.dir/depend:
	cd /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/g2o/types/sim3 /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build/g2o/types/sim3 /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build/g2o/types/sim3/CMakeFiles/types_sim3.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : g2o/types/sim3/CMakeFiles/types_sim3.dir/depend

