# Install script for directory: /home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/g2o/types

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build/g2o/types/data/cmake_install.cmake")
  include("/home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build/g2o/types/slam2d/cmake_install.cmake")
  include("/home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build/g2o/types/slam3d/cmake_install.cmake")
  include("/home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build/g2o/types/sba/cmake_install.cmake")
  include("/home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build/g2o/types/sim3/cmake_install.cmake")
  include("/home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build/g2o/types/icp/cmake_install.cmake")
  include("/home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build/g2o/types/sclam2d/cmake_install.cmake")
  include("/home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build/g2o/types/slam2d_addons/cmake_install.cmake")
  include("/home/rajat/robotics/RGBD-SLAM/cpp_style/Thirdparty/g2o_modified/build/g2o/types/slam3d_addons/cmake_install.cmake")

endif()

