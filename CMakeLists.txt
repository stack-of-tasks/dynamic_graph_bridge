# Copyright (C) 2008-2020 LAAS-CNRS, JRL AIST-CNRS.
#
# Author: Florent Lamiraux, Nirmal Giftsun, Guilhem Saurel
#

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

# Project properties
SET(PROJECT_ORG stack-of-tasks)
SET(PROJECT_NAME dynamic_graph_bridge)
SET(PROJECT_DESCRIPTION "Dynamic graph bridge library")
SET(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project options
OPTION(BUILD_PYTHON_INTERFACE "Build the python bindings" ON)

# Project configuration
SET(PROJECT_USE_CMAKE_EXPORT TRUE)
SET(CUSTOM_HEADER_DIR ${PROJECT_NAME})
set(CXX_DISABLE_WERROR FALSE)
SET(DOXYGEN_USE_MATHJAX YES)
SET(CMAKE_VERBOSE_MAKEFILE ON)
#SET(CATKIN_ENABLE_TESTING OFF)

# JRL-cmakemodule setup
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/python.cmake)
#INCLUDE(cmake/ros.cmake)

# Project definition
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

if(DEFINED ENV{ROS_VERSION} )
  # ROS-1
  if ($ENV{ROS_VERSION} EQUAL 1)
    include(dgb-ros1.cmake)
  endif()

  # IF ROS2
  if ($ENV{ROS_VERSION} EQUAL 2)
    include(dgb-ros2.cmake)
  endif()

endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Library for sot_external_interface
  add_library(impl_test_sot_external_interface SHARED
    tests/impl_test_sot_external_interface)
  
  target_link_libraries(impl_test_sot_external_interface PUBLIC sot-core::sot-core)
  
  ament_add_gmock(test_sot_loader_basic
    tests/sot_loader_basic_test.cpp
  )
  target_include_directories(test_sot_loader_basic PUBLIC include)
  target_link_libraries(test_sot_loader_basic sot_loader)

endif()
