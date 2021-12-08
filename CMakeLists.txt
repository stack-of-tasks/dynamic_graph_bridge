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
SET(CATKIN_ENABLE_TESTING OFF)

# JRL-cmakemodule setup
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/ros.cmake)

# Project definition
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

if(DEFINED ENV{ROS_VERSION} )
  if ($ENV{ROS_VERSION} EQUAL 2) # if ROS2
    add_subdirectory(ros2)
    #ROS 2 packaging
    ament_package()
  else()
    add_subdirectory(ros1)
  endif()
else()
  message(FATAL_ERROR "No ROS version found.")
endif()

# Install package information
install(FILES manifest.xml package.xml DESTINATION share/${PROJECT_NAME})