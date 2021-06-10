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

# Project dependencies
SET(CATKIN_REQUIRED_COMPONENTS roscpp std_msgs message_generation std_srvs geometry_msgs sensor_msgs tf2_ros
    realtime_tools)
ADD_PROJECT_DEPENDENCY(Boost REQUIRED COMPONENTS program_options)
ADD_PROJECT_DEPENDENCY(dynamic_graph_bridge_msgs 0.3.0 REQUIRED)

IF(BUILD_PYTHON_INTERFACE)
  FINDPYTHON()
  SEARCH_FOR_BOOST_PYTHON()
  STRING(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})
  ADD_PROJECT_DEPENDENCY(dynamic-graph-python 4.0.0 REQUIRED)
  SET(CATKIN_REQUIRED_COMPONENTS ${CATKIN_REQUIRED_COMPONENTS} rospy)

  IF(Boost_VERSION GREATER 107299 OR Boost_VERSION_MACRO GREATER 107299)
    # Silence a warning about a deprecated use of boost bind by boost >= 1.73
    # without dropping support for boost < 1.73
    ADD_DEFINITIONS(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)
  ENDIF()
ENDIF(BUILD_PYTHON_INTERFACE)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_REQUIRED_COMPONENTS})

ADD_PROJECT_DEPENDENCY(sot-core REQUIRED)

# Main Library
set(${PROJECT_NAME}_HEADERS
  include/${PROJECT_NAME}/fwd.hh
  include/${PROJECT_NAME}/ros_init.hh
  include/${PROJECT_NAME}/sot_loader.hh
  include/${PROJECT_NAME}/sot_loader_basic.hh
  include/${PROJECT_NAME}/ros_interpreter.hh
  src/converter.hh
  src/sot_to_ros.hh
  )

SET(${PROJECT_NAME}_SOURCES
  src/ros_init.cpp
  src/sot_to_ros.cpp
  src/ros_parameter.cpp
  )

ADD_LIBRARY(ros_bridge SHARED
  ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
TARGET_INCLUDE_DIRECTORIES(ros_bridge SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
TARGET_INCLUDE_DIRECTORIES(ros_bridge PUBLIC $<INSTALL_INTERFACE:include>)
TARGET_LINK_LIBRARIES(ros_bridge ${catkin_LIBRARIES}
  sot-core::sot-core pinocchio::pinocchio)

IF(SUFFIX_SO_VERSION)
  SET_TARGET_PROPERTIES(ros_bridge PROPERTIES SOVERSION ${PROJECT_VERSION})
ENDIF(SUFFIX_SO_VERSION)

IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  INSTALL(TARGETS ros_bridge EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)

add_subdirectory(src)
add_subdirectory(tests)

#install ros executables
install(PROGRAMS
  scripts/robot_pose_publisher
  scripts/run_command
  scripts/tf_publisher
  DESTINATION share/${PROJECT_NAME}
  )

# Install package information
install(FILES manifest.xml package.xml DESTINATION share/${PROJECT_NAME})
