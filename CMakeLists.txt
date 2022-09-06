# Copyright (C) 2008-2020 LAAS-CNRS, JRL AIST-CNRS.
#
# Author: Florent Lamiraux, Nirmal Giftsun, Guilhem Saurel
#

cmake_minimum_required(VERSION 3.1)

# Project properties
set(PROJECT_ORG stack-of-tasks)
set(PROJECT_NAME dynamic_graph_bridge)
set(PROJECT_DESCRIPTION "Dynamic graph bridge library")
set(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project options
option(BUILD_PYTHON_INTERFACE "Build the python bindings" ON)

# Project configuration
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CUSTOM_HEADER_DIR ${PROJECT_NAME})
set(CXX_DISABLE_WERROR FALSE)
set(DOXYGEN_USE_MATHJAX YES)
set(CATKIN_ENABLE_TESTING OFF)

# JRL-cmakemodule setup
include(cmake/base.cmake)
include(cmake/boost.cmake)
include(cmake/ros.cmake)

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

# Project dependencies
set(CATKIN_REQUIRED_COMPONENTS
    roscpp
    std_msgs
    message_generation
    std_srvs
    geometry_msgs
    sensor_msgs
    tf2_ros
    realtime_tools)

if(BUILD_PYTHON_INTERFACE)
  add_project_dependency(dynamic-graph-python 4.0.0 REQUIRED)
  string(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})
  set(CATKIN_REQUIRED_COMPONENTS ${CATKIN_REQUIRED_COMPONENTS} rospy)
endif(BUILD_PYTHON_INTERFACE)

add_project_dependency(Boost REQUIRED COMPONENTS program_options)
add_project_dependency(dynamic_graph_bridge_msgs 0.3.0 REQUIRED)
add_project_dependency(sot-core REQUIRED)

if(Boost_VERSION GREATER 107299 OR Boost_VERSION_MACRO GREATER 107299)
  # Silence a warning about a deprecated use of boost bind by boost >= 1.73
  # without dropping support for boost < 1.73
  add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)
endif()

find_package(catkin REQUIRED COMPONENTS ${CATKIN_REQUIRED_COMPONENTS})

# Main Library
set(${PROJECT_NAME}_HEADERS
    include/${PROJECT_NAME}/fwd.hh
    include/${PROJECT_NAME}/ros_init.hh
    include/${PROJECT_NAME}/sot_loader.hh
    include/${PROJECT_NAME}/sot_loader_basic.hh
    include/${PROJECT_NAME}/ros_interpreter.hh
    src/converter.hh
    src/sot_to_ros.hh)

set(${PROJECT_NAME}_SOURCES src/ros_init.cpp src/sot_to_ros.cpp
                            src/ros_parameter.cpp)

add_library(ros_bridge SHARED ${${PROJECT_NAME}_SOURCES}
                              ${${PROJECT_NAME}_HEADERS})
target_include_directories(ros_bridge SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_include_directories(ros_bridge PUBLIC $<INSTALL_INTERFACE:include>)
target_link_libraries(
  ros_bridge ${catkin_LIBRARIES} sot-core::sot-core pinocchio::pinocchio
  dynamic_graph_bridge_msgs::dynamic_graph_bridge_msgs)

if(SUFFIX_SO_VERSION)
  set_target_properties(ros_bridge PROPERTIES SOVERSION ${PROJECT_VERSION})
endif(SUFFIX_SO_VERSION)

if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  install(
    TARGETS ros_bridge
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION lib)
endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)

add_subdirectory(src)
add_subdirectory(tests)

# install ros executables
install(PROGRAMS scripts/robot_pose_publisher scripts/run_command
                 scripts/tf_publisher DESTINATION share/${PROJECT_NAME})

# Install package information
install(FILES manifest.xml package.xml DESTINATION share/${PROJECT_NAME})
