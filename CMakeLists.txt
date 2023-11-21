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

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

check_minimal_cxx_standard(17 ENFORCE)

cmake_policy(SET CMP0057 NEW)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_core REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdfdom REQUIRED)

add_project_dependency(Boost REQUIRED COMPONENTS program_options)
add_project_dependency(dynamic_graph_bridge_msgs 0.3.0 REQUIRED)
add_project_dependency(sot-core REQUIRED)

if(BUILD_PYTHON_INTERFACE)
  search_for_boost_python()
  string(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})
  add_project_dependency(dynamic-graph-python 4.0.0 REQUIRED)

  if(Boost_VERSION GREATER 107299 OR Boost_VERSION_MACRO GREATER 107299)
    # Silence a warning about a deprecated use of boost bind by boost >= 1.73
    # without dropping support for boost < 1.73
    add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)
  endif()
endif(BUILD_PYTHON_INTERFACE)

add_subdirectory(src)
add_subdirectory(tests)

# install ros executables
install(
  PROGRAMS scripts/remote_python_client.py
  RENAME remote_python_client
  DESTINATION lib/${PROJECT_NAME})
install(
  PROGRAMS scripts/remote_python_client.py
  RENAME run_command
  DESTINATION lib/${PROJECT_NAME})

# we also need to install the header files
install(DIRECTORY include/ DESTINATION include)

# Install package information
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# ROS 2 packaging
ament_package()
