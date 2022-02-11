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
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
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

# Project dependencies
IF ($ENV{ROS_VERSION}=="1")
  MESSAGE(FATAL_ERROR "This version is not supporting ROS-1")
ENDIF()

cmake_policy(SET CMP0057 NEW)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

ADD_PROJECT_DEPENDENCY(Boost REQUIRED COMPONENTS program_options)
ADD_PROJECT_DEPENDENCY(dynamic_graph_bridge_msgs 0.3.0 REQUIRED)

IF(BUILD_PYTHON_INTERFACE)
  FINDPYTHON()
  SEARCH_FOR_BOOST_PYTHON()
  STRING(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})
  ADD_PROJECT_DEPENDENCY(dynamic-graph-python 4.0.0 REQUIRED)
  IF ($ENV{ROS_VERSION}=="1")
    SET(CATKIN_REQUIRED_COMPONENTS ${CATKIN_REQUIRED_COMPONENTS} rospy)
  ENDIF()

  IF(Boost_VERSION GREATER 107299 OR Boost_VERSION_MACRO GREATER 107299)
    # Silence a warning about a deprecated use of boost bind by boost >= 1.73
    # without dropping support for boost < 1.73
    ADD_DEFINITIONS(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)
  ENDIF()
ENDIF(BUILD_PYTHON_INTERFACE)


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
  src/ros2_init.cpp
  src/sot_to_ros2.cpp
  src/ros2_parameter.cpp
  )

ADD_LIBRARY(ros_bridge SHARED
  ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})

TARGET_INCLUDE_DIRECTORIES(ros_bridge PUBLIC $<INSTALL_INTERFACE:include>)
TARGET_LINK_LIBRARIES(ros_bridge sot-core::sot-core pinocchio::pinocchio
  dynamic-graph-python::dynamic-graph-python
  dynamic_graph_bridge_msgs::dynamic_graph_bridge_msgs__rosidl_typesupport_cpp)

find_package(ament_cmake_core REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)  
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(realtime_tools REQUIRED)

ament_target_dependencies(ros_bridge
  ament_index_cpp
  std_msgs std_srvs geometry_msgs sensor_msgs tf2_ros rcutils)

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
  DESTINATION share/${PROJECT_NAME})

# Install package information
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  _ament_cmake_gmock_find_gmock()
  find_package(launch_testing_ament_cmake)
  ament_lint_auto_find_test_dependencies()

  # Library for sot_external_interface
  add_library(impl_test_sot_external_interface SHARED
    tests/impl_test_sot_external_interface)

  target_link_libraries(impl_test_sot_external_interface PUBLIC sot-core::sot-core)


  # Executable for SotLoaderBasic test
  add_executable(test_sot_loader_basic
    tests/test_sot_loader_basic.cpp
  )
  target_include_directories(test_sot_loader_basic PUBLIC include  "${GMOCK_INCLUDE_DIRS}")
  target_link_libraries(test_sot_loader_basic sot_loader "${GMOCK_LIBRARIES}" )


  add_launch_test(tests/launching_test_sot_loader_basic.py)

  # # Test for class SotLoader
  add_executable(test_sot_loader
     tests/test_sot_loader.cpp
   )
  target_include_directories(test_sot_loader PUBLIC include "${GMOCK_INCLUDE_DIRS}")
  target_link_libraries(test_sot_loader sot_loader "${GMOCK_LIBRARIES}")

  add_launch_test(tests/launching_test_sot_loader.py)

  # Install tests
  install(TARGETS test_sot_loader_basic
    test_sot_loader
    DESTINATION lib/${PROJECT_NAME}
    )

  # Install library for tests
  install(TARGETS impl_test_sot_external_interface
    DESTINATION lib
    )

  install(DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME}
    )

endif()

#ROS 2 packaging
find_package(ament_cmake REQUIRED)
ament_package()
