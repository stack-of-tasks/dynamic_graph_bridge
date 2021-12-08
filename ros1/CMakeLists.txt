# Copyright (C) 2008-2020 LAAS-CNRS, JRL AIST-CNRS.
#
# Author: Florent Lamiraux, Nirmal Giftsun, Guilhem Saurel, Maxmilien Naveau
#

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
TARGET_INCLUDE_DIRECTORIES(ros_bridge PUBLIC $<INSTALL_INTERFACE:include>
                                      PRIVATE ${PROJECT_SOURCE_DIR}/ros1/include)
TARGET_LINK_LIBRARIES(ros_bridge
  ${catkin_LIBRARIES}
  sot-core::sot-core
  pinocchio::pinocchio
  dynamic_graph_bridge_msgs::dynamic_graph_bridge_msgs
)
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

# we also need to install the header files
install(DIRECTORY include/ DESTINATION include)
