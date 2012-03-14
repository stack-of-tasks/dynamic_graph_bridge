cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

INCLUDE(cmake/base.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_boost_directories()

SET(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )

ADD_REQUIRED_DEPENDENCY(jrl-mal)
ADD_REQUIRED_DEPENDENCY(dynamic-graph)
ADD_REQUIRED_DEPENDENCY(dynamic-graph-python)
ADD_REQUIRED_DEPENDENCY(sot-core)
ADD_REQUIRED_DEPENDENCY(sot-dynamic)

include_directories(include)

file(MAKE_DIRECTORY "${LIBRARY_OUTPUT_PATH}/dynamic_graph/ros/ros_export")
file(MAKE_DIRECTORY "${LIBRARY_OUTPUT_PATH}/dynamic_graph/ros/ros_import")
file(MAKE_DIRECTORY
  "${LIBRARY_OUTPUT_PATH}/dynamic_graph/ros/ros_joint_state")

rosbuild_add_library(ros_bridge
  src/converter.hh
  include/dynamic_graph_bridge/ros_init.hh src/ros_init.cpp
  src/sot_to_ros.hh src/sot_to_ros.cpp
  )
# Make sure rpath are preserved during the install as ROS dependencies
# are not installed.
set_target_properties(ros_bridge PROPERTIES BUILD_WITH_INSTALL_RPATH True)

MACRO(COMPILE_PLUGIN NAME)
  rosbuild_add_library(${NAME} src/${NAME}.cpp src/${NAME}.hh)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} jrl-mal)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} dynamic-graph)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} sot-core)
  target_link_libraries(${NAME} ros_bridge)
  set_target_properties(${NAME} PROPERTIES BUILD_WITH_INSTALL_RPATH True)
  INSTALL(TARGETS ${NAME} DESTINATION lib)

  DYNAMIC_GRAPH_PYTHON_MODULE("ros/${NAME}"
    ${NAME}
    ros/${NAME}/wrap
    )

  SET_TARGET_PROPERTIES(ros/${NAME}/wrap
    PROPERTIES
    COMPILE_FLAGS
    ${JRL_MAL_CFLAGS} ${DYNAMIC_GRAPH_CFLAGS} ${SOT_CORE_CFLAGS}
    LINK_FLAGS
    ${JRL_MAL_LDFLAGS} ${DYNAMIC_GRAPH_LDFLAGS} ${SOT_CORE_LDFLAGS}
    )
ENDMACRO()

INCLUDE(cmake/python.cmake)

INCLUDE_DIRECTORIES(${DYNAMIC_GRAPH_INCLUDE_DIRS})
LINK_DIRECTORIES(${DYNAMIC_GRAPH_LIBRARY_DIRS})

COMPILE_PLUGIN(ros_import)
COMPILE_PLUGIN(ros_export)
COMPILE_PLUGIN(ros_export)

COMPILE_PLUGIN(ros_joint_state)
target_link_libraries(ros_joint_state "${DYNAMIC_GRAPH_PLUGINDIR}/dynamic.so")

# ros_interperter library.
rosbuild_add_library(ros_interpreter src/ros_interpreter.cpp)
PKG_CONFIG_USE_DEPENDENCY(ros_interpreter jrl-mal)
PKG_CONFIG_USE_DEPENDENCY(ros_interpreter dynamic-graph)
PKG_CONFIG_USE_DEPENDENCY(ros_interpreter sot-core)
target_link_libraries(ros_interpreter ros_bridge)
set_target_properties(ros_interpreter PROPERTIES BUILD_WITH_INSTALL_RPATH True)
INSTALL(TARGETS ros_interpreter DESTINATION lib)

# Stand alone remote dynamic-graph Python interpreter.
rosbuild_add_executable(interpreter src/interpreter.cpp)
target_link_libraries(interpreter ros_interpreter)
PKG_CONFIG_USE_DEPENDENCY(interpreter jrl-mal)
PKG_CONFIG_USE_DEPENDENCY(interpreter dynamic-graph)
PKG_CONFIG_USE_DEPENDENCY(interpreter sot-core)
PKG_CONFIG_USE_DEPENDENCY(interpreter sot-dynamic)
set_target_properties(interpreter PROPERTIES BUILD_WITH_INSTALL_RPATH True)
INSTALL(TARGETS interpreter DESTINATION bin)

ADD_SUBDIRECTORY(src)
