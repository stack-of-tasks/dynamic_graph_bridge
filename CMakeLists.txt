# Copyright (C) 2008-2013 LAAS-CNRS, JRL AIST-CNRS.
#
# Author: Florent Lamiraux, Nirmal Giftsun
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# Catkin part

cmake_minimum_required(VERSION 2.4.6)

project(dynamic_graph_bridge)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation std_srvs geometry_msgs sensor_msgs tf realtime_tools)
find_package(Boost REQUIRED COMPONENTS program_options)

catkin_package(CATKIN_DEPENDS message_runtime)


## LAAS cmake submodule part
set(PROJECT_DESCRIPTION "Dynamic graph bridge library")
set(PROJECT_NAME dynamic_graph_bridge)
set(PROJECT_URL "")

set(CXX_DISABLE_WERROR False)
set(CUSTOM_HEADER_DIR dynamic_graph_bridge)
set(${PROJECT_NAME}_HEADERS
  include/dynamic_graph_bridge/ros_init.hh
  include/dynamic_graph_bridge/ros_interpreter.hh
  )
include(cmake/base.cmake)
include(cmake/ros.cmake)
include(cmake/GNUInstallDirs.cmake)
include(cmake/python.cmake)

SETUP_PROJECT()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_INSTALL_RPATH "${LIBRARY_OUTPUT_PATH}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${LIBRARY_OUTPUT_PATH}")
set(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )

add_required_dependency(roscpp)
add_required_dependency("realtime_tools >= 1.8")
add_required_dependency(bullet)
add_required_dependency(jrl-mal)
add_required_dependency(dynamic-graph)
add_required_dependency(dynamic-graph-python)
add_required_dependency(sot-core)
add_required_dependency(sot-dynamic)
add_required_dependency(jrl-dynamics-urdf)
ADD_ROSPACK_DEPENDENCY(dynamic_graph_bridge_msgs)

add_library(ros_bridge
  src/converter.hh
  include/dynamic_graph_bridge/ros_init.hh src/ros_init.cpp
  src/sot_to_ros.hh src/sot_to_ros.cpp
  )
pkg_config_use_dependency(ros_bridge jrl-mal)
pkg_config_use_dependency(ros_bridge bullet)
rospack_use_dependency(ros_bridge dynamic_graph_bridge_msgs)
install(TARGETS ros_bridge DESTINATION lib)

# Make sure rpath are preserved during the install as ROS dependencies
# are not installed.
set_target_properties(ros_bridge PROPERTIES BUILD_WITH_INSTALL_RPATH True
                      LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

macro(compile_plugin NAME)
  message(lib path ${LIBRARY_OUTPUT_PATH})
  file(MAKE_DIRECTORY "${LIBRARY_OUTPUT_PATH}/dynamic_graph/ros/${NAME}")
  add_library(${NAME} src/${NAME}.cpp src/${NAME}.hh)
  pkg_config_use_dependency(${NAME} jrl-mal)
  pkg_config_use_dependency(${NAME} dynamic-graph)
  pkg_config_use_dependency(${NAME} sot-core)
  pkg_config_use_dependency(${NAME} jrl-dynamics-urdf)
  rospack_use_dependency(${NAME} dynamic_graph_bridge_msgs)
  add_dependencies(${NAME} ros_bridge)
  target_link_libraries(${NAME} ros_bridge)
  set_target_properties(${NAME} PROPERTIES BUILD_WITH_INSTALL_RPATH True)
  install(TARGETS ${NAME} DESTINATION lib)
  

  dynamic_graph_python_module("ros/${NAME}"
    ${NAME}
    ros/${NAME}/wrap
    )

  PKG_CONFIG_USE_DEPENDENCY(ros/${NAME}/wrap realtime_tools)
  PKG_CONFIG_USE_DEPENDENCY(ros/${NAME}/wrap jrl-mal)
  PKG_CONFIG_USE_DEPENDENCY(ros/${NAME}/wrap dynamic_graph)
  PKG_CONFIG_USE_DEPENDENCY(ros/${NAME}/wrap sot-core)
  ROSPACK_USE_DEPENDENCY(ros/${NAME}/wrap dynamic_graph_bridge_msgs)
endmacro()

include(cmake/python.cmake)

compile_plugin(ros_publish)
compile_plugin(ros_subscribe)
compile_plugin(ros_time)
compile_plugin(ros_joint_state)

target_link_libraries(ros_joint_state "${DYNAMIC_GRAPH_PLUGINDIR}/dynamic.so")

compile_plugin(robot_model)

# ros_interperter library.
add_library(ros_interpreter src/ros_interpreter.cpp)
pkg_config_use_dependency(ros_interpreter jrl-mal)
pkg_config_use_dependency(ros_interpreter dynamic-graph)
pkg_config_use_dependency(ros_interpreter sot-core)
pkg_config_use_dependency(ros_interpreter roscpp)
rospack_use_dependency(ros_interpreter dynamic_graph_bridge_msgs)

add_dependencies(ros_interpreter ros_bridge)
target_link_libraries(ros_interpreter ros_bridge)
set_target_properties(ros_interpreter PROPERTIES BUILD_WITH_INSTALL_RPATH True
                      LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
message(cmakeinstalllibdir ${CMAKE_INSTALL_LIBDIR} )
install(TARGETS ros_interpreter DESTINATION ${CMAKE_INSTALL_LIBDIR})

# Stand alone remote dynamic-graph Python interpreter.
add_executable(interpreter src/interpreter.cpp)
add_dependencies(interpreter ros_interpreter)
target_link_libraries(interpreter ros_interpreter)
pkg_config_use_dependency(interpreter jrl-mal)
pkg_config_use_dependency(interpreter dynamic-graph)
pkg_config_use_dependency(interpreter sot-core)
pkg_config_use_dependency(interpreter sot-dynamic)
rospack_use_dependency(interpreter dynamic_graph_bridge_msgs)
# set_target_properties(interpreter PROPERTIES BUILD_WITH_INSTALL_RPATH True)
#install(TARGETS interpreter DESTINATION bin)

# Stand alone embedded intepreter with a robot controller.
add_executable(geometric_simu src/geometric_simu.cpp src/sot_loader.cpp)
pkg_config_use_dependency(geometric_simu roscpp)
target_link_libraries(geometric_simu  ${Boost_LIBRARIES} dl)

add_subdirectory(src)

#install ros executables
install(PROGRAMS
  ${CMAKE_SOURCE_DIR}/scripts/robot_pose_publisher
  ${CMAKE_SOURCE_DIR}/scripts/run_command
  ${CMAKE_SOURCE_DIR}/scripts/tf_publisher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
SETUP_PROJECT_FINALIZE()

