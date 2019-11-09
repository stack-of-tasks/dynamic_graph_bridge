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

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

SET(PROJECT_ORG stack-of-tasks)
set(PROJECT_DESCRIPTION "Dynamic graph bridge library")
set(PROJECT_NAME dynamic_graph_bridge)
set(PROJECT_URL "https://github.com/stack-of-tasks/dynamic_graph_bridge")
set(PROJECT_SUFFIX "-v3")

# Export CMake Target
SET(PROJECT_USE_CMAKE_EXPORT TRUE)

# Disable failing compilation when a compilation error appears
set(CXX_DISABLE_WERROR False)

# Make sure that every header is generated in dynamic-graph
SET(CUSTOM_HEADER_DIR ${PROJECT_NAME})

include(cmake/base.cmake)

# Specify the project.
cmake_policy(SET CMP0048 NEW)
PROJECT(${PROJECT_NAME}
  LANGUAGES
  CXX
  VERSION
  ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}
  )


INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/eigen.cmake)
include(cmake/ros.cmake)
include(cmake/GNUInstallDirs.cmake)
include(cmake/python.cmake)
include(cmake/test.cmake)

SET(CATKIN_REQUIRED_COMPONENTS
  roscpp std_msgs message_generation std_srvs geometry_msgs sensor_msgs
  tf tf2_bullet)
SET(CATKIN_DEPENDS_LIBRARIES ros_bridge sot_loader)

## LAAS cmake submodule part

SET(DOXYGEN_USE_MATHJAX YES)

# Add option build python interface
OPTION (BUILD_PYTHON_INTERFACE "Build the python binding" ON)

set(CUSTOM_HEADER_DIR dynamic_graph_bridge)
set(${PROJECT_NAME}_HEADERS
  include/dynamic_graph_bridge/ros_init.hh
  include/dynamic_graph_bridge/sot_loader.hh
  include/dynamic_graph_bridge/sot_loader_basic.hh
  )

IF(BUILD_PYTHON_INTERFACE)
  set(${PROJECT_NAME}_HEADERS ${${PROJECT_NAME}_HEADERS}
    include/dynamic_graph_bridge/ros_interpreter.hh )
ENDIF(BUILD_PYTHON_INTERFACE)

IF(BUILD_PYTHON_INTERFACE)
  FINDPYTHON()
  STRING(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})
  
  ADD_PROJECT_DEPENDENCY(dynamic-graph-python REQUIRED )
  SET(CATKIN_REQUIRED_COMPONENTS ${CATKIN_REQUIRED_COMPONENTS} rospy)
  SET(CATKIN_DEPENDS_LIBRARIES ${CATKIN_DEPENDS_LIBRARIES} ros_interpreter)
ENDIF(BUILD_PYTHON_INTERFACE)

SET(CATKIN_ENABLE_TESTING OFF)

MESSAGE(STATUS "CATKIN_REQUIRED_COMPONENTS: ${CATKIN_REQUIRED_COMPONENTS}")
# Add catkin components
find_package(catkin REQUIRED
  COMPONENTS ${CATKIN_REQUIRED_COMPONENTS})
find_package(realtime_tools)


SEARCH_FOR_EIGEN()
SEARCH_FOR_BOOST()

SETUP_PROJECT_PACKAGE_FINALIZE()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_INSTALL_RPATH "${LIBRARY_OUTPUT_PATH}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${LIBRARY_OUTPUT_PATH}")
set(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )

# Add dependency to SoT specific packages.
SET(SOT_PKGNAMES
dynamic_graph_bridge_msgs)

#add_project_dependency(realtime_tools 1.8 REQUIRED)


ADD_PROJECT_DEPENDENCY(dynamic-graph  3.0.0 REQUIRED)
ADD_PROJECT_DEPENDENCY(sot-core REQUIRED)

ADD_PROJECT_dependency(dynamic_graph_bridge_msgs)

foreach(sot_pkgname ${SOT_PKGNAMES})
  add_required_dependency(${sot_pkgname})
  pkg_check_modules(SOT_${sot_pkgname} REQUIRED ${sot_pkgname})
endforeach(sot_pkgname)


# Build ros_bridge library
add_library(ros_bridge
  src/converter.hh
  include/dynamic_graph_bridge/ros_init.hh src/ros_init.cpp
  src/sot_to_ros.hh src/sot_to_ros.cpp
  )

MESSAGE(STATUS "catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")

include_directories(${catkin_INCLUDE_DIRS})

target_include_directories(ros_bridge
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_BUILD_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${PYTHON_INCLUDE_DIRS}>
  INTERFACE ${EIGEN3_INCLUDE_DIR}
  $<INSTALL_INTERFACE:include>
  )

#target_link_libraries(ros_bridge tf2_bullet)
target_link_libraries(ros_bridge
  dynamic-graph::dynamic-graph)
target_link_libraries(ros_bridge
  dynamic-graph::dynamic-graph)

MESSAGE(STATUS "catkin_LIBRARIES: ${catkin_LIBRARIES}")
target_link_libraries(ros_bridge
  ${catkin_LIBRARIES} )

install(TARGETS ros_bridge
  EXPORT ${TARGETS_EXPORT_NAME}  
  DESTINATION lib)

# Add ros_bridge in the dynamic-graph-bridge pkg-config file.
# Make sure rpath are preserved during the install as ROS dependencies
# are not installed.
set_target_properties(ros_bridge PROPERTIES BUILD_WITH_INSTALL_RPATH True
                      LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

macro(compile_plugin NAME)
  message(lib path ${LIBRARY_OUTPUT_PATH})
  file(MAKE_DIRECTORY "${LIBRARY_OUTPUT_PATH}/dynamic_graph/ros/${NAME}")
  add_library(${NAME} SHARED src/${NAME}.cpp src/${NAME}.hh)

  # Headers
  target_include_directories(${NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_BUILD_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${PYTHON_INCLUDE_DIRS}>
    INTERFACE ${EIGEN3_INCLUDE_DIR}
    $<INSTALL_INTERFACE:include>
    )

  # Libraries
  target_link_libraries(${NAME} dynamic-graph::dynamic-graph)
  target_link_libraries(${NAME} sot-core::sot-core)
  target_link_libraries(${NAME} ${CATKIN_DEPENDS_LIBRARIES})
  add_dependencies(${NAME} ros_bridge)
  target_link_libraries(${NAME} ros_bridge)
  set_target_properties(${NAME} PROPERTIES BUILD_WITH_INSTALL_RPATH True)
  set_target_properties(${NAME} PROPERTIES PREFIX "")
  install(TARGETS ${NAME} DESTINATION lib/plugin)
endmacro()

# Build Sot Entities
set(listplugins
  ros_publish ros_subscribe ros_queued_subscribe ros_tf_listener ros_time)

foreach(aplugin ${listplugins})
  compile_plugin(${aplugin})
endforeach()

target_link_libraries(ros_publish ros_bridge)

IF(BUILD_PYTHON_INTERFACE)
  foreach(NAME ${listplugins})
    dynamic_graph_python_module("ros/${NAME}"
      ${NAME}
      ros/${NAME}/wrap
      )

    #PKG_CONFIG_USE_DEPENDENCY(ros/${NAME}/wrap realtime_tools)
    #PKG_CONFIG_USE_DEPENDENCY(ros/${NAME}/wrap dynamic_graph)
    #PKG_CONFIG_USE_DEPENDENCY(ros/${NAME}/wrap sot-core)
    PKG_CONFIG_USE_DEPENDENCY(ros/${NAME}/wrap dynamic_graph_bridge_msgs)
  endforeach()

  # ros_interperter library.

  add_library(ros_interpreter src/ros_interpreter.cpp)
  #pkg_config_use_dependency(ros_interpreter dynamic-graph)
  #pkg_config_use_dependency(ros_interpreter sot-core)
  #pkg_config_use_dependency(ros_interpreter roscpp)
  pkg_config_use_dependency(ros_interpreter dynamic_graph_bridge_msgs)
  #pkg_config_use_dependency(ros_interpreter dynamic-graph-python)

  add_dependencies(ros_interpreter ros_bridge)
  target_link_libraries(ros_interpreter roscpp)
  target_link_libraries(ros_interpreter
    dynamic-graph::dynamic-graph)
  target_link_libraries(ros_interpreter sot-core::sot-core)
  target_link_libraries(ros_interpreter
    dynamic-graph-python::dynamic-graph-python)  
  target_link_libraries(ros_interpreter realtime_tools)
  target_link_libraries(ros_interpreter ros_bridge)
  set_target_properties(ros_interpreter PROPERTIES BUILD_WITH_INSTALL_RPATH True
                        LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
  message(cmakeinstalllibdir " is ${CMAKE_INSTALL_LIBDIR} ")
  install(TARGETS ros_interpreter DESTINATION lib)

ENDIF(BUILD_PYTHON_INTERFACE)

MESSAGE(STATUS "CATKIN_DEPENDS_LIBRARIES: ${CATKIN_DEPENDS_LIBRARIES}")

# Stand alone embedded intepreter with a robot controller.
add_executable(geometric_simu src/geometric_simu.cpp
  src/sot_loader.cpp src/sot_loader_basic.cpp)
target_link_libraries(geometric_simu dynamic-graph::dynamic-graph)
target_link_libraries(geometric_simu ros_bridge tf ${Boost_LIBRARIES}
  ${CMAKE_DL_LIBS})
target_link_libraries(geometric_simu ${CATKIN_DEPENDS_LIBRARIES})

# Sot loader library
add_library(sot_loader src/sot_loader.cpp src/sot_loader_basic.cpp)
target_link_libraries(sot_loader dynamic-graph::dynamic-graph)
target_link_libraries(sot_loader sot-core::sot-core)
target_link_libraries(sot_loader ${Boost_LIBRARIES} )
install(TARGETS sot_loader DESTINATION lib)

add_subdirectory(src)
add_subdirectory(tests)

# Deal with the ROS part.
add_service_files( FILES RunPythonFile.srv )
generate_messages( DEPENDENCIES std_msgs )

# This is necessary so that the pc file generated by catking is similar to
# the on done directly by jrl-cmake-modules
catkin_package(INCLUDE_DIRS include
  CATKIN_DEPENDS
  message_runtime roscpp realtime_tools tf2_bullet ${SOT_PKGNAMES} tf
  LIBRARIES ${CATKIN_DEPENDS_LIBRARIES}
)

# Add libraries in pc file generated by cmake submodule
PKG_CONFIG_APPEND_LIBS(ros_bridge sot_loader)

# In the python interface needs to be build.
IF(BUILD_PYTHON_INTERFACE)
  PKG_CONFIG_APPEND_LIBS(ros_interpreter)

  #install ros executables
  install(PROGRAMS
    ${CMAKE_SOURCE_DIR}/scripts/robot_pose_publisher
    ${CMAKE_SOURCE_DIR}/scripts/run_command
    ${CMAKE_SOURCE_DIR}/scripts/tf_publisher
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

  # Service file.
  install(FILES ./srv/RunPythonFile.srv
    DESTINATION
    ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/srv)

ENDIF(BUILD_PYTHON_INTERFACE)

message(cmake_install_bindir " is ${CMAKE_INSTALL_BINDIR} ")

# Install the geometrical simulation node
install(TARGETS geometric_simu
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install package information
install(FILES manifest.xml
  DESTINATION
  ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/)


SETUP_PROJECT_PACKAGE_FINALIZE()
