# Copyright (C) 2008-2020 LAAS-CNRS, JRL AIST-CNRS.
#
# Author: Florent Lamiraux, Nirmal Giftsun
#

cmake_minimum_required(VERSION 3.1)

set(PROJECT_ORG stack-of-tasks)
set(PROJECT_NAME dynamic_graph_bridge)
set(PROJECT_DESCRIPTION "Dynamic graph bridge library")
set(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")
set(PROJECT_SUFFIX "-v3")

OPTION (BUILD_PYTHON_INTERFACE "Build the python binding" ON)

set(CUSTOM_HEADER_DIR dynamic_graph_bridge)
SET(DOXYGEN_USE_MATHJAX YES)
SET(CATKIN_ENABLE_TESTING OFF)
set(CXX_DISABLE_WERROR False)

include(cmake/base.cmake)
include(cmake/boost.cmake)
include(cmake/ros.cmake)
include(cmake/python.cmake)

CMAKE_POLICY(SET CMP0048 OLD)
project(${PROJECT_NAME} CXX)

SET(CATKIN_REQUIRED_COMPONENTS roscpp std_msgs message_generation std_srvs geometry_msgs sensor_msgs tf2_ros)
SET(CATKIN_DEPENDS_LIBRARIES ros_bridge sot_loader)

IF(BUILD_PYTHON_INTERFACE)
  FINDPYTHON()
  STRING(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})
  INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
  ADD_REQUIRED_DEPENDENCY("dynamic-graph-python >= 3.0.0")
  SET(CATKIN_REQUIRED_COMPONENTS ${CATKIN_REQUIRED_COMPONENTS} rospy)
  SET(CATKIN_DEPENDS_LIBRARIES ${CATKIN_DEPENDS_LIBRARIES} ros_interpreter)
ENDIF(BUILD_PYTHON_INTERFACE)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_REQUIRED_COMPONENTS})
find_package(realtime_tools)

set(${PROJECT_NAME}_HEADERS
  include/dynamic_graph_bridge/ros_init.hh
  include/dynamic_graph_bridge/sot_loader.hh
  include/dynamic_graph_bridge/sot_loader_basic.hh
  )

IF(BUILD_PYTHON_INTERFACE)
  set(${PROJECT_NAME}_HEADERS ${${PROJECT_NAME}_HEADERS}
    include/dynamic_graph_bridge/ros_interpreter.hh )
ENDIF(BUILD_PYTHON_INTERFACE)

SEARCH_FOR_BOOST()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_INSTALL_RPATH "${LIBRARY_OUTPUT_PATH}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${LIBRARY_OUTPUT_PATH}")

# Add dependency to SoT specific packages.
SET(SOT_PKGNAMES
dynamic_graph_bridge_msgs)

add_required_dependency(roscpp)
add_required_dependency(tf2_ros)
add_required_dependency("realtime_tools >= 1.8")

ADD_REQUIRED_DEPENDENCY("dynamic-graph >= 3.0.0")
ADD_REQUIRED_DEPENDENCY("sot-core >= 3.0.0")

add_required_dependency(dynamic_graph_bridge_msgs)

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
pkg_config_use_dependency(ros_bridge dynamic_graph_bridge_msgs)
install(TARGETS ros_bridge DESTINATION lib)

# Add ros_bridge in the dynamic-graph-bridge pkg-config file.


# Make sure rpath are preserved during the install as ROS dependencies
# are not installed.
set_target_properties(ros_bridge PROPERTIES BUILD_WITH_INSTALL_RPATH True
                      LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

macro(compile_plugin NAME)
  message(lib path ${LIBRARY_OUTPUT_PATH})
  file(MAKE_DIRECTORY "${LIBRARY_OUTPUT_PATH}/dynamic_graph/ros/${NAME}")
  add_library(${NAME} SHARED src/${NAME}.cpp src/${NAME}.hh)
  pkg_config_use_dependency(${NAME} dynamic-graph)
  pkg_config_use_dependency(${NAME} sot-core)
  pkg_config_use_dependency(${NAME} dynamic_graph_bridge_msgs)
  add_dependencies(${NAME} ros_bridge)
  target_link_libraries(${NAME} ros_bridge)
  set_target_properties(${NAME} PROPERTIES BUILD_WITH_INSTALL_RPATH True)
  set_target_properties(${NAME} PROPERTIES PREFIX "")
  install(TARGETS ${NAME} DESTINATION lib/plugin)
endmacro()

# Build Sot Entities
set(listplugins ros_publish ros_subscribe ros_queued_subscribe ros_tf_listener ros_time)

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

    PKG_CONFIG_USE_DEPENDENCY(ros/${NAME}/wrap realtime_tools)
    PKG_CONFIG_USE_DEPENDENCY(ros/${NAME}/wrap dynamic_graph)
    PKG_CONFIG_USE_DEPENDENCY(ros/${NAME}/wrap sot-core)
    PKG_CONFIG_USE_DEPENDENCY(ros/${NAME}/wrap dynamic_graph_bridge_msgs)
    PKG_CONFIG_USE_DEPENDENCY(ros/${NAME}/wrap tf2_ros)
  endforeach()

  # ros_interperter library.

  add_library(ros_interpreter src/ros_interpreter.cpp)
  pkg_config_use_dependency(ros_interpreter dynamic-graph)
  pkg_config_use_dependency(ros_interpreter sot-core)
  pkg_config_use_dependency(ros_interpreter roscpp)
  pkg_config_use_dependency(ros_interpreter dynamic_graph_bridge_msgs)
  pkg_config_use_dependency(ros_interpreter dynamic-graph-python)

  add_dependencies(ros_interpreter ros_bridge)
  target_link_libraries(ros_interpreter ros_bridge)
  set_target_properties(ros_interpreter PROPERTIES BUILD_WITH_INSTALL_RPATH True
                        LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
  message(cmakeinstalllibdir " is ${CMAKE_INSTALL_LIBDIR} ")
  install(TARGETS ros_interpreter
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION lib)

ENDIF(BUILD_PYTHON_INTERFACE)

# Stand alone embedded intepreter with a robot controller.
add_executable(geometric_simu src/geometric_simu.cpp src/sot_loader.cpp src/sot_loader_basic.cpp)
pkg_config_use_dependency(geometric_simu tf2_ros)
pkg_config_use_dependency(geometric_simu roscpp)
pkg_config_use_dependency(geometric_simu dynamic-graph)
target_link_libraries(geometric_simu  ros_bridge tf2_ros ${Boost_LIBRARIES} ${CMAKE_DL_LIBS})

# Sot loader library
add_library(sot_loader src/sot_loader.cpp src/sot_loader_basic.cpp)
pkg_config_use_dependency(sot_loader dynamic-graph)
pkg_config_use_dependency(sot_loader sot-core)
target_link_libraries(sot_loader ${Boost_LIBRARIES} roscpp ros_bridge tf2_ros)
install(TARGETS sot_loader
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)

add_subdirectory(src)
add_subdirectory(tests)

# This is necessary so that the pc file generated by catking is similar to the on
# done directly by jrl-cmake-modules
catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS message_runtime roscpp realtime_tools ${SOT_PKGNAMES} tf2_ros
  LIBRARIES ${CATKIN_DEPENDS_LIBRARIES}
)

# Add libraries in pc file generated by cmake submodule
PKG_CONFIG_APPEND_LIBS(ros_bridge sot_loader)

IF(BUILD_PYTHON_INTERFACE)
  PKG_CONFIG_APPEND_LIBS(ros_interpreter)

  #install ros executables
  install(PROGRAMS
    ${CMAKE_SOURCE_DIR}/scripts/robot_pose_publisher
    ${CMAKE_SOURCE_DIR}/scripts/run_command
    ${CMAKE_SOURCE_DIR}/scripts/tf_publisher
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

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
