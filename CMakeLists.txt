cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

include(cmake/base.cmake)
include(cmake/GNUInstallDirs.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_INSTALL_RPATH "${LIBRARY_OUTPUT_PATH}")

rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_boost_directories()

set(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )

add_required_dependency(jrl-mal)
add_required_dependency(dynamic-graph)
add_required_dependency(dynamic-graph-python)
add_required_dependency(sot-core)
add_required_dependency(sot-dynamic)

include_directories(include)

rosbuild_add_library(ros_bridge
  src/converter.hh
  include/dynamic_graph_bridge/ros_init.hh src/ros_init.cpp
  src/sot_to_ros.hh src/sot_to_ros.cpp
  )
# Make sure rpath are preserved during the install as ROS dependencies
# are not installed.
set_target_properties(ros_bridge PROPERTIES BUILD_WITH_INSTALL_RPATH True)

macro(compile_plugin NAME)
  file(MAKE_DIRECTORY "${LIBRARY_OUTPUT_PATH}/dynamic_graph/ros/${NAME}")
  rosbuild_add_library(${NAME} src/${NAME}.cpp src/${NAME}.hh)
  pkg_config_use_dependency(${NAME} jrl-mal)
  pkg_config_use_dependency(${NAME} dynamic-graph)
  pkg_config_use_dependency(${NAME} sot-core)
  add_dependencies(${NAME} ros_bridge)
  target_link_libraries(${NAME} ros_bridge)
  set_target_properties(${NAME} PROPERTIES BUILD_WITH_INSTALL_RPATH True)
  install(TARGETS ${NAME} DESTINATION lib)

  dynamic_graph_python_module("ros/${NAME}"
    ${NAME}
    ros/${NAME}/wrap
    )

  PKG_CONFIG_USE_DEPENDENCY(ros/${NAME}/wrap jrl-mal)
  PKG_CONFIG_USE_DEPENDENCY(ros/${NAME}/wrap dynamic_graph)
  PKG_CONFIG_USE_DEPENDENCY(ros/${NAME}/wrap sot-core)
endmacro()

include(cmake/python.cmake)

include_directories(${DYNAMIC_GRAPH_include_DIRS})
link_directories(${DYNAMIC_GRAPH_LIBRARY_DIRS})
MESSAGE("dyn graph include dirs:   ${DYNAMIC_GRAPH_include_DIRS}")

compile_plugin(ros_publish)
compile_plugin(ros_subscribe)
compile_plugin(ros_time)

compile_plugin(ros_joint_state)
target_link_libraries(ros_joint_state "${DYNAMIC_GRAPH_PLUGINDIR}/dynamic.so")

compile_plugin(robot_model)

# ros_interperter library.
rosbuild_add_library(ros_interpreter src/ros_interpreter.cpp)
pkg_config_use_dependency(ros_interpreter jrl-mal)
pkg_config_use_dependency(ros_interpreter dynamic-graph)
pkg_config_use_dependency(ros_interpreter sot-core)
add_dependencies(ros_interpreter ros_bridge)
target_link_libraries(ros_interpreter ros_bridge)
set_target_properties(ros_interpreter PROPERTIES BUILD_WITH_INSTALL_RPATH True)
install(TARGETS ros_interpreter DESTINATION ${CMAKE_INSTALL_LIBDIR})

# Stand alone remote dynamic-graph Python interpreter.
rosbuild_add_executable(interpreter src/interpreter.cpp)
add_dependencies(interpreter ros_interpreter)
target_link_libraries(interpreter ros_interpreter)
pkg_config_use_dependency(interpreter jrl-mal)
pkg_config_use_dependency(interpreter dynamic-graph)
pkg_config_use_dependency(interpreter sot-core)
pkg_config_use_dependency(interpreter sot-dynamic)
# set_target_properties(interpreter PROPERTIES BUILD_WITH_INSTALL_RPATH True)
#install(TARGETS interpreter DESTINATION bin)

# Stand alone embedded intepreter with a robot controller.
rosbuild_add_executable(geometric_simu src/geometric_simu.cpp src/sot_loader.cpp)
rosbuild_link_boost(geometric_simu program_options)
target_link_libraries(geometric_simu  ${Boost_LIBRARIES} dl)
add_dependencies(geometric_simu ros_interpreter)
target_link_libraries(geometric_simu ros_interpreter)

# sot_loader library
rosbuild_add_library(sot_loader src/sot_loader.cpp)
install(TARGETS sot_loader DESTINATION ${CMAKE_INSTALL_LIBDIR}) 



add_subdirectory(src)

# Print warning.
MESSAGE(STATUS "=====================================================")
MESSAGE(STATUS "WARNING: this package has an 'install' target.")
MESSAGE(STATUS "Always run 'make install' in the package directory to")
MESSAGE(STATUS "properly install the Python modules and entities.")
MESSAGE(STATUS "=====================================================")
