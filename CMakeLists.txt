cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

include(FindPkgConfig)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

pkg_check_modules(JRL_MAL REQUIRED jrl-mal)
pkg_check_modules(DYNAMIC_GRAPH REQUIRED dynamic-graph)
pkg_check_modules(SOT_CORE REQUIRED sot-core)

rosbuild_add_library(ros_import
  src/sot_to_ros.cpp
  src/ros_import.cpp src/ros_import.hh src/converter.hh src/sot_to_ros.hh)
rosbuild_add_compile_flags(ros_import ${JRL_MAL_CFLAGS} ${DYNAMIC_GRAPH_CFLAGS}
  ${SOT_CORE_CFLAGS})
rosbuild_add_link_flags(ros_import ${JRL_MAL_LDFLAGS} ${DYNAMIC_GRAPH_LDFLAGS}
  ${SOT_CORE_LDFLAGS})
target_link_libraries(ros_import
  ${JRL_MAL_LIBRARIES} ${DYNAMIC_GRAPH_LIBRARIES} ${SOT_CORE_LIBRARIES})

rosbuild_add_library(ros_export
  src/sot_to_ros.cpp
  src/ros_export.cpp src/ros_export.hh src/converter.hh src/sot_to_ros.hh)
rosbuild_add_compile_flags(ros_export ${JRL_MAL_CFLAGS} ${DYNAMIC_GRAPH_CFLAGS}
  ${SOT_CORE_CFLAGS})
rosbuild_add_link_flags(ros_export ${JRL_MAL_LDFLAGS} ${DYNAMIC_GRAPH_LDFLAGS}
  ${SOT_CORE_LDFLAGS})
target_link_libraries(ros_export
  ${JRL_MAL_LIBRARIES} ${DYNAMIC_GRAPH_LIBRARIES} ${SOT_CORE_LIBRARIES})

rosbuild_add_library(ros_joint_state
  src/ros_joint_state.cpp src/ros_joint_state.hh src/converter.hh src/sot_to_ros.hh)
rosbuild_add_compile_flags(ros_joint_state ${JRL_MAL_CFLAGS} ${DYNAMIC_GRAPH_CFLAGS}
  ${SOT_CORE_CFLAGS})
rosbuild_add_link_flags(ros_joint_state ${JRL_MAL_LDFLAGS} ${DYNAMIC_GRAPH_LDFLAGS}
  ${SOT_CORE_LDFLAGS})
target_link_libraries(ros_joint_state
  ${JRL_MAL_LIBRARIES} ${DYNAMIC_GRAPH_LIBRARIES} ${SOT_CORE_LIBRARIES})



INSTALL(TARGETS ros_import DESTINATION lib)
INSTALL(TARGETS ros_export DESTINATION lib)
INSTALL(TARGETS ros_joint_state DESTINATION lib)

INCLUDE(cmake/python.cmake)

INCLUDE_DIRECTORIES(${DYNAMIC_GRAPH_INCLUDE_DIRS})
LINK_DIRECTORIES(${DYNAMIC_GRAPH_LIBRARY_DIRS})

DYNAMIC_GRAPH_PYTHON_MODULE("ros/ros_import"
  ros_import
  ros/ros_import/wrap
  )
SET_TARGET_PROPERTIES(dynamic_graph/ros/ros_import/wrap
  PROPERTIES
  COMPILE_FLAGS
  ${JRL_MAL_CFLAGS} ${DYNAMIC_GRAPH_CFLAGS} ${SOT_CORE_CFLAGS}
  LINK_FLAGS
  ${JRL_MAL_LDFLAGS} ${DYNAMIC_GRAPH_LDFLAGS} ${SOT_CORE_LDFLAGS}
  )

DYNAMIC_GRAPH_PYTHON_MODULE("ros/ros_export"
  ros_export
  ros/ros_export/wrap
  )
SET_TARGET_PROPERTIES(dynamic_graph/ros/ros_export/wrap
  PROPERTIES
  COMPILE_FLAGS
  ${JRL_MAL_CFLAGS} ${DYNAMIC_GRAPH_CFLAGS} ${SOT_CORE_CFLAGS}
  LINK_FLAGS
  ${JRL_MAL_LDFLAGS} ${DYNAMIC_GRAPH_LDFLAGS} ${SOT_CORE_LDFLAGS}
  )

DYNAMIC_GRAPH_PYTHON_MODULE("ros/ros_joint_state"
  ros_joint_state
  ros/ros_joint_state/wrap
  )
SET_TARGET_PROPERTIES(dynamic_graph/ros/ros_joint_state/wrap
  PROPERTIES
  COMPILE_FLAGS
  ${JRL_MAL_CFLAGS} ${DYNAMIC_GRAPH_CFLAGS} ${SOT_CORE_CFLAGS}
  LINK_FLAGS
  ${JRL_MAL_LDFLAGS} ${DYNAMIC_GRAPH_LDFLAGS} ${SOT_CORE_LDFLAGS}
  )

ADD_SUBDIRECTORY(src)
