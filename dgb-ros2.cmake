if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

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

  IF(Boost_VERSION GREATER 107299 OR Boost_VERSION_MACRO GREATER 107299)
    # Silence a warning about a deprecated use of boost bind by boost >= 1.73
    # without dropping support for boost < 1.73
    ADD_DEFINITIONS(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)
  ENDIF()
ENDIF(BUILD_PYTHON_INTERFACE)

ADD_PROJECT_DEPENDENCY(sot-core REQUIRED)

# Main Library
set(${PROJECT_NAME}_HEADERS
  include/${PROJECT_NAME}/ros2_init.hh
  include/${PROJECT_NAME}/sot_loader.hh
  include/${PROJECT_NAME}/sot_loader_basic.hh
  src/converter.hh
  src/sot_to_ros2.hh
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
MESSAGE(STATUS "TARGETS_EXPORT_NAME: ${TARGETS_EXPORT_NAME}" )

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

# Install package information
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
