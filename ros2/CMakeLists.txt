# Copyright (C) 2021 LAAS-CNRS.
#
# Author: Maxmilien Naveau
#

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

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

add_project_dependency(Boost REQUIRED COMPONENTS program_options)
add_project_dependency(dynamic_graph_bridge_msgs 0.3.0 REQUIRED)
add_project_dependency(sot-core REQUIRED)

IF(BUILD_PYTHON_INTERFACE)
  findpython()
  search_for_boost_python()
  string(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})
  add_project_dependency(dynamic-graph-python 4.0.0 REQUIRED)

  IF(Boost_VERSION GREATER 107299 OR Boost_VERSION_MACRO GREATER 107299)
    # Silence a warning about a deprecated use of boost bind by boost >= 1.73
    # without dropping support for boost < 1.73
    add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)
  ENDIF()
ENDIF(BUILD_PYTHON_INTERFACE)

add_subdirectory(src)
# add_subdirectory(tests)

#install ros executables
install(PROGRAMS
  scripts/robot_pose_publisher
  scripts/run_command
  scripts/tf_publisher
  DESTINATION share/${PROJECT_NAME}
  )

# Install package information
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

#ROS 2 packaging
ament_package()
