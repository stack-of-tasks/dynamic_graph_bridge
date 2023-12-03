# Copyright (C) 2021 LAAS-CNRS.
#
# Author: Maxmilien Naveau
#

# Main Library
set(${PROJECT_NAME}_HEADERS
    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/ros_python_interpreter_client.hpp
    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/ros_python_interpreter_server.hpp
    ${PROJECT_SOURCE_DIR}/src/dg_ros_mapping.hpp
    ${PROJECT_SOURCE_DIR}/src/matrix_geometry.hpp
    ${PROJECT_SOURCE_DIR}/src/time_point_io.hpp)
set(${PROJECT_NAME}_SOURCES
    ${PROJECT_SOURCE_DIR}/src/dg_ros_mapping.cpp #
    ${PROJECT_SOURCE_DIR}/src/ros_python_interpreter_client.cpp #
    ${PROJECT_SOURCE_DIR}/src/ros_python_interpreter_server.cpp #
    ${PROJECT_SOURCE_DIR}/src/ros.cpp #
)
add_library(ros_bridge SHARED ${${PROJECT_NAME}_SOURCES}
                              ${${PROJECT_NAME}_HEADERS})
target_include_directories(
  ros_bridge
  PUBLIC $<INSTALL_INTERFACE:include>
  PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(
  ros_bridge
  sot-core::sot-core #
  pinocchio::pinocchio #
  dynamic-graph-python::dynamic-graph-python #
  dynamic_graph_bridge_msgs::dynamic_graph_bridge_msgs__rosidl_typesupport_cpp)
ament_target_dependencies(
  ros_bridge
  ament_index_cpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  tf2_ros
  rcutils
  urdfdom)

if(SUFFIX_SO_VERSION)
  set_target_properties(ros_bridge PROPERTIES SOVERSION ${PROJECT_VERSION})
endif(SUFFIX_SO_VERSION)
if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  install(
    TARGETS ros_bridge
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION lib)
endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)

# Dynamic graph ros plugin.
set(plugins ros_publish ros_subscribe)
foreach(plugin ${plugins})
  get_filename_component(plugin_library_name ${plugin} NAME)
  add_library(
    ${plugin_library_name} SHARED ${PROJECT_SOURCE_DIR}/src/${plugin}.cpp #
                                  ${PROJECT_SOURCE_DIR}/src/${plugin}.hpp #
  )
  if(SUFFIX_SO_VERSION)
    set_target_properties(${plugin_library_name} PROPERTIES SOVERSION
                                                            ${PROJECT_VERSION})
  endif(SUFFIX_SO_VERSION)
  target_link_libraries(
    ${plugin_library_name}
    ${${plugin_library_name}_deps}
    ros_bridge
    dynamic_graph_bridge_msgs::dynamic_graph_bridge_msgs__rosidl_typesupport_cpp
    rclcpp::rclcpp
    rcl_interfaces::rcl_interfaces__rosidl_typesupport_cpp
    std_srvs::std_srvs__rosidl_typesupport_cpp)
  target_include_directories(
    ${plugin_library_name}
    PUBLIC $<INSTALL_INTERFACE:include>
    PRIVATE ${PROJECT_SOURCE_DIR}/include)
  if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
    install(
      TARGETS ${plugin_library_name}
      EXPORT ${TARGETS_EXPORT_NAME}
      DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR})
  endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  if(BUILD_PYTHON_INTERFACE)
    string(REPLACE - _ PYTHON_LIBRARY_NAME ${plugin_library_name})
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python-module-py.cc")
      dynamic_graph_python_module(
        "ros/${PYTHON_LIBRARY_NAME}" ${plugin_library_name}
        ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap SOURCE_PYTHON_MODULE
        "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python-module-py.cc")
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python.hh")
      dynamic_graph_python_module(
        "ros/${PYTHON_LIBRARY_NAME}" ${plugin_library_name}
        ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap MODULE_HEADER
        "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python.hh")
    endif()
    target_include_directories(
      ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap
      PUBLIC $<INSTALL_INTERFACE:include>
      PRIVATE ${PROJECT_SOURCE_DIR}/include)
  endif(BUILD_PYTHON_INTERFACE)
endforeach(plugin)

if(BUILD_PYTHON_INTERFACE)
  python_install_on_site("dynamic_graph/ros" "__init__.py")
  python_install_on_site("dynamic_graph/ros" "ros.py")
  python_install_on_site("dynamic_graph/ros" "dgcompleter.py")
endif(BUILD_PYTHON_INTERFACE)

# Sot loader library
add_library(sot_loader SHARED sot_loader.cpp sot_loader_basic.cpp)
target_link_libraries(
  sot_loader Boost::program_options ${catkin_LIBRARIES} ros_bridge
  dynamic_graph_bridge_msgs::dynamic_graph_bridge_msgs__rosidl_typesupport_cpp)
target_include_directories(
  sot_loader
  PUBLIC $<INSTALL_INTERFACE:include>
  PRIVATE ${PROJECT_SOURCE_DIR}/include)
install(
  TARGETS sot_loader
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)

# Stand alone embedded intepreter with a robot controller.
add_executable(geometric_simu programs/geometric_simu.cpp)
target_link_libraries(
  geometric_simu
  Boost::program_options
  ${CMAKE_DL_LIBS}
  ${catkin_LIBRARIES}
  ros_bridge
  sot_loader
  dynamic_graph_bridge_msgs::dynamic_graph_bridge_msgs__rosidl_typesupport_cpp)
target_include_directories(
  geometric_simu
  PUBLIC $<INSTALL_INTERFACE:include>
  PRIVATE ${PROJECT_SOURCE_DIR}/include)
install(TARGETS geometric_simu DESTINATION lib/${PROJECT_NAME})
