#.rst:
# .. command:: CUSTOM_DYNAMIC_GRAPH_PYTHON_MODULE ( SUBMODULENAME LIBRARYNAME TARGETNAME INSTALL_INIT_PY=1 SOURCE_PYTHON_MODULE=cmake/dynamic_graph/python-module-py.cc)
#
#   Add a python submodule to dynamic_graph
#
#   :param SUBMODULENAME: the name of the submodule (can be foo/bar),
#
#   :param LIBRARYNAME:   library to link the submodule with.
#
#   :param TARGETNAME:     name of the target: should be different for several
#                   calls to the macro.
#
#   :param INSTALL_INIT_PY: if set to 1 install and generated a __init__.py file.
#                   Set to 1 by default.
#
#   :param SOURCE_PYTHON_MODULE: Location of the cpp file for the python module in the package.
#                   Set to cmake/dynamic_graph/python-module-py.cc by default.
#
#  .. note::
#    Before calling this macro, set variable NEW_ENTITY_CLASS as
#    the list of new Entity types that you want to be bound.
#    Entity class name should match the name referencing the type
#    in the factory.
#
MACRO(CUSTOM_DYNAMIC_GRAPH_PYTHON_MODULE SUBMODULENAME LIBRARYNAME TARGETNAME)

  set(options DONT_INSTALL_INIT_PY)
  set(oneValueArgs SOURCE_PYTHON_MODULE MODULE_HEADER)
  cmake_parse_arguments(ARG "${options}" "${oneValueArgs}"
                        "${multiValueArgs}" ${ARGN} )

  # By default the __init__.py file is installed.
  if(NOT DEFINED ARG_SOURCE_PYTHON_MODULE)
    set(DYNAMICGRAPH_MODULE_HEADER ${ARG_MODULE_HEADER})
    configure_file(
      ${PROJECT_JRL_CMAKE_MODULE_DIR}/dynamic_graph/python-module-py.cc.in
      ${PROJECT_BINARY_DIR}/ros1/src/dynamic_graph/${SUBMODULENAME}/python-module-py.cc
      @ONLY
      )
    SET(ARG_SOURCE_PYTHON_MODULE "${PROJECT_BINARY_DIR}/ros1/src/dynamic_graph/${SUBMODULENAME}/python-module-py.cc")
  endif()

  IF(NOT DEFINED PYTHONLIBS_FOUND)
    FINDPYTHON()
  ELSEIF(NOT ${PYTHONLIBS_FOUND} STREQUAL "TRUE")
    MESSAGE(FATAL_ERROR "Python has not been found.")
  ENDIF()
  if(NOT DEFINED Boost_PYTHON_LIBRARIES)
    MESSAGE(FATAL_ERROR "Boost Python library must have been found to call this macro.")
  endif()

  SET(PYTHON_MODULE ${TARGETNAME})

  ADD_LIBRARY(${PYTHON_MODULE}
    MODULE
    ${ARG_SOURCE_PYTHON_MODULE})

  FILE(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/ros1/src/dynamic_graph/${SUBMODULENAME})

  SET(PYTHON_INSTALL_DIR "${PYTHON_SITELIB}/dynamic_graph/${SUBMODULENAME}")
  STRING(REGEX REPLACE "[^/]+" ".." PYTHON_INSTALL_DIR_REVERSE ${PYTHON_INSTALL_DIR})

  SET_TARGET_PROPERTIES(${PYTHON_MODULE}
    PROPERTIES PREFIX ""
    OUTPUT_NAME dynamic_graph/${SUBMODULENAME}/wrap
    BUILD_RPATH "${DYNAMIC_GRAPH_PLUGINDIR}:\$ORIGIN/${PYTHON_INSTALL_DIR_REVERSE}/${DYNAMIC_GRAPH_PLUGINDIR}"
   )

  IF (UNIX AND NOT APPLE)
    TARGET_LINK_LIBRARIES(${PYTHON_MODULE} ${PUBLIC_KEYWORD} "-Wl,--no-as-needed")
  ENDIF(UNIX AND NOT APPLE)
  TARGET_LINK_LIBRARIES(${PYTHON_MODULE} ${PUBLIC_KEYWORD} ${LIBRARYNAME} ${PYTHON_LIBRARY} dynamic-graph::dynamic-graph)
  TARGET_LINK_BOOST_PYTHON(${PYTHON_MODULE} ${PUBLIC_KEYWORD})
  if(PROJECT_NAME STREQUAL "dynamic-graph-python")
    TARGET_LINK_LIBRARIES(${PYTHON_MODULE} ${PUBLIC_KEYWORD} dynamic-graph-python)
  else()
    TARGET_LINK_LIBRARIES(${PYTHON_MODULE} ${PUBLIC_KEYWORD} dynamic-graph-python::dynamic-graph-python)
  endif()

  TARGET_INCLUDE_DIRECTORIES(${PYTHON_MODULE} SYSTEM PRIVATE ${PYTHON_INCLUDE_DIRS})

  #
  # Installation
  #

  INSTALL(TARGETS ${PYTHON_MODULE}
    DESTINATION
    ${PYTHON_INSTALL_DIR})

  SET(ENTITY_CLASS_LIST "")
  FOREACH (ENTITY ${NEW_ENTITY_CLASS})
    SET(ENTITY_CLASS_LIST "${ENTITY_CLASS_LIST}${ENTITY}('')\n")
  ENDFOREACH(ENTITY ${NEW_ENTITY_CLASS})

  # Install if not DONT_INSTALL_INIT_PY
  if(NOT DONT_INSTALL_INIT_PY)

    CONFIGURE_FILE(
      ${PROJECT_JRL_CMAKE_MODULE_DIR}/dynamic_graph/submodule/__init__.py.cmake
      ${PROJECT_BINARY_DIR}/ros1/src/dynamic_graph/${SUBMODULENAME}/__init__.py
      )

    INSTALL(
      FILES ${PROJECT_BINARY_DIR}/ros1/src/dynamic_graph/${SUBMODULENAME}/__init__.py
      DESTINATION ${PYTHON_INSTALL_DIR}
      )

  endif()

ENDMACRO(CUSTOM_DYNAMIC_GRAPH_PYTHON_MODULE SUBMODULENAME)








SET(plugins
  ros_publish
  ros_subscribe
  ros_queued_subscribe
  ros_tf_listener
  ros_time
  )

FOREACH(plugin ${plugins})
  GET_FILENAME_COMPONENT(LIBRARY_NAME ${plugin} NAME)
  ADD_LIBRARY(${LIBRARY_NAME} SHARED ${plugin}.cpp ${plugin}.hh)

  IF(SUFFIX_SO_VERSION)
    SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
  ENDIF(SUFFIX_SO_VERSION)

  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${${LIBRARY_NAME}_deps} ${catkin_LIBRARIES} ros_bridge
    dynamic_graph_bridge_msgs::dynamic_graph_bridge_msgs)
  
  TARGET_INCLUDE_DIRECTORIES(${LIBRARY_NAME} PUBLIC $<INSTALL_INTERFACE:include>
    PRIVATE ${PROJECT_SOURCE_DIR}/ros1/include)

  IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
    INSTALL(TARGETS ${LIBRARY_NAME} EXPORT ${TARGETS_EXPORT_NAME}
      DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR})
  ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)

  IF(BUILD_PYTHON_INTERFACE)
    STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${LIBRARY_NAME})
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python-module-py.cc")
      CUSTOM_DYNAMIC_GRAPH_PYTHON_MODULE("ros/${PYTHON_LIBRARY_NAME}"
        ${LIBRARY_NAME} ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap
        SOURCE_PYTHON_MODULE "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python-module-py.cc")
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python.hh")
      CUSTOM_DYNAMIC_GRAPH_PYTHON_MODULE("ros/${PYTHON_LIBRARY_NAME}"
        ${LIBRARY_NAME} ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap
        MODULE_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python.hh")
    endif()
    TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap
      PRIVATE ${PROJECT_SOURCE_DIR}/ros1/include)
  ENDIF(BUILD_PYTHON_INTERFACE)
ENDFOREACH(plugin)

target_link_libraries(ros_publish ros_bridge)

IF(BUILD_PYTHON_INTERFACE)
  PYTHON_INSTALL_ON_SITE("dynamic_graph/ros" "__init__.py")
  PYTHON_INSTALL_ON_SITE("dynamic_graph/ros" "ros.py")
  PYTHON_INSTALL_ON_SITE("dynamic_graph/ros" "dgcompleter.py")

  # ros_interperter library.
  add_library(ros_interpreter ros_interpreter.cpp)
  TARGET_LINK_LIBRARIES(ros_interpreter ros_bridge ${catkin_LIBRARIES}
    dynamic-graph-python::dynamic-graph-python
    dynamic_graph_bridge_msgs::dynamic_graph_bridge_msgs)
  TARGET_INCLUDE_DIRECTORIES(ros_interpreter PUBLIC $<INSTALL_INTERFACE:include>
    PRIVATE ${PROJECT_SOURCE_DIR}/ros1/include)


  install(TARGETS ros_interpreter
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION lib)
ENDIF(BUILD_PYTHON_INTERFACE)

# Stand alone embedded intepreter with a robot controller.
add_executable(geometric_simu geometric_simu.cpp sot_loader.cpp sot_loader_basic.cpp)
TARGET_INCLUDE_DIRECTORIES(geometric_simu PUBLIC $<INSTALL_INTERFACE:include>
    PRIVATE ${PROJECT_SOURCE_DIR}/ros1/include)
target_link_libraries(geometric_simu Boost::program_options ${CMAKE_DL_LIBS} ${catkin_LIBRARIES} ros_bridge
  dynamic_graph_bridge_msgs::dynamic_graph_bridge_msgs)
install(TARGETS geometric_simu
  DESTINATION lib/${PROJECT_NAME})

# Sot loader library
add_library(sot_loader sot_loader.cpp sot_loader_basic.cpp)
TARGET_INCLUDE_DIRECTORIES(sot_loader PUBLIC $<INSTALL_INTERFACE:include>
    PRIVATE ${PROJECT_SOURCE_DIR}/ros1/include)
target_link_libraries(sot_loader Boost::program_options ${catkin_LIBRARIES} ros_bridge
  dynamic_graph_bridge_msgs::dynamic_graph_bridge_msgs)
install(TARGETS sot_loader EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)
