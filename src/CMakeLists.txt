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

  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${${LIBRARY_NAME}_deps} ${catkin_LIBRARIES} ros_bridge)

  IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
    INSTALL(TARGETS ${LIBRARY_NAME} EXPORT ${TARGETS_EXPORT_NAME}
      DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR})
  ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)

  IF(BUILD_PYTHON_INTERFACE)
    STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${LIBRARY_NAME})
    DYNAMIC_GRAPH_PYTHON_MODULE("ros/${PYTHON_LIBRARY_NAME}"
      ${LIBRARY_NAME} ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap)
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
    dynamic-graph-python::dynamic-graph-python)

  install(TARGETS ros_interpreter
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION lib)
ENDIF(BUILD_PYTHON_INTERFACE)

# Stand alone embedded intepreter with a robot controller.
add_executable(geometric_simu geometric_simu.cpp sot_loader.cpp sot_loader_basic.cpp)
target_link_libraries(geometric_simu ${Boost_LIBRARIES} ${CMAKE_DL_LIBS} ${catkin_LIBRARIES} ros_bridge)
install(TARGETS geometric_simu
  DESTINATION lib/${PROJECT_NAME})

# Sot loader library
add_library(sot_loader sot_loader.cpp sot_loader_basic.cpp)
target_link_libraries(sot_loader ${Boost_LIBRARIES} ${catkin_LIBRARIES} ros_bridge)
install(TARGETS sot_loader EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)
