set(plugins ros_publish ros_subscribe ros_queued_subscribe ros_tf_listener
            ros_time)

foreach(plugin ${plugins})
  get_filename_component(LIBRARY_NAME ${plugin} NAME)
  add_library(${LIBRARY_NAME} SHARED ${plugin}.cpp ${plugin}.hh)

  if(SUFFIX_SO_VERSION)
    set_target_properties(${LIBRARY_NAME} PROPERTIES SOVERSION
                                                     ${PROJECT_VERSION})
  endif(SUFFIX_SO_VERSION)

  target_link_libraries(
    ${LIBRARY_NAME} ${${LIBRARY_NAME}_deps} ${catkin_LIBRARIES} ros_bridge)

  if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
    install(
      TARGETS ${LIBRARY_NAME}
      EXPORT ${TARGETS_EXPORT_NAME}
      DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR})
  endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)

  if(BUILD_PYTHON_INTERFACE)
    string(REPLACE - _ PYTHON_LIBRARY_NAME ${LIBRARY_NAME})
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python-module-py.cc")
      dynamic_graph_python_module(
        "ros/${PYTHON_LIBRARY_NAME}" ${LIBRARY_NAME}
        ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap SOURCE_PYTHON_MODULE
        "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python-module-py.cc")
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python.hh")
      dynamic_graph_python_module(
        "ros/${PYTHON_LIBRARY_NAME}" ${LIBRARY_NAME}
        ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap MODULE_HEADER
        "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python.hh")
    endif()
  endif(BUILD_PYTHON_INTERFACE)
endforeach(plugin)

target_link_libraries(ros_publish ros_bridge)

if(BUILD_PYTHON_INTERFACE)
  python_install_on_site("dynamic_graph/ros" "__init__.py")
  python_install_on_site("dynamic_graph/ros" "ros.py")
  python_install_on_site("dynamic_graph/ros" "dgcompleter.py")

  # ros_interperter library.
  add_library(ros_interpreter ros_interpreter.cpp)
  target_link_libraries(
    ros_interpreter ros_bridge ${catkin_LIBRARIES}
    dynamic-graph-python::dynamic-graph-python)

  install(
    TARGETS ros_interpreter
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION lib)
endif(BUILD_PYTHON_INTERFACE)

# Stand alone embedded intepreter with a robot controller.
add_executable(geometric_simu geometric_simu.cpp sot_loader.cpp
                              sot_loader_basic.cpp)
target_link_libraries(
  geometric_simu Boost::program_options ${CMAKE_DL_LIBS} ${catkin_LIBRARIES}
  ros_bridge)
install(TARGETS geometric_simu DESTINATION lib/${PROJECT_NAME})

# Sot loader library
add_library(sot_loader sot_loader.cpp sot_loader_basic.cpp)
target_link_libraries(
  sot_loader Boost::program_options ${catkin_LIBRARIES} ros_bridge)
install(
  TARGETS sot_loader
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)
