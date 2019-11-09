IF(BUILD_PYTHON_INTERFACE)
  INSTALL(FILES
    "dynamic_graph/ros/__init__.py"
    "dynamic_graph/ros/ros.py"
    "dynamic_graph/ros/dgcompleter.py"
    DESTINATION "${PYTHON_SITELIB}/dynamic_graph/ros"
    )
ENDIF(BUILD_PYTHON_INTERFACE)

