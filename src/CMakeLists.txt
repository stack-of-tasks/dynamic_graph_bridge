EXEC_PROGRAM(
  ${PYTHON_EXECUTABLE}
  ARGS
  "-c \"from distutils import sysconfig; print sysconfig.get_python_lib(0,0,prefix='')\""
  OUTPUT_VARIABLE PYTHON_SITELIB
  )
PYTHON_INSTALL("dynamic_graph/ros" "__init__.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL("dynamic_graph/ros" "ros.py" "${PYTHON_SITELIB}")