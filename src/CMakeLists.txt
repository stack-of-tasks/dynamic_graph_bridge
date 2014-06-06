PYTHON_INSTALL("dynamic_graph/ros" "__init__.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL("dynamic_graph/ros" "ros.py" "${PYTHON_SITELIB}")

# DelPrete: add main library
#SET(sources   converter.hh        robot_model.hh    ros_joint_state.hh
#              ros_publish.hh      ros_subscribe.hh  ros_time.hh
#              sot_loader.hh       sot_to_ros.hh
#              geometric_simu.cpp  robot_model.cpp  ros_interpreter.cpp
#              ros_publish.cpp     ros_time.cpp     sot_to_ros.cpp
#              interpreter.cpp     ros_init.cpp     ros_joint_state.cpp
#              ros_subscribe.cpp   sot_loader.cpp )
SET(sources   robot_model.hh     robot_model.cpp)
SET(LIBRARY_NAME ${PROJECT_NAME})
ADD_LIBRARY(${LIBRARY_NAME} SHARED ${sources})

TARGET_LINK_LIBRARIES(${LIBRARY_NAME} sot-core)
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} sot-dynamic)
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} dynamic-graph)

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} jrl-mal)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} sot-core)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} sot-dynamic)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} dynamic-graph)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} jrl-dynamics-urdf)

set_target_properties(${LIBRARY_NAME} PROPERTIES BUILD_WITH_INSTALL_RPATH True)

# Do not use ${CMAKE_INSTALL_LIBDIR} because it is not set to "lib"
INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)
# headers are already installed

