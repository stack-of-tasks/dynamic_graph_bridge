# Copyright (C) 2021 LAAS-CNRS.
#
# Author: Maxmilien Naveau
#

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  _ament_cmake_gmock_find_gmock()
  find_package(launch_testing_ament_cmake)
  ament_lint_auto_find_test_dependencies()

  # Library for sot_mock_device
  add_library(impl_test_sot_mock_device SHARED impl_test_sot_mock_device.cpp)
  set_target_properties(impl_test_sot_mock_device PROPERTIES INSTALL_RPATH
                                                             $ORIGIN)
  target_link_libraries(
    impl_test_sot_mock_device
    PUBLIC sot-core::sot-core dynamic-graph-python::dynamic-graph-python
           ros_bridge)

  install(
    TARGETS impl_test_sot_mock_device
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR})

  file(
    MAKE_DIRECTORY
    ${PROJECT_BINARY_DIR}/tests/dynamic_graph/ros/tests/impl_test_sot_mock_device
  )

  dynamic_graph_python_module(
    "${PYTHON_DIR}/ros/tests/impl_test_sot_mock_device"
    impl_test_sot_mock_device
    dynamic_graph_bridge-impl_test_sot_mock_device-wrap
    SOURCE_PYTHON_MODULE
    "${CMAKE_CURRENT_SOURCE_DIR}/impl_test_sot_mock_device-python-module-py.cc")

  # Library for sot_external_interface
  add_library(impl_test_library SHARED impl_test_sot_external_interface.cpp)
  set_target_properties(impl_test_library PROPERTIES INSTALL_RPATH $ORIGIN)

  target_link_libraries(impl_test_library PUBLIC impl_test_sot_mock_device
                                                 sot-core::sot-core)
  ament_target_dependencies(impl_test_library PUBLIC dynamic_graph_bridge_msgs
                            rclcpp rcl_interfaces std_srvs)

  install(
    TARGETS impl_test_library
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR})

  # Executable for SotLoaderBasic test
  add_executable(test_sot_loader_basic test_sot_loader_basic.cpp)
  target_include_directories(
    test_sot_loader_basic
    PUBLIC include "${GMOCK_INCLUDE_DIRS}"
    PRIVATE ${PROJECT_SOURCE_DIR}/include)
  target_link_libraries(test_sot_loader_basic sot_loader "${GMOCK_LIBRARIES}")

  add_launch_test(launch/launching_test_sot_loader_basic.py)

  # Test for class SotLoader
  add_executable(test_sot_loader test_sot_loader.cpp)
  target_include_directories(
    test_sot_loader
    PUBLIC include "${GMOCK_INCLUDE_DIRS}"
    PRIVATE ${PROJECT_SOURCE_DIR}/include)
  target_link_libraries(test_sot_loader sot_loader "${GMOCK_LIBRARIES}")

  add_launch_test(launch/launching_test_sot_loader.py)

  # Install tests
  install(TARGETS test_sot_loader_basic test_sot_loader
          DESTINATION lib/${PROJECT_NAME})

  # Install library for tests
  install(TARGETS impl_test_library DESTINATION lib)

  install(DIRECTORY launch urdf DESTINATION share/${PROJECT_NAME})

  #
  # this macro define a unit tests using gtest
  #
  macro(create_ros_bridge_unittest test_name dependencies)
    # Set a general config folder path for all tests
    set(TEST_CONFIG_PATH ${PROJECT_SOURCE_DIR}/tests/config/)

    # Create the cmake target using ament and gtest.
    ament_add_gtest(test_${test_name} main.cpp test_${test_name}.cpp)

    # Include dependencies.
    target_include_directories(
      test_${test_name}
      PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
             $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/demos>
             $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/tests>
             $<INSTALL_INTERFACE:include>
             SYSTEM
      PUBLIC ${PYTHON_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
      PRIVATE ${PROJECT_SOURCE_DIR}/include)

    # Link the dependecies to it.
    #
    target_link_libraries(test_${test_name} ros_bridge)
    set(current_test_name test_${test_name})
    ament_target_dependencies(${current_test_name} dynamic_graph_bridge_msgs
                              rclcpp rcl_interfaces std_srvs)

    # add some preprocessor variable
    target_compile_definitions(
      test_${test_name}
      PUBLIC TEST_CONFIG_PATH="${CMAKE_CURRENT_LIST_DIR}/python_scripts/")
  endmacro(create_ros_bridge_unittest test_name)

  # C++ unit-tests.
  find_package(ament_cmake_gtest)
  create_ros_bridge_unittest(ros_init dynamic_graph_bridge)
  create_ros_bridge_unittest(ros_interpreter dynamic_graph_bridge)

endif()
