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

  # Library for sot_external_interface
  add_library(impl_test_sot_external_interface SHARED
    tests/impl_test_sot_external_interface)

  target_link_libraries(impl_test_sot_external_interface PUBLIC sot-core::sot-core)

  # Executable for SotLoaderBasic test
  add_executable(test_sot_loader_basic
    tests/test_sot_loader_basic.cpp
  )
  target_include_directories(test_sot_loader_basic PUBLIC include  "${GMOCK_INCLUDE_DIRS}")
  target_link_libraries(test_sot_loader_basic sot_loader "${GMOCK_LIBRARIES}" )

  add_launch_test(tests/launching_test_sot_loader_basic.py)

  # Test for class SotLoader
  add_executable(test_sot_loader
    tests/test_sot_loader.cpp
  )
  target_include_directories(test_sot_loader PUBLIC include "${GMOCK_INCLUDE_DIRS}")
  target_link_libraries(test_sot_loader sot_loader "${GMOCK_LIBRARIES}")

  add_launch_test(tests/launching_test_sot_loader.py)

  # Install tests
  install(TARGETS test_sot_loader_basic
    test_sot_loader
    DESTINATION lib/${PROJECT_NAME}
    )

  # Install library for tests
  install(TARGETS impl_test_sot_external_interface
    DESTINATION lib
    )

  install(DIRECTORY launch urdf
    DESTINATION share/${PROJECT_NAME}
    )

endif()
