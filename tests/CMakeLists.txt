IF(BUILD_PYTHON_INTERFACE)
  ADD_PYTHON_UNIT_TEST("py-import" "tests/test_import.py")
  ADD_PYTHON_UNIT_TEST("py-joint_state" "tests/test_joint_state.py")
ENDIF(BUILD_PYTHON_INTERFACE)
