cobot_add_test(test_dynamics
  test_analytic.cpp
  test_arm_properties.cpp
  test_kdl_reference.cpp
  test_no_alloc.cpp
  LIBS cobot::dynamics)
