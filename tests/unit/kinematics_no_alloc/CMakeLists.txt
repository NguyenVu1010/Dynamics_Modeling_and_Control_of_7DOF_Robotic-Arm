# Library-level no-allocation check of the kinematics hot path (FK, Jacobian, Jdot qd): the
# replaced global operator new counts allocations made inside the compiled cobot_kinematics,
# which EIGEN_RUNTIME_NO_MALLOC in the test translation unit cannot see.
cobot_add_test(test_kinematics_no_alloc test_kinematics_no_alloc.cpp LIBS cobot::kinematics)
