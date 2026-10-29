# Feature model-armature: reflected rotor inertia in M(q), RNEA and forward dynamics. Own
# directory so the frozen dynamics CMakeLists stays as is.
cobot_add_test(test_dynamics_armature test_armature_dynamics.cpp LIBS cobot::dynamics)
target_include_directories(test_dynamics_armature PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../dynamics)
