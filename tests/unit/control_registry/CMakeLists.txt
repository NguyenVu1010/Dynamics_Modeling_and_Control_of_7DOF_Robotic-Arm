# Feature controller-registry (ADR-023, cobot_studio phase C PR 2): the closed registry of
# tracking laws (control/algorithm.hpp), PD plus gravity compensation, TrackingController and
# ControlLoop's cobot.control.algorithm. The loop tests reuse the shared rig of
# tests/component/control_loop (read only).
set(COBOT_LOOP_RIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../component/control_loop)

cobot_add_test(test_control_registry
  test_algorithms.cpp
  test_pd_gravity.cpp
  test_tracking_controller.cpp
  test_loop_registry.cpp)
target_include_directories(test_control_registry PRIVATE ${COBOT_LOOP_RIG_DIR})

# Replaces the global operator new: its own binary.
cobot_add_test(test_control_registry_no_alloc test_registry_no_alloc.cpp)
target_include_directories(test_control_registry_no_alloc PRIVATE ${COBOT_LOOP_RIG_DIR})
