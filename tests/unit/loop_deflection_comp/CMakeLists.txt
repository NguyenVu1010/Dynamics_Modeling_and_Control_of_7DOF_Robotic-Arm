# Feature deflection-comp (realism-2 design 4.7, ADR-019): the controller removes the static
# Harmonic Drive deflection from motor-encoder positions (q_hat = q - deflection(g(q))).
# The rigid-reference test reuses the E2E-01 runner of tests/unit/sim_plant_transmission (read
# only).
set(COBOT_TRANSMISSION_TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../sim_plant_transmission)

cobot_add_test(test_loop_deflection_comp
  test_harmonic_spring_deflection.cpp
  test_config_deflection_comp.cpp
  test_q_from_motor.cpp
  test_deflection_comp_loop.cpp)
target_include_directories(test_loop_deflection_comp PRIVATE ${COBOT_TRANSMISSION_TEST_DIR})

# Elastic 7-DOF closed loops (12 s each, four runs): their own binary.
cobot_add_test(test_loop_deflection_comp_closed_loop test_deflection_comp_closed_loop.cpp)

# Replaces the global operator new: its own binary.
cobot_add_test(test_loop_deflection_comp_no_alloc test_deflection_comp_no_alloc.cpp)
