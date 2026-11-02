# SimPlant caps the applied torque at the plant model's tau_max (feature joint-limits).
cobot_add_test(test_sim_plant_torque_cap test_sim_plant_torque_cap.cpp)
