# Hardware requirements

Worst level of each param at which every seed still meets the thresholds.

| param | worst passing level | first failing level | metric that fails |
| --- | --- | --- | --- |
| `sensor.encoder_bits` | 10 | - | - |
| `sensor.q_noise_std_rad` | 0.001 | - | - |
| `sensor.sensor_delay_cycles` | 4 | 8 | final_pos_err_m, final_rot_err_deg |
| `actuator.command_delay_cycles` | 8 | - | - |
| `actuator.torque_scale_std` | 0.2 | 0.3 | final_pos_err_m |
| `actuator.torque_ripple_frac` | 0.1 | - | - |
| `actuator.torque_cmd_resolution_nm` | 2 | - | - |
| `actuator.motor_time_constant_s` | 0.01 | - | - |
| `friction.coulomb_nm` | 8 | - | - |
| `model.link_mass_scale_std` | 0.2 | - | - |
| `model.payload_kg` | 3 | - | - |
| `timing.period_jitter_std_us` | 300 | - | - |
