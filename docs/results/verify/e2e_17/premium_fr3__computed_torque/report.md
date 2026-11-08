Screening and comparison tool, not a certification: it does not replace a safety assessment to ISO 10218 or ISO/TS 15066, nor measurements on the real robot.

**PASS**

# Verify: premium_fr3, computed_torque, reference_move

Gain: config.computed_torque.omega = 50 (tuned)

## Runs

| runs | seeds | passed | pass rate | Wilson 95 % | held | hold rate |
|---|---|---|---|---|---|---|
| 100 | 101..200 | 100 | 1 | 0.963 .. 1 | 0 | 0 |

## Worst metrics (runs that did not hold)

| metric | worst | threshold | margin limit |
|---|---|---|---|
| final_pos_err_m | 0.0001944 | 0.005 | 0.004 |
| final_rot_err_deg | 0.03774 | 2 | 1.6 |
| rms_joint_err_rad | 0.0004729 | 0.02 | 0.016 |
| cycle_time_s | 9.119 | 12 | 9.6 |

## Tuning

config.computed_torque.omega: level 50: lowest median rms joint error (0.000296 rad) of 4 admissible of 14 levels

| level | seeds | held | sat_frac_max | decays_all | rms_joint_err_median | final_pos_err_max | chosen |
|---|---|---|---|---|---|---|---|
| 20 | 10 | 0 | 0 | 1 | 0.002014 | 0.001279 | 0 |
| 30 | 10 | 0 | 0 | 1 | 0.000884 | 0.0005813 | 0 |
| 40 | 10 | 0 | 0 | 1 | 0.0004816 | 0.0002504 | 0 |
| 50 | 10 | 0 | 0 | 1 | 0.0002959 | 0.0001327 | 1 |
| 60 | 10 | 0 | 0.05358 | 0 | 0.000372 | 0.001099 | 0 |
| 70 | 10 | 0 | 0.2826 | 0 | 0.0007989 | 0.002123 | 0 |
| 80 | 10 | 0 | 0.5455 | 0 | 0.0009808 | 0.002769 | 0 |
| 90 | 10 | 0 | 0.7419 | 0 | 0.001079 | 0.002846 | 0 |
| 100 | 10 | 0 | 0.8162 | 0 | 0.001227 | 0.002911 | 0 |
| 110 | 10 | 0 | 0.8523 | 0 | 0.001246 | 0.003032 | 0 |
| 120 | 10 | 0 | 0.8725 | 0 | 0.001292 | 0.003201 | 0 |
| 130 | 10 | 0 | 0.9008 | 0 | 0.001355 | 0.003349 | 0 |
| 140 | 10 | 0 | 0.9157 | 0 | 0.001446 | 0.003468 | 0 |
| 150 | 10 | 0 | 0.9294 | 0 | 0.001524 | 0.003119 | 0 |

## Assumptions

| key | kind | bounds | base passed | low passed | high passed | verdict stable |
|---|---|---|---|---|---|---|
| sensor.q_noise_std_rad | derived | range 2.5e-06 .. 1e-05 | - | - | - | not run |
| sensor.q_offset_std_rad | assumed | range 5e-05 .. 0.001 | - | - | - | not run |
| sensor.qd_from_diff | assumed | - | - | - | - | not run |
| sensor.qd_filter_hz | assumed | range 100 .. 500 | - | - | - | not run |
| sensor.sensor_delay_cycles | assumed | range 0 .. 3 | - | - | - | not run |
| sensor.position_source | assumed | - | - | - | - | not run |
| actuator.torque_scale_asym_frac | assumed | range 0 .. 0.02 | - | - | - | not run |
| actuator.torque_ripple_frac | assumed | range 0 .. 0.03 | - | - | - | not run |
| actuator.ripple_periods_per_rev | assumed | range 2 .. 12 | - | - | - | not run |
| actuator.motor_time_constant_s | assumed | range 0.0002 .. 0.002 | - | - | - | not run |
| actuator.command_delay_cycles | assumed | range 0 .. 3 | - | - | - | not run |
| actuator.torque_slew_nm_s | assumed | scale 0.5 .. 2 | - | - | - | not run |
| actuator.knee_speed_rad_s | derived | scale 0.8 .. 1.25 | - | - | - | not run |
| actuator.zero_torque_speed_rad_s | derived | scale 0.8 .. 1.25 | - | - | - | not run |
| actuator.bus.r_source_ohm | assumed | range 0.01 .. 0.15 | - | - | - | not run |
| actuator.bus.v_min_v | assumed | range 36 .. 43.2 | - | - | - | not run |
| friction.viscous_nm_s_rad | assumed | scale 0.5 .. 2 | - | - | - | not run |
| model.link_mass_scale_std | assumed | range 0.005 .. 0.05 | - | - | - | not run |
| model.com_shift_std_m | assumed | range 0.0005 .. 0.005 | - | - | - | not run |
| timing.overrun_prob | assumed | range 1e-06 .. 0.0001 | - | - | - | not run |
| timing.overrun_cycles | assumed | range 1 .. 3 | - | - | - | not run |
| comm.packet_loss_prob | assumed | range 1e-06 .. 0.0001 | - | - | - | not run |
| comm.latency_burst_prob | assumed | range 1e-06 .. 0.0001 | - | - | - | not run |
| comm.burst_cycles | assumed | range 1 .. 3 | - | - | - | not run |
| transmission.damping_ratio | assumed | range 0.005 .. 0.05 | - | - | - | not run |

## Versions

git: 6ae849e

| input | sha256 |
|---|---|
| /home/runner/work/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/config/hardware/premium_fr3.yaml | 3db531e91aa390689b6fcdc92e11617b2476dbcd405842a6167456b83045e0df |
| /home/runner/work/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/config/imperfections/premium_fr3.yaml | 8901a3bbc84404f0f9fa07678caa3db8d3c59eb65720499ac2740750811b9a68 |
| /home/runner/work/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/config/premium_fr3.yaml | af837d1d618799807c46d21a69a2527b43dbe9b8aa9644107a729dc02f78b7a8 |
| /home/runner/work/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/config/tasks/reference_move.yaml | 601b56df0d0d9b8295c4ff6d25174bf48609400623cbb3edeb7690a01ed0daca |
