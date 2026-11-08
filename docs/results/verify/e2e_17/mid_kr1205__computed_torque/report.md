Screening and comparison tool, not a certification: it does not replace a safety assessment to ISO 10218 or ISO/TS 15066, nor measurements on the real robot.

**PASS**

# Verify: mid_kr1205, computed_torque, reference_move

Gain: config.computed_torque.omega = 40 (tuned)

## Runs

| runs | seeds | passed | pass rate | Wilson 95 % | held | hold rate |
|---|---|---|---|---|---|---|
| 100 | 101..200 | 100 | 1 | 0.963 .. 1 | 0 | 0 |

## Worst metrics (runs that did not hold)

| metric | worst | threshold | margin limit |
|---|---|---|---|
| final_pos_err_m | 0.0008838 | 0.005 | 0.004 |
| final_rot_err_deg | 0.1064 | 2 | 1.6 |
| rms_joint_err_rad | 0.001701 | 0.02 | 0.016 |
| cycle_time_s | 9.144 | 12 | 9.6 |

## Tuning

config.computed_torque.omega: level 40: lowest median rms joint error (0.000427 rad) of 3 admissible of 14 levels

| level | seeds | held | sat_frac_max | decays_all | rms_joint_err_median | final_pos_err_max | chosen |
|---|---|---|---|---|---|---|---|
| 20 | 10 | 0 | 0 | 1 | 0.001848 | 0.002554 | 0 |
| 30 | 10 | 0 | 0 | 1 | 0.0008171 | 0.001184 | 0 |
| 40 | 10 | 0 | 0 | 1 | 0.0004265 | 0.0005806 | 1 |
| 50 | 10 | 0 | 0.1243 | 0 | 0.0009011 | 0.002928 | 0 |
| 60 | 10 | 0 | 0.5009 | 0 | 0.003482 | 0.007661 | 0 |
| 70 | 10 | 0 | 0.7944 | 0 | 0.006454 | 0.009746 | 0 |
| 80 | 10 | 1 | 0.9002 | 0 | 0.008521 | 0.01012 | 0 |
| 90 | 10 | 0 | 0.9399 | 0 | 0.01204 | 0.01124 | 0 |
| 100 | 10 | 1 | 0.9913 | 0 | 0.01331 | 0.01143 | 0 |
| 110 | 10 | 4 | 0.9961 | 0 | 0.01317 | 0.01211 | 0 |
| 120 | 10 | 9 | 0.9946 | 0 | 0.009827 | 0.009489 | 0 |
| 130 | 10 | 10 | 0.9978 | 0 | - | - | 0 |
| 140 | 10 | 10 | 0.9983 | 0 | - | - | 0 |
| 150 | 10 | 10 | 0.9987 | 0 | - | - | 0 |

## Assumptions

| key | kind | bounds | base passed | low passed | high passed | verdict stable |
|---|---|---|---|---|---|---|
| sensor.q_noise_std_rad | derived | range 5e-06 .. 2e-05 | - | - | - | not run |
| sensor.q_offset_std_rad | assumed | range 0.0002 .. 0.003 | - | - | - | not run |
| sensor.qd_from_diff | assumed | - | - | - | - | not run |
| sensor.qd_filter_hz | assumed | range 50 .. 200 | - | - | - | not run |
| sensor.sensor_delay_cycles | assumed | range 0 .. 3 | - | - | - | not run |
| sensor.position_source | assumed | - | - | - | - | not run |
| actuator.torque_scale_std | assumed | range 0.02 .. 0.1 | - | - | - | not run |
| actuator.torque_scale_asym_frac | assumed | range 0 .. 0.03 | - | - | - | not run |
| actuator.torque_ripple_frac | assumed | range 0.005 .. 0.05 | - | - | - | not run |
| actuator.ripple_periods_per_rev | assumed | range 2 .. 12 | - | - | - | not run |
| actuator.deadband_nm | assumed | scale 0.2 .. 3 | - | - | - | not run |
| actuator.motor_time_constant_s | assumed | range 0.0003 .. 0.003 | - | - | - | not run |
| actuator.command_delay_cycles | assumed | range 0 .. 3 | - | - | - | not run |
| actuator.torque_slew_nm_s | assumed | scale 0.5 .. 2 | - | - | - | not run |
| actuator.knee_speed_rad_s | derived | scale 0.8 .. 1.25 | - | - | - | not run |
| actuator.zero_torque_speed_rad_s | derived | scale 0.8 .. 1.25 | - | - | - | not run |
| actuator.bus.r_source_ohm | assumed | range 0.02 .. 0.25 | - | - | - | not run |
| actuator.bus.v_min_v | assumed | range 36 .. 43.2 | - | - | - | not run |
| friction.coulomb_nm | derived | scale 0.5 .. 1.5 | - | - | - | not run |
| friction.viscous_nm_s_rad | assumed | scale 0.5 .. 2 | - | - | - | not run |
| model.link_mass_scale_std | assumed | range 0.01 .. 0.1 | - | - | - | not run |
| model.com_shift_std_m | assumed | range 0.001 .. 0.01 | - | - | - | not run |
| timing.period_jitter_std_us | assumed | range 5 .. 50 | - | - | - | not run |
| timing.overrun_prob | assumed | range 1e-05 .. 0.001 | - | - | - | not run |
| timing.overrun_cycles | assumed | range 1 .. 3 | - | - | - | not run |
| comm.packet_loss_prob | assumed | range 1e-05 .. 0.001 | - | - | - | not run |
| comm.latency_burst_prob | assumed | range 1e-05 .. 0.001 | - | - | - | not run |
| comm.burst_cycles | assumed | range 1 .. 5 | - | - | - | not run |
| transmission.damping_ratio | assumed | range 0.005 .. 0.05 | - | - | - | not run |

## Versions

git: 6ae849e

| input | sha256 |
|---|---|
| /home/runner/work/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/config/hardware/mid_kr1205.yaml | 417f9a1b8b3858398955636130f7a39ab6a00ed6c71c0c2195d9d06e0107f3f6 |
| /home/runner/work/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/config/imperfections/mid_kr1205.yaml | b2c659a0c66042fa09b8bbca97a2dafe3d3b89c4bb40d95ed04cb8dc78a0c45e |
| /home/runner/work/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/config/mid_kr1205.yaml | d7f454b79a5cdd97fe595232b781c0722fc8bf84fd4745f75ae265081c917a91 |
| /home/runner/work/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/config/tasks/reference_move.yaml | 601b56df0d0d9b8295c4ff6d25174bf48609400623cbb3edeb7690a01ed0daca |
