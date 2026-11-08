Screening and comparison tool, not a certification: it does not replace a safety assessment to ISO 10218 or ISO/TS 15066, nor measurements on the real robot.

**MARGINAL, measure on hardware**

# Verify: budget_xarm7, drive_position, reference_move

Gain: config.drive.omega_rad_s = 50 (tuned)

## Runs

| runs | seeds | passed | pass rate | Wilson 95 % | held | hold rate |
|---|---|---|---|---|---|---|
| 100 | 1..100 | 92 | 0.92 | 0.85 .. 0.9589 | 0 | 0 |

## Worst metrics (runs that did not hold)

| metric | worst | threshold | margin limit |
|---|---|---|---|
| final_pos_err_m | 0.007505 | 0.005 | 0.004 |
| final_rot_err_deg | 0.6629 | 2 | 1.6 |
| rms_joint_err_rad | 0.007414 | 0.02 | 0.016 |
| cycle_time_s | - | 12 | 9.6 |

## Tuning

config.drive.omega_rad_s: level 50: lowest median rms joint error (0.0032 rad) of 9 admissible of 19 levels

| level | seeds | held | sat_frac_max | decays_all | rms_joint_err_median | final_pos_err_max | chosen |
|---|---|---|---|---|---|---|---|
| 20 | 10 | 0 | 0 | 1 | 0.005334 | 0.009829 | 0 |
| 30 | 10 | 0 | 0 | 1 | 0.003968 | 0.007215 | 0 |
| 40 | 10 | 0 | 0 | 1 | 0.003277 | 0.007348 | 0 |
| 50 | 10 | 0 | 0 | 1 | 0.003204 | 0.007353 | 1 |
| 60 | 10 | 0 | 0 | 1 | 0.003263 | 0.007355 | 0 |
| 70 | 10 | 0 | 0 | 1 | 0.003295 | 0.007366 | 0 |
| 80 | 10 | 0 | 0 | 1 | 0.00331 | 0.007375 | 0 |
| 90 | 10 | 0 | 0 | 1 | 0.003329 | 0.007323 | 0 |
| 100 | 10 | 0 | 0 | 1 | 0.003343 | 0.007321 | 0 |
| 110 | 10 | 0 | 0 | 0 | 0.003374 | 0.007322 | 0 |
| 120 | 10 | 0 | 0 | 0 | 0.003466 | 0.00727 | 0 |
| 130 | 10 | 0 | 0 | 0 | 0.00352 | 0.007295 | 0 |
| 140 | 10 | 2 | 0.0006639 | 0 | 0.003656 | 0.009422 | 0 |
| 150 | 10 | 5 | 0.01846 | 0 | 0.005708 | 0.01542 | 0 |
| 160 | 10 | 5 | 0.07858 | 0 | 0.008944 | 0.01695 | 0 |
| 170 | 10 | 7 | 0.09737 | 0 | 0.009434 | 0.01384 | 0 |
| 180 | 10 | 7 | 0.1556 | 0 | 0.00981 | 0.01689 | 0 |
| 190 | 10 | 10 | 0.06485 | 0 | - | - | 0 |
| 200 | 10 | 10 | 0.3446 | 0 | - | - | 0 |

## Assumptions

| key | kind | bounds | base passed | low passed | high passed | verdict stable |
|---|---|---|---|---|---|---|
| sensor.q_noise_std_rad | derived | range 2.5e-05 .. 0.0001 | 9/10 | 9/10 | 9/10 | stable |
| sensor.q_offset_std_rad | assumed | range 0.0005 .. 0.005 | 9/10 | 10/10 | 3/10 | may change |
| sensor.qd_from_diff | assumed | - | - | - | - | not varied |
| sensor.qd_filter_hz | assumed | range 25 .. 100 | 9/10 | 9/10 | 9/10 | stable |
| sensor.sensor_delay_cycles | assumed | range 0 .. 3 | 9/10 | 9/10 | 9/10 | stable |
| sensor.position_source | assumed | - | - | - | - | not varied |
| actuator.torque_scale_std | assumed | range 0.05 .. 0.2 | 9/10 | 9/10 | 8/10 | stable |
| actuator.torque_scale_asym_frac | assumed | range 0 .. 0.06 | 9/10 | 9/10 | 9/10 | stable |
| actuator.torque_ripple_frac | assumed | range 0.02 .. 0.1 | 9/10 | 9/10 | 9/10 | stable |
| actuator.ripple_periods_per_rev | assumed | range 2 .. 12 | 9/10 | 9/10 | 9/10 | stable |
| actuator.deadband_nm | assumed | scale 0.2 .. 3 | 9/10 | 9/10 | 9/10 | stable |
| actuator.torque_cmd_resolution_nm | assumed | scale 0.25 .. 2 | 9/10 | 9/10 | 9/10 | stable |
| actuator.motor_time_constant_s | assumed | range 0.0005 .. 0.005 | 9/10 | 9/10 | 9/10 | stable |
| actuator.command_delay_cycles | assumed | range 0 .. 3 | 9/10 | 9/10 | 9/10 | stable |
| actuator.torque_slew_nm_s | assumed | scale 0.5 .. 2 | 9/10 | 9/10 | 9/10 | stable |
| actuator.knee_speed_rad_s | derived | scale 0.8 .. 1.25 | 9/10 | 9/10 | 9/10 | stable |
| actuator.zero_torque_speed_rad_s | derived | scale 0.8 .. 1.25 | 9/10 | 9/10 | 9/10 | stable |
| actuator.bus.r_source_ohm | assumed | range 0.05 .. 0.5 | 9/10 | 9/10 | 9/10 | stable |
| actuator.bus.v_min_v | assumed | range 36 .. 43.2 | 9/10 | 9/10 | 9/10 | stable |
| friction.coulomb_nm | derived | scale 0.5 .. 1.5 | 9/10 | 9/10 | 9/10 | stable |
| friction.viscous_nm_s_rad | assumed | scale 0.5 .. 2 | 9/10 | 9/10 | 9/10 | stable |
| model.link_mass_scale_std | assumed | range 0.02 .. 0.2 | 9/10 | 9/10 | 9/10 | stable |
| model.com_shift_std_m | assumed | range 0.002 .. 0.02 | 9/10 | 9/10 | 9/10 | stable |
| timing.overrun_prob | assumed | range 0.0001 .. 0.01 | 9/10 | 9/10 | 9/10 | stable |
| timing.overrun_cycles | assumed | range 1 .. 5 | 9/10 | 9/10 | 9/10 | stable |
| comm.packet_loss_prob | assumed | range 0.0001 .. 0.01 | 9/10 | 9/10 | 9/10 | stable |
| comm.latency_burst_prob | assumed | range 0.0001 .. 0.01 | 9/10 | 9/10 | 7/10 | may change |
| comm.burst_cycles | assumed | range 1 .. 10 | 9/10 | 9/10 | 8/10 | may change |
| transmission.damping_ratio | assumed | range 0.005 .. 0.05 | 9/10 | 9/10 | 9/10 | stable |
| transmission.error_amp_rad | assumed | range 0.00029 .. 0.0017 | 9/10 | 9/10 | 9/10 | stable |
| transmission.lost_motion_rad | assumed | range 0.00029 .. 0.0017 | 9/10 | 9/10 | 9/10 | stable |

## Versions

git: 6ae849e

| input | sha256 |
|---|---|
| /home/runner/work/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/config/hardware/budget_xarm7.yaml | 1fedba18d2beba7cd96c10bcc001c939f8d2a697477e3753559468e6b673d449 |
| /home/runner/work/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/config/imperfections/budget_xarm7.yaml | 12ed5edf10110806c50e5659ca321a152611335224f60c021a084a40a6d70941 |
| /home/runner/work/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/config/budget_xarm7.yaml | f99f4b7418e3f409ca42ff6d9102e14c79a50d90b4d39b5471f8620edb2479ba |
| /home/runner/work/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/config/tasks/reference_move.yaml | 601b56df0d0d9b8295c4ff6d25174bf48609400623cbb3edeb7690a01ed0daca |
