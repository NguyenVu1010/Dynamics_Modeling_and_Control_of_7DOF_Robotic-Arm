Screening and comparison tool, not a certification: it does not replace a safety assessment to ISO 10218 or ISO/TS 15066, nor measurements on the real robot.

**FAIL**

# Verify: budget_xarm7, computed_torque, reference_move

Gain: config.computed_torque.omega = 20 (no admissible gain: verdict on the model's default gain)

## Runs

| runs | seeds | passed | pass rate | Wilson 95 % | held | hold rate |
|---|---|---|---|---|---|---|
| 100 | 1..100 | 0 | 0 | 0 .. 0.03699 | 100 | 1 |

## Worst metrics (runs that did not hold)

| metric | worst | threshold | margin limit |
|---|---|---|---|
| final_pos_err_m | - | 0.005 | 0.004 |
| final_rot_err_deg | - | 2 | 1.6 |
| rms_joint_err_rad | - | 0.02 | 0.016 |
| cycle_time_s | - | 12 | 9.6 |

## Tuning

config.computed_torque.omega: no admissible gain of 14 levels: held 14, saturated 14, not decaying 14, no rms 14

| level | seeds | held | sat_frac_max | decays_all | rms_joint_err_median | final_pos_err_max | chosen |
|---|---|---|---|---|---|---|---|
| 20 | 10 | 10 | 0.565 | 0 | - | - | 0 |
| 30 | 10 | 10 | 0.9694 | 0 | - | - | 0 |
| 40 | 10 | 10 | 0.9878 | 0 | - | - | 0 |
| 50 | 10 | 10 | 0.9926 | 0 | - | - | 0 |
| 60 | 10 | 10 | 0.9956 | 0 | - | - | 0 |
| 70 | 10 | 10 | 0.9965 | 0 | - | - | 0 |
| 80 | 10 | 10 | 0.9975 | 0 | - | - | 0 |
| 90 | 10 | 10 | 0.9978 | 0 | - | - | 0 |
| 100 | 10 | 10 | 0.9979 | 0 | - | - | 0 |
| 110 | 10 | 10 | 0.9979 | 0 | - | - | 0 |
| 120 | 10 | 10 | 0.9982 | 0 | - | - | 0 |
| 130 | 10 | 10 | 0.9982 | 0 | - | - | 0 |
| 140 | 10 | 10 | 0.9985 | 0 | - | - | 0 |
| 150 | 10 | 10 | 0.9988 | 0 | - | - | 0 |

## Assumptions

| key | kind | bounds | base passed | low passed | high passed | verdict stable |
|---|---|---|---|---|---|---|
| sensor.q_noise_std_rad | derived | range 2.5e-05 .. 0.0001 | 0/10 | 0/10 | 0/10 | stable |
| sensor.q_offset_std_rad | assumed | range 0.0005 .. 0.005 | 0/10 | 0/10 | 0/10 | stable |
| sensor.qd_from_diff | assumed | - | - | - | - | not varied |
| sensor.qd_filter_hz | assumed | range 25 .. 100 | 0/10 | 0/10 | 0/10 | stable |
| sensor.sensor_delay_cycles | assumed | range 0 .. 3 | 0/10 | 0/10 | 0/10 | stable |
| sensor.position_source | assumed | - | - | - | - | not varied |
| actuator.torque_scale_std | assumed | range 0.05 .. 0.2 | 0/10 | 0/10 | 0/10 | stable |
| actuator.torque_scale_asym_frac | assumed | range 0 .. 0.06 | 0/10 | 0/10 | 0/10 | stable |
| actuator.torque_ripple_frac | assumed | range 0.02 .. 0.1 | 0/10 | 0/10 | 0/10 | stable |
| actuator.ripple_periods_per_rev | assumed | range 2 .. 12 | 0/10 | 0/10 | 0/10 | stable |
| actuator.deadband_nm | assumed | scale 0.2 .. 3 | 0/10 | 0/10 | 0/10 | stable |
| actuator.torque_cmd_resolution_nm | assumed | scale 0.25 .. 2 | 0/10 | 0/10 | 0/10 | stable |
| actuator.motor_time_constant_s | assumed | range 0.0005 .. 0.005 | 0/10 | 0/10 | 0/10 | stable |
| actuator.command_delay_cycles | assumed | range 0 .. 3 | 0/10 | 0/10 | 0/10 | stable |
| actuator.torque_slew_nm_s | assumed | scale 0.5 .. 2 | 0/10 | 0/10 | 0/10 | stable |
| actuator.knee_speed_rad_s | derived | scale 0.8 .. 1.25 | 0/10 | 0/10 | 0/10 | stable |
| actuator.zero_torque_speed_rad_s | derived | scale 0.8 .. 1.25 | 0/10 | 0/10 | 0/10 | stable |
| actuator.bus.r_source_ohm | assumed | range 0.05 .. 0.5 | 0/10 | 0/10 | 0/10 | stable |
| actuator.bus.v_min_v | assumed | range 36 .. 43.2 | 0/10 | 0/10 | 0/10 | stable |
| friction.coulomb_nm | derived | scale 0.5 .. 1.5 | 0/10 | 0/10 | 0/10 | stable |
| friction.viscous_nm_s_rad | assumed | scale 0.5 .. 2 | 0/10 | 0/10 | 0/10 | stable |
| model.link_mass_scale_std | assumed | range 0.02 .. 0.2 | 0/10 | 0/10 | 0/10 | stable |
| model.com_shift_std_m | assumed | range 0.002 .. 0.02 | 0/10 | 0/10 | 0/10 | stable |
| timing.overrun_prob | assumed | range 0.0001 .. 0.01 | 0/10 | 0/10 | 0/10 | stable |
| timing.overrun_cycles | assumed | range 1 .. 5 | 0/10 | 0/10 | 0/10 | stable |
| comm.packet_loss_prob | assumed | range 0.0001 .. 0.01 | 0/10 | 0/10 | 0/10 | stable |
| comm.latency_burst_prob | assumed | range 0.0001 .. 0.01 | 0/10 | 0/10 | 0/10 | stable |
| comm.burst_cycles | assumed | range 1 .. 10 | 0/10 | 0/10 | 0/10 | stable |
| transmission.damping_ratio | assumed | range 0.005 .. 0.05 | 0/10 | 0/10 | 0/10 | stable |
| transmission.error_amp_rad | assumed | range 0.00029 .. 0.0017 | 0/10 | 0/10 | 0/10 | stable |
| transmission.lost_motion_rad | assumed | range 0.00029 .. 0.0017 | 0/10 | 0/10 | 0/10 | stable |

## Causes

| group off | passed without | base passed | main |
|---|---|---|---|
| sensor | 0 | 0 | no |
| actuator | 0 | 0 | no |
| friction | 0 | 0 | no |
| model | 0 | 0 | no |
| timing | 0 | 0 | no |
| comm | 17 | 0 | yes |
| transmission | 0 | 0 | no |

Main cause: comm

Hold reasons: overspeed 1; state_timeout 99;

## Suggestions

- needs a position loop in the drive (algorithm drive_position)
- raise the servo rate or reduce packet loss
- no admissible gain in the sweep: widen the levels or change the law

## Versions

git: 6ae849e

| input | sha256 |
|---|---|
| /home/runner/work/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/config/hardware/budget_xarm7.yaml | 1fedba18d2beba7cd96c10bcc001c939f8d2a697477e3753559468e6b673d449 |
| /home/runner/work/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/config/imperfections/budget_xarm7.yaml | 12ed5edf10110806c50e5659ca321a152611335224f60c021a084a40a6d70941 |
| /home/runner/work/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/config/budget_xarm7.yaml | f99f4b7418e3f409ca42ff6d9102e14c79a50d90b4d39b5471f8620edb2479ba |
| /home/runner/work/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/config/tasks/reference_move.yaml | 601b56df0d0d9b8295c4ff6d25174bf48609400623cbb3edeb7690a01ed0daca |
