Screening and comparison tool, not a certification: it does not replace a safety assessment to ISO 10218 or ISO/TS 15066, nor measurements on the real robot.

# Verify comparison

| model | algorithm | verdict | passed / runs | Wilson 95 % | worst final_pos_err_m | worst final_rot_err_deg | worst rms_joint_err_rad | worst cycle_time_s | gain used | gain source |
|---|---|---|---|---|---|---|---|---|---|---|
| premium_fr3 | computed_torque | PASS | 100 / 100 | 0.963 .. 1 | 0.0001936 | 0.04006 | 0.0004608 | 9.119 | 50 | tuned |
| premium_fr3 | drive_position | not listed | - | - | - | - | - | - | - | - |
| mid_kr1205 | computed_torque | PASS | 100 / 100 | 0.963 .. 1 | 0.0008521 | 0.1023 | 0.001523 | 9.143 | 40 | tuned |
| mid_kr1205 | drive_position | not listed | - | - | - | - | - | - | - | - |
| budget_xarm7 | computed_torque | FAIL | 0 / 100 | 0 .. 0.03699 | - | - | - | - | 20 | default_no_admissible |
| budget_xarm7 | drive_position | MARGINAL, measure on hardware | 92 / 100 | 0.85 .. 0.9589 | 0.007505 | 0.6629 | 0.007414 | - | 50 | tuned |
