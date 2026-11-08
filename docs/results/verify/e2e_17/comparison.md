Screening and comparison tool, not a certification: it does not replace a safety assessment to ISO 10218 or ISO/TS 15066, nor measurements on the real robot.

# Verify comparison

| model | algorithm | verdict | passed / runs | Wilson 95 % | worst final_pos_err_m | worst final_rot_err_deg | worst rms_joint_err_rad | worst cycle_time_s | gain used | gain source |
|---|---|---|---|---|---|---|---|---|---|---|
| premium_fr3 | computed_torque | PASS | 100 / 100 | 0.963 .. 1 | 0.0001944 | 0.03774 | 0.0004729 | 9.119 | 50 | tuned |
| premium_fr3 | drive_position | not listed | - | - | - | - | - | - | - | - |
| mid_kr1205 | computed_torque | PASS | 100 / 100 | 0.963 .. 1 | 0.0008838 | 0.1064 | 0.001701 | 9.144 | 40 | tuned |
| mid_kr1205 | drive_position | not listed | - | - | - | - | - | - | - | - |
| budget_xarm7 | computed_torque | FAIL | 0 / 100 | 0 .. 0.03699 | - | - | - | - | 20 | default_no_admissible |
| budget_xarm7 | drive_position | MARGINAL, measure on hardware | 92 / 100 | 0.85 .. 0.9589 | 0.006135 | 0.59 | 0.008693 | - | 50 | tuned |
