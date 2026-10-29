# E2E-02 report

Run: `run1.csv`, evaluated at t = 11.998 s. Result: **PASS**

| Metric | Value | Threshold | Result |
| --- | --- | --- | --- |
| final_pos_err_m | 0.000170349 | 0.005 | PASS |
| final_rot_err_deg | 0.306261 | 2 | PASS |
| rms_joint_err_rad | 0.0104533 | 0.02 | PASS |
| final_rot_err_rad | 0.00534526 | | |
| peak_tau_ratio | 0.6783 | | |
| sat_time_s | 0 | | |
| held | no | | |

## Joint tracking RMS (tracking and finished rows)

| Joint | RMS (rad) |
| --- | --- |
| 1 | 0.0010952 |
| 2 | 0.000426522 |
| 3 | 0.00193105 |
| 4 | 0.000901275 |
| 5 | 0.00332796 |
| 6 | 0.00599538 |
| 7 | 0.0104533 |

## Plots

![q](q.png)
![tracking error](tracking_error.png)
![tau](tau.png)
![task error](task_error.png)
