# Waypoint trajectories: results

Run on `main` at c4e660d (after #33–#39), realistic profile, seed 7. The waypoint file is
`tests/e2e/waypoints/e2e_10.yaml`: four points from home, waypoint 1 stops, auto and user
durations mixed. Design: `docs/design/2026-10-29-waypoint-gui-design.md`; decision: ADR-018.

| Scenario | Runner | Final position | Final rotation | Joint RMS | Other | Peak τ/τ_max |
| --- | --- | --- | --- | --- | --- | --- |
| [E2E-10](e2e_10/report.md) | `cobot_bench run --waypoints` | 0.067 mm | 0.0095° | 2.0e-4 rad | stop waypoint error 0.110 mm | 0.77 |
| [E2E-11](e2e_11/report.md) | Gazebo, `FollowWaypoints` action through `send_waypoints.py` | 0.055 mm | 0.013° | 2.0e-4 rad | client exit 0 | 0.79 |
| E2E-11b | as E2E-11, goal canceled after 3 s | – | – | – | client exit 3 (canceled), last row `standby`, max \|q̇\| 0.016 rad/s over the last 0.5 s | in limits |

Thresholds: E2E-02 values (5 mm, 2°, 0.02 rad); E2E-10 also needs every stop waypoint within
5 mm. The stop waypoint error is the closest approach of the tip to the waypoint between its node
time and 0.2 s later.

## Notes

- Bench and Gazebo agree within 0.02 mm at the end of the path, as in the single-segment
  scenarios (E2E-02 0.060 mm, E2E-09 0.057 mm).
- E2E-11b rest threshold is 0.03 rad/s, not 0.01: the CSV `qd` is the measured velocity
  (quantized encoder, differentiated, 100 Hz filter) and has a standard deviation of
  0.0047 rad/s per joint at rest in Gazebo, already 0.013–0.017 rad/s peak in standby before any
  goal. 0.03 is about 6σ. Pending owner approval.
- Preview and run use the same function, inputs and model, so a path that previews green runs
  the same way. The preview and reset services are not yet covered by a Gazebo-level test.

## Reproduce

```bash
source tools/wsl/env.sh
source ~/projects/7dof_dynamic_control/venv/bin/activate
python tools/run_e2e.py --scenario tests/e2e/scenarios/e2e_10_waypoints.yaml \
  --bench ~/projects/7dof_dynamic_control/build/main/bench/cobot_bench --out ~/projects/7dof_dynamic_control/out/wp
python tools/run_e2e.py --all --tag gazebo --out ~/projects/7dof_dynamic_control/out/wp
```

Interactive: `ros2 launch cobot_sim sim.launch.py controller:=cobot_controller rviz:=true`
(README "Waypoint GUI").
