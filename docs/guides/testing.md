# Tests and results

How the project is tested ([ADR-010](../adr/010-testing.md),
[ADR-016](../adr/016-frozen-tests-and-cicd.md)) and the full table of measured results.

## Test levels

Unit and known-answer component tests cover every module (`tests/unit/`, `tests/component/`,
`tests/python/`). End-to-end scenarios live in `tests/e2e/scenarios/` and run through
`tools/run_e2e.py`:

| ID | Content | Threshold | Runs |
| --- | --- | --- | --- |
| E2E-01 | bench + SimPlant, profile `ideal`, MVP trajectory | 2 mm, 1°, 0.01 rad | every PR |
| E2E-02 | as E2E-01, profile `realistic`, fixed seed | 5 mm, 2°, 0.02 rad | every PR |
| E2E-03 | determinism: two runs with one seed give byte-identical CSV | equal | every PR |
| E2E-04 | safety: goal into a joint limit, NaN from HAL, HAL timeout | hold mode, no NaN out, torque in limits | every PR |
| E2E-05 | Monte Carlo, 50 seeds, `realistic` | ≥ 95 % pass E2E-02 thresholds | nightly, manual |
| E2E-06 | one-factor sensitivity sweep, report + hardware requirement table | report generated | nightly, manual |
| E2E-07 | headless Gazebo, real controller through `gz_ros2_control`, `ideal` | G3 | every PR |
| E2E-08 | arm64 (Pi 5) container under QEMU: build, unit, E2E-01..04 | pass | every PR |
| E2E-09 | headless Gazebo, real controller, `realistic`, seed 7 (friction in the physics) | 5 mm, 2°, 0.02 rad | every PR |
| E2E-10 | bench, 4 waypoints, `realistic` | 5 mm, 2°, 0.02 rad, stop waypoints ≤ 5 mm | every PR |
| E2E-11 | headless Gazebo, E2E-10 waypoints sent through `follow_waypoints`, `realistic`, seed 7 | 5 mm, 2°, 0.02 rad | every PR |
| E2E-11b | as E2E-11, goal canceled after 3 s | canceled, ends in standby, max \|qd\| < 0.03 rad/s over the last 0.5 s (measured velocity: encoder noise std 0.0047 rad/s at rest), torque in limits | every PR |
| E2E-12 | the E2E-01 trajectory on the three hardware profiles, seeds 1–3 | E2E-02 level (proposed) | tag `realism2`, manual |
| E2E-13 | headless Gazebo driven through the studio bridge: jog, joint move, TCP move, waypoints; RViz goal refused | E2E-02 level | every PR |
| PF-01 | `ControlLoop` step time | p50/p95 reported; p95 < 1 ms on CI, 200 µs measured on desktop | every PR |

## Results

MVP acceptance (G3): track a fixed 10 s straight-line trajectory; at t = 12 s end-effector
position error < 2 mm, orientation error < 1°, joint tracking RMS < 0.01 rad, desktop step
time p95 < 200 µs.

Current report: [docs/results/realism](../results/realism/README.md). Waypoint runs:
[docs/results/waypoints](../results/waypoints/README.md). Studio and bridge:
[docs/results/studio](../results/studio/README.md). The MVP baseline stays in
[docs/results/mvp](../results/mvp/README.md) for comparison.

| Check | Threshold | MVP | Current |
| --- | --- | --- | --- |
| E2E-01 bench, ideal | 2 mm / 1° / 0.01 rad | 2.6e-7 mm / 9.8e-7 rad | 6.7e-8 mm / 6.7e-7 rad |
| E2E-02 bench, realistic sensors, motors and friction | 5 mm / 2° / 0.02 rad | 0.17 mm / 0.31° / 0.0105 rad | 0.060 mm / 0.0075° / 1.8e-4 rad |
| E2E-05 Monte Carlo, 50 seeds realistic | ≥ 95 % pass | 96 % | 100 % |
| E2E-07 Gazebo Harmonic, ideal | 2 mm / 1° / 0.01 rad | 0.000 mm / 3.7e-6 rad | 0.000 mm / 4.8e-6 rad |
| E2E-09 Gazebo, realistic profile | 5 mm / 2° / 0.02 rad | – | 0.071 mm / 0.014° / 2.3e-4 rad |
| PF-01 control step, desktop | p95 < 200 µs | p95 4.9 µs | p95 5.3 µs |
| E2E-08 arm64 Pi5-like image (QEMU) | ctest + E2E-01..04 | pass in CI | pass in CI |
| E2E-10 bench, 4 waypoints, realistic | 5 mm / 2° / 0.02 rad, stops ≤ 5 mm | – | 0.067 mm / 0.0095° / 2.0e-4 rad, stops 0.11 mm |
| E2E-11 Gazebo, waypoints through the action | 5 mm / 2° / 0.02 rad | – | 0.055 mm / 0.013° / 2.0e-4 rad |
| E2E-11b Gazebo, cancel after 3 s | canceled, standby, at rest | – | pass, max \|q̇\| 0.016 rad/s |

The main gain came from modelling the gearbox-reflected rotor inertia (armature, `J_in N²`):
without it the wrist inertia was about 1300× too small and its effective stiffness
`M_ii k_p` about 1 N·m/rad. The MVP friction numbers were also optimistic, because the old plant
lost Coulomb friction numerically. Armature values are assumed until the eRob H-series datasheet
is confirmed (`src/sim/urdf/mass_budget.yaml`).

The sweep over 12 sensor, motor, model and timing parameters gives a
[hardware requirement table](../results/realism/hardware_requirements.md): sensor delay
≤ 4 cycles and torque constant error ≤ 20 % are the binding limits. Peak torque now reaches
89 % of `τ_max` in Monte Carlo, so faster trajectories need a torque check.
