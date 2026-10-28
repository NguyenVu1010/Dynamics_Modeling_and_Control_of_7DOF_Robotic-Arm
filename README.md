# Dynamics Modeling and Control of a 7-DOF Robotic Arm

![ci](https://github.com/NguyenVu1010/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/actions/workflows/ci.yml/badge.svg)

A final-year project: a self-written C++17 dynamics core (forward kinematics, geometric
Jacobian, RNEA, CRBA, forward dynamics, no Pinocchio or KDL) and a 1 kHz computed-torque
controller for a 7-DOF arm, read from a single URDF. The controller runs in Gazebo Harmonic
through `ros2_control` and in a ROS-free bench, a configurable imperfection layer (encoder
resolution, noise, delays, torque errors, friction, model mismatch) turns the simulation into a
sim-to-real sensitivity study, and the core is kept free of anything a Raspberry Pi 5 cannot
install with `apt`, which is the later hardware target.

![Gazebo](docs/images/gazebo.png)

## Layout

```
core/         C++17, no ROS; one target cobot::<module> per directory
  common/       VecN, MatN, Pose, Timestamp, Result, logger, so3_exp/so3_log
  model/        URDF -> Model<N> (fixed joints merged)
  kinematics/   FK, Jacobian J_e_b, Jdot*qdot
  dynamics/     RNEA, CRBA, G, C*qdot, forward dynamics, SimPlant
  planning/     quintic Cartesian trajectory, CLIK reference
  control/      computed torque, hold mode
  hal/          JointInterface<N>
  imperfect/    sensor/actuator/model imperfections
  pipeline/     ControlLoop, YAML config, SPSC ring, CSV
ros2_node/    ros2_control controller plugin around ControlLoop
bench/        cobot_bench: ControlLoop + SimPlant, run / montecarlo / sweep / timing
sim/          cobot_sim: URDF/xacro, meshes, world, controllers, launch
eval/         Python reports and sensitivity analysis (reads CSV only)
config/       default.yaml, imperfections/, sweeps/
docker/       Pi 5 (arm64) and ROS 2 + Gazebo images
tests/        unit/, component/, e2e/scenarios/, python/, data/
tools/        wsl/ scripts, ci/ checks, URDF mass fix, E2E runner
```

## Control data flow

One 1 kHz cycle (`ControlLoop::step()`), two tiers:

```
                     reference tier (planning)                 control tier (control)
                  +------------------------------+          +--------------------------+
 goal pose ---->  | CartesianTrajectory (quintic)|  x_d     | ComputedTorque           |
 (YAML)           |  x_d, xd_d, xdd_d            | -------> |  tau = M(q)(qdd_d        |
                  | ClikReference (2nd order,    |  q_d,    |   + Kd(qd_d - qd)        |
                  |  DLS, joint-limit null space)|  qd_d,   |   + Kp(q_d - q))         |
                  |  runs on q_d, not on q       |  qdd_d   |   + C qd + G + D qd      |
                  +------------------------------+ -------> |  clamp tau_max; hold mode|
                                                            +------------+-------------+
                                                                 tau     |   ^ q, qd
                                                                         v   |
                              +------------------------------------------------------+
                              | ImperfectJointInterface<N>  (decorator, optional)     |
                              |   read: quantise, noise, offset, delay               |
                              |   write: torque scale, ripple, resolution, lag, delay|
                              +------------------------------------------------------+
                              | hal::JointInterface<N>                               |
                              |   SimJointInterface (SimPlant)  |  gz_ros2_control   |
                              +------------------------------------------------------+
```

The core creates no threads; the host (`cobot_bench` or `controller_manager`) owns the control
thread and drains the log ring into a CSV file.

## Status

| Phase | Content | Status |
| --- | --- | --- |
| 0 | Repo setup: build, CI jobs, corrected URDF, Gazebo assets, interface headers, docs, ADRs | in progress |
| 1 | `common-lie`, `model-urdf`, `pipeline-config`, `eval-report` | planned |
| 2 | `kinematics`, `planning-traj`, `imperfect-io` | planned |
| 3 | `dynamics` (with KDL reference fixture), `planning-clik` | planned |
| 4 | `sim-plant`, `control` | planned |
| 5 | `control-loop`, `bench-e2e` | planned |
| 6 | `ros2-controller` (Gazebo), `sensitivity` | planned |
| 7 | `mvp-results`: tuning, reports, hardware requirement table | planned |

MVP acceptance (G3): track a fixed 10 s straight-line trajectory; at t = 12 s end-effector
position error < 2 mm, orientation error < 1°, joint tracking RMS < 0.01 rad, desktop step
time p95 < 200 µs.

## Build

All C++ builds run in WSL Ubuntu-24.04 (ROS 2 Jazzy and Gazebo Harmonic installed). From
PowerShell: `wsl -d Ubuntu-24.04 -- bash <repo>/tools/wsl/<script>.sh`.

```bash
tools/wsl/setup.sh                 # prints the apt line to type, creates .venv
tools/wsl/build.sh                 # CMake + Ninja, -Werror, ctest
tools/wsl/build.sh --asan          # Debug with ASan + UBSan
tools/wsl/build.sh --target T      # one target
tools/wsl/pytest.sh                # Python tests
tools/wsl/lint.sh                  # clang-format, clang-tidy, ruff
tools/wsl/sim.sh build             # colcon build of sim/
tools/wsl/sim.sh check             # xacro + check_urdf + gz sdf, no Gazebo needed
tools/wsl/sim.sh smoke             # headless Gazebo, 20 s
```

## Test levels

Unit and known-answer component tests cover every module. End-to-end scenarios live in
`tests/e2e/scenarios/` and run through `tools/run_e2e.py`:

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
| PF-01 | `ControlLoop` step time | p50/p95 reported; p95 < 1 ms on CI, 200 µs measured on desktop | every PR |

## Documentation

- [Design](docs/design/) — the full design (Vietnamese)
- [ADRs](docs/adr/) — architecture decisions and rejected options (Vietnamese)
- [Conventions](docs/conventions.md) — frames, units, data contracts
- [Contributing](CONTRIBUTING.md) — rules for every change

The previous ROS 1 / Python implementation (six nodes, Maple closed-form M, C, G) is kept at
tag [`v0-python`](https://github.com/NguyenVu1010/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/tree/v0-python).

## Tóm tắt

Đồ án tốt nghiệp: lõi động lực học C++17 tự viết (FK, Jacobian, RNEA, CRBA) và bộ điều khiển
computed torque 1 kHz cho tay máy 7 bậc, mô hình đọc từ một file URDF duy nhất. Điều khiển hai
tầng: quỹ đạo Cartesian → tham chiếu khớp bằng CLIK → computed torque → HAL. Chạy trong Gazebo
Harmonic qua `ros2_control` và trong bench không ROS; lớp mô hình không hoàn hảo (nhiễu cảm biến,
trễ, sai số động cơ, sai lệch mô hình) dùng để đánh giá độ nhạy khi chuyển sang robot thật.
Đích phần cứng sau này là Raspberry Pi 5. Thiết kế và ADR viết bằng tiếng Việt trong `docs/`.

## License

MIT, see [LICENSE](LICENSE).
