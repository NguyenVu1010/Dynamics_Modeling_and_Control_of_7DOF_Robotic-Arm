# Architecture

How the repository is laid out, what one control cycle does, and how the project was built up
phase by phase. Decisions behind each part: [ADRs](../adr/).

## Layout

```
src/
  core/         C++17, no ROS; one target cobot::<module> per directory
    common/       VecN, MatN, Pose, Timestamp, Result, logger, so3_exp/so3_log
    model/        URDF -> Model<N> (fixed joints merged)
    kinematics/   FK, Jacobian J_e_b, Jdot*qdot
    dynamics/     RNEA, CRBA, G, C*qdot, forward dynamics, SimPlant
    planning/     quintic Cartesian trajectory, CLIK reference
    control/      computed torque, PD plus gravity, hold mode
    hal/          JointInterface<N>
    imperfect/    sensor/actuator/model imperfections
    pipeline/     ControlLoop, YAML config, SPSC ring, CSV
    bridge/       shared-memory studio bridge (ADR-022)
  ros2_node/    ros2_control controller plugin around ControlLoop
  ros2_interfaces/ cobot_interfaces: FollowWaypoints action, PreviewWaypoints service
  ros2_gui/     cobot_gui: RViz2 waypoint panel
  bench/        cobot_bench: ControlLoop + SimPlant, run / montecarlo / sweep / timing / serve
  studio/       cobot_studio_core and the Qt app (ADR-021)
  sim/          cobot_sim: URDF/xacro, meshes, world, controllers, launch
  eval/         Python reports and sensitivity analysis (reads CSV only)
config/       default.yaml, per-profile controller configs, imperfections/, sweeps/
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
| 0 | Repo setup: build, CI jobs, corrected URDF, Gazebo assets, interface headers, docs, ADRs | done (#1, #5) |
| 1 | `common-lie`, `model-urdf`, `pipeline-config`, `eval-report` | done (#2, #3, #4, #7) |
| 2 | `kinematics`, `planning-traj`, `imperfect-io` | done (#6, #8, #9) |
| 3 | `dynamics` (with KDL reference fixture), `planning-clik` | done (#10, #11) |
| 4 | `sim-plant`, `control` | done (#12, #13) |
| 5 | `control-loop`, `bench-e2e` | done (#14, #15) |
| 6 | `ros2-controller` (Gazebo), `sensitivity` | done (#16, #17) |
| 7 | `mvp-results`: tuning, reports, hardware requirement table | done |
| 8 | Realism: friction compensation, armature, stick-slip plant friction, Gazebo with noise | done (#23–#28) |
| 9 | Waypoints: multi-point trajectory, path validation, external command mode, `FollowWaypoints` action, RViz2 panel | done (#33–#39) |

Later work (realism-2 hardware profiles, `cobot_studio`, the studio bridge and manual control,
the tracking-law registry) is described in ADR-019 to ADR-023 and the design documents in
[docs/design](../design/).
