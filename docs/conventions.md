# Math and frame conventions

Every module follows these. If a change needs something different, write an ADR first.

## Frames

| Symbol | Frame |
| --- | --- |
| `b` | `base_link`; equal to world because the robot is fixed to it |
| `i` | frame of link `i` (`i = 1..N`), origin at joint `i` |
| `e` | end effector = origin of `link_7` after fixed joints are merged |
| `d` | desired end-effector pose (`T_b_d`), produced by `planning` |

## Transforms

- `T_a_b` maps points from `b` to `a`: `p_a = T_a_b * p_b`.
- Composition must cancel inner frames: `T_b_e = T_b_i * T_i_e`. A product such as
  `T_b_i * T_e_i` is a bug.
- `R_a_b` is the rotation part, `p_a_b` the origin of `b` expressed in `a`.
- In code a transform is `cobot::common::Pose{R, p}`; `compose`, `inverse`, `transform`
  live in `cobot/common/types.hpp`.
- URDF `<origin>` of joint `i` is `T_parent_joint`: the joint frame in the parent link frame.
  The joint axis is a unit vector in the joint frame.

## Twists, Jacobians, errors

- Twist and Jacobian row order is `[v; ω]` (linear, angular), expressed in `b`.
- The geometric Jacobian 6×N of the end effector in `b` is `J_e_b` (`Mat6N<N>`).
- `J̇ q̇` is computed by acceleration propagation with `q̈ = 0` (`jdot_qdot`), never by
  numerical differentiation in the loop.
- Orientation error is `so3_log(R_d * R^T)` (axis-angle vector, rad, in `b`).
  `pose_error(T_b_d, T_b_e)` returns `[p_d − p; so3_log(R_d R^T)]`.
- `so3_exp`, `so3_log`, `pose_error` live in `cobot/common/lie.hpp`, so `planning` does
  not need `kinematics` for them.
- No Euler angles inside the control loop. RPY appears only in YAML (`goal_rpy_deg`) and is
  converted at load time.

## Quaternions

Only at I/O boundaries (ROS messages), Hamilton convention, order `(w, x, y, z)`. Write the
order in a comment at every construction site.

## Units and time

- SI everywhere. Angles in code are radians.
- Degrees only in YAML, and the key carries a `_deg` suffix (`home_q_deg`,
  `soft_margin_deg`); `pipeline` converts them when loading.
- Time in core is `Timestamp` = `std::int64_t` nanoseconds. `dt` is a double in seconds,
  obtained with `to_seconds`. Never compare floating-point times.
- Gravity is `[0, 0, -9.80665]` m/s² in `b` (`gravity_b` in YAML).

## Configuration files

`config/default.yaml` configures the controller (including friction compensation);
`config/imperfections/*.yaml` describe the environment (sensors, motors, friction, model
mismatch, timing, seed). Which file holds what, how to pick a profile for bench
(`--imperfections`) and Gazebo (`imperfections_path:=`, default `realistic.yaml`) and how to
make a custom one: README, section "Environment configuration". On the Gazebo path the profile's
friction is written into each arm joint's `<dynamics friction damping>` (ADR-011).

## Model and fixed-joint merge

`Model<N>` holds `N` revolute joints. For each joint: `T_parent_joint`, axis, limits
`q_min, q_max, qd_max, tau_max`, `damping`, `armature` (reflected rotor inertia `J_in * N^2`,
kg m², from the non-standard child element `<cobot_armature value="..."/>`, 0 when absent); for
each child link: mass `m`, centre of mass `c` in the link frame, inertia `I_c` about the centre
of mass in the link frame. Gazebo ignores `<cobot_armature>`, so its plant uses
`sim/urdf/cobot30_5_gazebo.urdf`, where each armature is folded into the child link's inertia
as a rotor disc about the joint axis (`tools/fix_urdf_inertia.py --gazebo-out`), and whose
joint `<dynamics>` come from the xacro args `joint_<i>_friction` / `joint_<i>_damping` (default
0; the URDF's nominal damping is not used there).

Fixed joints are merged into the parent link when the URDF is loaded:

1. Masses add: `m = m_parent + m_child`.
2. The combined centre of mass is the mass-weighted mean, both expressed in the parent frame.
3. Inertias are rotated into the parent frame and moved to the combined centre of mass with
   the parallel-axis (Steiner) theorem: `I += R I_c R^T + m (|d|² I₃ − d dᵀ)`.
4. The fixed joint's transform is composed into the `<origin>` of the next revolute joint.

Every link reached from a chain link through fixed joints is merged, including side branches
off the base → tip path and fixed children of the tip link. Links fixed before the first
revolute joint belong to the base and are dropped. Moving side branches are ignored.

This URDF has two fixed joints. `fixed` (world → `base_link`) is dropped. `joint_6_2_fix`
sits **in the middle of the chain** (`link_6 → link_6_2 → joint_7`): `link_6_2` is merged
into `link_6`, and the origin of `joint_7` becomes `T_link6_link62 * T_link62_joint7`.

## Core types (`cobot/common/types.hpp`)

| Type | Definition |
| --- | --- |
| `VecN<N>` | `Eigen::Matrix<double, N, 1>` |
| `MatN<N>` | `Eigen::Matrix<double, N, N>` |
| `Mat6N<N>` | `Eigen::Matrix<double, 6, N>` |
| `Vec3`, `Mat3`, `Vec6`, `Mat6` | fixed-size double |
| `Pose` | `{Mat3 R; Vec3 p;}` |
| `Timestamp` | `std::int64_t`, nanoseconds |

The project instantiates `N = 7` (`kArmDof`, `Arm7 = Model<7>`). Dynamic-size Eigen types
are not used in core.

## Data contracts

### Run CSV

Written by `pipeline::write_csv_header` / `write_csv_row`. Comma separated, one header row,
doubles printed with `%.17g`, NaN always written `nan` (never `-nan`). Rows are exported from the
host-owned `pipeline::Recorder` after the run (bench) or on each buffer swap (controller), see
ADR-020. Columns in this order (57 + 8 · 7 + 1 = 114 for N = 7):

```
t_ns,mode,
q1..q7, qd1..qd7, qref1..qref7, qdref1..qdref7, tau1..tau7,
px,py,pz, rx,ry,rz, pdx,pdy,pdz, rdx,rdy,rdz,
e1..e6, step_us, hold,
qddref1..qddref7, qraw1..qraw7, qt1..qt7, qdt1..qdt7, qddt1..qddt7, qmt1..qmt7,
taum1..taum7, tauf1..tauf7, cmd
```

`taum`, `tauf` and `qddt` of row k describe the plant interval (t_{k-1}, t_k]; `taum` of row k+1
is the plant's response to `tau` of row k. The plant truth columns (`qt` to `tauf`) are `nan` when
the host does not know them (a real robot). On the Gazebo path `qt`, `qdt` are the raw Gazebo
reading before the imperfection decorator, `taum` the last command written to Gazebo (the
decorator's output, motor lag included) and `qddt`, `qmt`, `tauf` are `nan` (friction is in the
physics engine). The legacy layout, the first 57 columns ending with `hold` (files written before
feature log-channels, such as those in `docs/results/`), is still accepted by `cobot_eval`
(`legacy_run_columns`).

| Columns | Meaning |
| --- | --- |
| `t_ns` | `Timestamp` of the cycle |
| `mode` | one of `idle`, `tracking`, `holding`, `finished`, `fault`, `standby` (external command mode: holding a fixed Cartesian reference through CLIK and computed torque, waiting for a trajectory; not counted as tracking error) |
| `q*`, `qd*` | the controller's working joint position (rad) and the measured velocity (rad/s). `q` is the raw reading, except when `cobot.deflection_comp.enabled` is true and the state comes from the motor encoder (`JointState::q_from_motor`): then it is `q_hat = q - deflection(g(q))`, the static spring deflection of the controller model's gravity torque removed. `qd` is always the raw reading |
| `qref*`, `qdref*` | CLIK reference `q_d`, `q̇_d` |
| `tau*` | commanded torque (N·m), after clamping |
| `px..pz`, `rx..rz` | position (m) and `so3_log(R_b_e)` (rad) of `T_b_e`, the forward kinematics of the logged `q*` (so of `q_hat` when the deflection compensation is active) |
| `pdx..pdz`, `rdx..rdz` | same for `T_b_d` |
| `e1..e6` | CLIK `task_error`, order `[v; ω]`; 0 in `holding` rows (CLIK is not stepped there, `qref*` is the hold target and `qdref*` is 0) |
| `step_us` | wall time of `ControlLoop::step()` (µs); written as 0 when `deterministic: true` so that runs compare byte for byte (timing comes from `cobot_bench timing`) |
| `hold` | why `ControlLoop` entered `holding`, kept in every row until a reset: `none` (never held), `non_finite` (a non-finite q or qd), `stamp_backwards` (state stamp older than the newest seen), `state_timeout` (no fresh stamp for more than `safety.hal_timeout_ms`), `joint_limit` (q within the soft margin of a limit), `overspeed` (\|qd\| above qd_max); the first in this order wins when several hold at once |
| `qddref*` | CLIK reference acceleration `q̈_d` (rad/s²); 0 in `holding`, `fault` and `idle` rows, like `qdref*` |
| `qraw*` | measured joint position before the deflection compensation (equal to `q*` when it is inactive) |
| `qt*`, `qdt*` | plant link position (rad) and velocity (rad/s) at the row's instant, before the sensor decorator |
| `qddt*` | mean plant link acceleration (rad/s²) over the plant interval ending at `t_ns` (velocity difference over the plant clock; 0 in the first row) |
| `qmt*` | plant motor-side position in link units (rad); equal to `qt*` on a rigid joint |
| `taum*` | torque the plant applied (N·m) over the interval ending at `t_ns`, after the decorator, motor lag and the drive cap |
| `tauf*` | plant friction torque (N·m) over that interval: Coulomb + profile viscous (URDF `<dynamics damping>` excluded), opposing the motion; on an elastic joint the motor-side friction |
| `cmd` | command mode: `config` (`command_source: config`) or `waypoints` (external); `jog_joint`, `jog_tcp`, `move_joint`, `move_tcp` are reserved |

Black-box log (ADR-020): the ROS controller records into the POSIX shared-memory region
`/cobot_log_<controller node name>` (`pipeline::ShmStorage`, Linux only), which outlives a crash
of the controller or of Gazebo; `cobot.log.lock_memory: true` mlocks it (on the Pi 5). Its service
`~/dump_log` (std_srvs/Trigger) appends the rows not yet exported to `log_path` (ROS parameter
`log_stream_ms` > 0: every that many ms); deactivation and cleanup export the rest and leave the
region in place. `cobot_logdump <controller> [--out <csv>] [--keep] [--remove]` (also
`ros2 run cobot_ros2 cobot_logdump`) exports the rows not yet exported to a Run CSV, by default
`./<controller>-<YYYYmmdd-HHMMSS>.csv`. At activation the controller writes the unexported rows of
an old region to `<log_path>.recovered-<YYYYmmdd-HHMMSS>.csv`, removes it and creates a new one.
Every row is exported exactly once, oldest first; once the buffer being written is full, its
oldest row (the slot the next record overwrites) is never exported.

### Summary CSV

One row per run of `cobot_bench montecarlo` / `sweep`:

```
run_id,param,level,seed,final_pos_err_m,final_rot_err_rad,rms_joint_err_rad,peak_tau_ratio,sat_time_s,rms_tau_ratio,held,hold_time_s,hold_reason,passed
```

`held` and `passed` are `0`/`1`; `param` is `none` for Monte Carlo runs. `hold_time_s` is `t` of
the first `holding` row minus `t` of the first row (s), empty when the run never held;
`hold_reason` is the `hold` value of that row, `none` when the run never held. A sweep `param`
named `config.<section>.<key>` (such as `config.safety.hal_timeout_ms`) replaces that scalar of
the controller config instead of an imperfection key; the virtual param
`config.computed_torque.omega` (rad/s, > 0) sets `kp = omega^2` and `kd = 2 omega` on every joint. `peak_tau_ratio` (max
`|tau| / P`) and `sat_time_s` (time with any `|tau| >= 0.999 P`) use the peak torque P of
`cobot.limits.peak_torque_nm`, the controller's `tau_max`; `rms_tau_ratio` is the max over joints
of `RMS(tau_j) / R_j` over the `tracking`/`finished` rows, R the rated torque
`cobot.limits.rated_torque_nm`. Without the `limits` section both P and R are the URDF effort.

### Studio file

A cobot_studio configuration (`studio::load_studio_file` / `save_studio_file`, ADR-021): a base
pair, typed key overrides with the key names of the YAML files (`config.<path under cobot:>` or
`<path under imperfections:>`, plus the virtual `config.computed_torque.omega`), and the studio's
own settings. Paths are relative to the file; `null` keeps the profile's seed, the configured
trajectory and its duration.

```yaml
studio:
  version: 1
  base: {name: premium_fr3, config: ../config/premium_fr3.yaml, imperfections: ../config/imperfections/premium_fr3.yaml}
  overrides:
    config.computed_torque.omega: 50
    sensor.q_noise_std_rad: 1.0e-4
    friction.coulomb_nm: [1.3, 1.3, 0.8, 0.8, 0.3, 0.3, 0.3]
  seed: 1
  tolerance: {goal_pos_mm: 1.0, goal_rot_deg: 0.5, path_pos_mm: 2.0, path_rot_deg: 1.0}
  speed_pct: 10
  waypoints: null
  duration_s: null
```

Each studio run writes `<work_dir>/<run-id>/{config.yaml, imperfections.yaml, studio.yaml}`: the
pair with the overrides applied (and the seed written into the profile), which
`cobot_bench run --config config.yaml --imperfections imperfections.yaml` reruns exactly.

### Waypoint file

A list of Cartesian waypoints for the external command mode, shared by the GUI, the bench and
the E2E scenarios (`pipeline::load_waypoints` / `save_waypoints`, design
`docs/design/2026-10-29-waypoint-gui-design.md` 6.6):

```yaml
frame: base_link            # must equal cobot.base_link of the config
waypoints:
  - {position_m: [-0.5, -0.6, 0.9], rpy_deg: [0, 0, 90], duration_s: 0, stop: false}
  - {position_m: [-0.3, -0.6, 0.7], rpy_deg: [0, 0, 90], duration_s: 4.0, stop: true}
```

| Key | Meaning |
| --- | --- |
| `position_m`, `rpy_deg` | TCP pose in `frame`; `R = Rz(yaw) Ry(pitch) Rx(roll)` |
| `duration_s` | optional, default 0: time from the previous point (s); 0 means computed from `cobot.cartesian_limits` |
| `stop` | optional, default `false`: come to rest at this point (the last point always stops) |

The poses are of the TCP; the loader returns tip (`link_7`) poses
`T_b_tip = T_b_tcp * T_tip_tcp^-1` with `T_tip_tcp` from `cobot.tcp_offset`, and the writer
stores `T_b_tcp = T_b_tip * T_tip_tcp`. Numbers are written with `%.17g`, so a saved list
loads back within 1e-9. Errors name the file and the key (`waypoints[2].rpy_deg: ...`).
