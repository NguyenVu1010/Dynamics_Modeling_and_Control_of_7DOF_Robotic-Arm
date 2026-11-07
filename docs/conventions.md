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
make a custom one: [docs/guides/configuration.md](guides/configuration.md). On the Gazebo path the profile's
friction is written into each arm joint's `<dynamics friction damping>` (ADR-011).

Optional section `cobot.manual` (studio jog and move commands, ADR-022; a missing section or key
keeps the default): `joint_a_max_rad_s2` (7 values or one scalar broadcast, rad/s², default 5),
`tcp_v_max_m_s` (0.25), `tcp_w_max_rad_s` (1.0), `tcp_a_max_m_s2` (0.5), `tcp_alpha_max_rad_s2`
(2.0), all at 100 % speed and > 0 (each command's speed fraction in (0, 1] scales them; the joint
velocity limit is `joint_speed_fraction` times the URDF qd_max); `deadman_ms` (100, > 0: a jog not
refreshed for this long ramps to rest); `jog_margin_deg` (2, >= 0: jogs and `move_joint` stop
this far inside `safety.soft_margin_deg`); `joint_speed_fraction` (0.9, in (0, 1]: a joint jog or
`move_joint` at 100 % targets at most this fraction of qd_max, so the tracked |qd|, which
overshoots its target slightly, stays below the `overspeed` hold at qd_max). Errors name the full
key, such as `cobot.manual.deadman_ms`.

Optional section `cobot.control` (ADR-023) with the single optional key `algorithm`: the tracking
law, one of the names registered at build time (`computed_torque`, `pd_gravity`, `drive_position`; table
`kAlgorithms` in `cobot/control/algorithm.hpp`). A missing section or key selects
`computed_torque`; another value is `cobot.control.algorithm: unknown algorithm 'foo' (registered:
computed_torque, pd_gravity, drive_position)`. The hold law of the torque laws is the same (computed-torque
gains). Optional section `cobot.pd_gravity: {kp, kd}` holds the gains of PD plus gravity
compensation, `tau = J .* (kp .* (q_ref - q) + kd .* (qd_ref - qd)) + G(q)`: both keys required
in the section, > 0, 7 values or one scalar broadcast (a wrong count is `cobot.pd_gravity.kp:
expected a scalar or 7 values, got 6`, the wording of `coulomb_nm`), `kp` in 1/s² and `kd` in
1/s, scaled by the constant joint inertia `J = diag(M(home_q))` of the controller's model, so the same
`omega` (`kp = omega²`, `kd = 2 omega`) means the same bandwidth as computed torque. The section
is required when `algorithm` is `pd_gravity` (`cobot.pd_gravity: required when
cobot.control.algorithm is pd_gravity`) and, with another algorithm, parsed and checked but
unused, so one file switches law through an override.

Optional section `cobot.payload: {kg, offset_m, radius_m}` (ADR-023): the payload the controller
is told about, with the meaning and rules of `imperfections.model.payload_*` (`kg` >= 0 and
required in the section, `offset_m` 3 values in the tip frame, `radius_m` >= 0, a solid sphere). A
missing section is no payload. `pipeline::controller_model` merges it into the last link
(`imperfect::merge_payload`, the function the plant uses); `kg` 0 leaves the model untouched.

Algorithm `drive_position` (feature drive-loop, ADR-023): the host sends a position set point,
a velocity feed-forward and a torque feed-forward (`hal::DriveMode::kPosition`); a simulated
joint drive (`pipeline::SimDrive`, bench only: `cobot_bench serve` and `cobot_controller`
reject it with `cobot.control.algorithm drive_position is bench only (no drive model on this
host)`) closes the position loop on its motor encoder. Section `cobot.drive: {rate_hz,
omega_rad_s, zeta, velocity_filter_hz, torque_feedforward}`, every key required in it, required
when the algorithm is `drive_position` (`cobot.drive: required when cobot.control.algorithm is
drive_position`): `rate_hz` > 0 and an integer multiple of `control_rate_hz` (`cobot.drive.rate_hz:
must be an integer multiple of control_rate_hz (1000), got 2500`); `omega_rad_s` > 0, 7 values
or a scalar (position-loop bandwidth, `kp = J omega²` N·m/rad, `kd = 2 zeta J omega` N·m·s/rad,
`J = diag(M(home_q))` of the controller model); `zeta` > 0; `velocity_filter_hz` > 0 (low-pass
of the drive's encoder difference); `torque_feedforward` `none`, `gravity` (`G(q_ref)`) or
`inverse_dynamics` (`rnea(q_ref, qd_ref, qdd_ref) + D qd_ref`).

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
| `hal::DriveMode` | `std::uint8_t`: `kTorque = 0` (apply `tau`), `kPosition = 1` (set-point packet) |
| `hal::JointCommand<N>` | `{tau, mode, q, qd, seq}`: torque mode uses `tau`; position mode sends the set point `q` (rad, motor encoder in link units), the velocity feed-forward `qd` (rad/s) and the torque feed-forward `tau` (N·m); `seq` is the set-point number (`ControlLoop` counts 1, 2, 3, ... every step) |

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
| `tau*` | commanded torque (N·m), after clamping; in a `drive_position` run the torque feed-forward sent with the set point (`qref*` is then the set point before the deflection shift) |
| `px..pz`, `rx..rz` | position (m) and `so3_log(R_b_e)` (rad) of `T_b_e`, the forward kinematics of the logged `q*` (so of `q_hat` when the deflection compensation is active) |
| `pdx..pdz`, `rdx..rdz` | same for `T_b_d` |
| `e1..e6` | CLIK `task_error`, order `[v; ω]`; 0 in `holding` rows (CLIK is not stepped there, `qref*` is the hold target and `qdref*` is 0) |
| `step_us` | wall time of `ControlLoop::step()` (µs); written as 0 when `deterministic: true` so that runs compare byte for byte (timing comes from `cobot_bench timing`) |
| `hold` | why `ControlLoop` entered `holding`, kept in every row until a reset: `none` (never held), `non_finite` (a non-finite q or qd), `stamp_backwards` (state stamp older than the newest seen), `state_timeout` (no fresh stamp for more than `safety.hal_timeout_ms`), `joint_limit` (q within the soft margin of a limit), `overspeed` (\|qd\| above qd_max), `estop` (E-stop from a host, `ControlLoop::estop()`; a reset is needed); the first in this order wins when several hold at once |
| `qddref*` | CLIK reference acceleration `q̈_d` (rad/s²); 0 in `holding`, `fault` and `idle` rows, like `qdref*` |
| `qraw*` | measured joint position before the deflection compensation (equal to `q*` when it is inactive) |
| `qt*`, `qdt*` | plant link position (rad) and velocity (rad/s) at the row's instant, before the sensor decorator |
| `qddt*` | mean plant link acceleration (rad/s²) over the plant interval ending at `t_ns` (velocity difference over the plant clock; 0 in the first row) |
| `qmt*` | plant motor-side position in link units (rad); equal to `qt*` on a rigid joint |
| `taum*` | torque the plant applied (N·m) over the interval ending at `t_ns`, after the decorator, motor lag and the drive cap; in a `drive_position` run the torque of the last drive sub-step |
| `tauf*` | plant friction torque (N·m) over that interval: Coulomb + profile viscous (URDF `<dynamics damping>` excluded), opposing the motion; on an elastic joint the motor-side friction |
| `cmd` | command mode (`ControlLoop::command_mode()`): `config` in every row of `command_source: config`; in external mode the mode of the command running (`waypoints` for `load`, `move_tcp` for `load(..., kMoveTcp)`, `jog_joint`, `jog_tcp`, `move_joint`), `waypoints` before any command; `standby` and `holding` rows keep the mode of the last accepted command |

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
In a `drive_position` run the three torque metrics of the C++ summary (`cobot_bench run`,
`montecarlo`, `sweep`, `run_in_memory`) use the applied torque `taum` (the drive's PD output plus
the feed-forward, after the actuator chain), since `tau` is only the feed-forward sent with the
set point; `cobot_eval` on the Run CSV still reads `tau`.

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

### Hardware model file

`config/hardware/<model>.yaml` (`pipeline::load_hardware_model`, `cobot/pipeline/hardware_model.hpp`,
ADR-023): one piece of hardware, independent of the task.

```yaml
hardware:
  name: budget_xarm7
  description: "..."
  imperfections: ../imperfections/budget_xarm7.yaml   # relative to this file
  controller: ../budget_xarm7.yaml                    # base controller config, relative to this file
  limits: {rated_torque_nm: [7 values], peak_torque_nm: [7 values]}   # cobot.limits
  algorithms:
    computed_torque: {overrides: {config.computed_torque.omega: 20.0}}
    pd_gravity: {overrides: {config.pd_gravity.kp: 400.0, config.pd_gravity.kd: 40.0}}
  provenance:
    - {key: sensor.encoder_bits, kind: published}
    - {key: sensor.q_offset_std_rad, kind: assumed, range: [5.0e-4, 5.0e-3]}
    - {key: friction.coulomb_nm, kind: derived, scale: [0.5, 1.5]}
```

Every key is required. Names under `algorithms` are those of `cobot.control.algorithm`; an override
key is a `pipeline::KeyOverride` key and a plain `true`/`false`, a plain number and a list of numbers
keep their type, anything else is a string. `pipeline::config_for(model, algorithm)` is the base
controller file plus `config.control.algorithm`, the limits and that algorithm's overrides, through
`load_config_with`; an algorithm the model does not list is an error naming the model and the listed
ones. Errors name the file and the full key (`hardware.provenance[3].kind: ...`), and a model loads
only if its files and all its listed algorithms load.

Provenance rule (a model with an unsourced key does not load): every key of
`active_imperfection_keys(profile)` has exactly one entry, and no other key has one. A leaf of the
profile is active unless at its off value (0, every element of a list 0, false, or absent); `seed`,
`source` and `model.payload_*` are never listed; `sensor.qd_filter_hz` counts only with
`sensor.qd_from_diff` true, `actuator.ripple_periods_per_rev` only with a non-zero
`actuator.torque_ripple_frac`, and `sensor.position_source` when it is not `link`. `kind` is
`published`, `derived` or `assumed`. `derived` and `assumed` need exactly one of `range: [lo, hi]`
(absolute, for a key the profile writes as a scalar, lo <= the file's value <= hi) and `scale: [lo,
hi]` (multiplier of every element, for a key the profile writes as a list, 0 <= lo <= 1 <= hi);
`published` may omit both; bool and choice keys take neither, whatever the kind.

### Task file

`config/tasks/<task>.yaml` (`pipeline::load_task`, `cobot/pipeline/task.hpp`, ADR-023): what to
move, what is carried and what counts as good enough.

```yaml
task:
  name: reference_move
  trajectory: {goal_position_m: [x, y, z], goal_rpy_deg: [r, p, y], duration_s: 10.0, hold_s: 2.0}
  # or instead of trajectory: waypoints: <Waypoint file, relative to this file>
  payload: {kg: 0.0, offset_m: [0, 0, 0], radius_m: 0.0, known: true}
  requirements:
    final_pos_err_m: 0.005
    final_rot_err_deg: 2.0
    rms_joint_err_rad: 0.02
    cycle_time_s: 12.0
    max_hold_rate: 0.05
    target_pass_rate: 0.95    # optional, default 0.95
    margin_frac: 0.2          # optional, default 0.2
```

Exactly one of `trajectory` and `waypoints`; the waypoint file must exist, the run parses it.
`payload` is optional (0 kg, known); `kg` is required in it, `offset_m`, `radius_m` (0) and `known`
(true) are optional. Every requirement is > 0, the two rates are in (0, 1] and `margin_frac` in (0,
1). `cycle_time_s` is the settle time: from the first row until the TCP position and rotation errors
enter and stay within the final thresholds; the loader only stores it.

`pipeline::run_overrides(model, algorithm, task)` binds the three: `imperfections` gets
`model.payload_{kg,offset_m,radius_m}` (the plant always carries the payload) and `config` is the
whole list for `load_config_with(model.controller, ...)`: the model's overrides for the algorithm,
the trajectory keys, and `config.payload.*` when `known` is true.

### Studio bridge

The shared-memory link between a controller and `cobot_studio` (`cobot::bridge`, ADR-022,
`cobot/bridge/wire.hpp`, `endpoint.hpp`), Linux only. Region `/cobot_<name>` (name
`[A-Za-z0-9_]{1,64}`; `cobot_bench timing` uses `timing_<pid>`), about 3 MB for N = 7, reserved with
`posix_fallocate`. The server (controller) creates it and never unlinks it on destruction; a
region whose server pid is gone is replaced. The header starts with the magic `COTBRDG1`, then the
layout version (`kBridgeLayoutVersion`, bumped on any wire or header change), N, the sizes of
`StateSample<N>`, `Command<N>` and `Response`, and the three ring capacities; a client refuses a
mismatch naming the field and both values (`dof: region 2, client 7`).

| Part | Size | Writer -> reader |
| --- | --- | --- |
| state ring | 4096 `StateSample<N>` (~4 s at 1 kHz), seqlock per slot | control thread -> studio; overwrites, never waits; lost samples are counted by the reader |
| command ring | 64 `Command<N>` | studio (owner only) -> bridge thread |
| response ring | 16 `Response` | bridge thread -> studio (owner only); a full ring refuses and counts the response |
| jog slot | one `JogRequest`, seqlock | studio (owner only) -> control thread |
| session | one `SessionInfo<N>`, seqlock | server at start and after "apply config" (generation + 1) |

Units: SI and radians; times `*_ns` are `CLOCK_MONOTONIC` nanoseconds (`bridge::monotonic_ns`, the
same clock in every process), durations `*_s` seconds, `step_time_us` microseconds. `WirePose`
holds `p[3]` (m) and `R[9]` row-major (`R[3 r + c] = R(r, c)`); state poses are tip poses
(`T_b_e`, `T_b_d`), command and waypoint poses are TCP poses. Enum fields carry the integer value
of the pipeline enum (`mode`: `LoopMode`, `hold_reason`: `HoldReason`, `cmd_mode`: `CommandMode`,
`WireIssue::issue`: `PathIssue`); `q_true` is NaN when the host does not know the truth. Text
fields are 0-terminated (`kTextChars` 256, `kPathChars` 512); longer text ends with `...`.

Authority: one owner at a time. A client claims with its random non-zero token (compare-and-swap
from 0), then writes its pid and beats every 50 ms (`kHeartbeatPeriodNs`); the server sees the owner as
lost after 200 ms without a beat (`kHeartbeatTimeoutNs`) or when its pid is gone. Commands carry
the id given by `send` (1, 2, ...) and the owner's token; `refresh` of a jog write is the slot's
write count + 1, so it changes on every write, also when the owner changes.

Hosts (feature controller-bridge): `cobot_controller` with `log.mode: recorder_live` (region
`/cobot_<node name>`, launch argument `studio:=true`) and `cobot_bench serve` (region
`/cobot_<--name>`, default `cobot_bench`). The control thread applies at most one command per step
(`bridge::LoopPort`); the bridge thread (`bridge::CommandService::poll`, every 5 ms) checks and
answers. Every command gets exactly one `Response` (`command_id` = its id, `type` = its type):

| Command | Needs | Response |
| --- | --- | --- |
| any | token = current owner, else "not the owner" | |
| `kMoveJoint`, `kMoveTcp`, `kRunWaypoints`, `kApplyConfig` | `command_source: external` ("command_source is config"), loop in standby ("not in standby (mode tracking)") | |
| `kMoveJoint` | `plan_joint_move` from the loop's joint reference | on the step that applied it: `accepted`, `t_start_ns` (that step's stamp), `duration_s` |
| `kMoveTcp` | goal tip `T_b_tcp · inverse(T_tip_tcp)`, `plan_tcp_move` from the standby q and reference | rejected with `issues`, or as `kMoveJoint` |
| `kRunWaypoints` | TCP -> tip, `validate_path` from the standby q and reference (the panel's call) | rejected with `issues`, or as `kMoveJoint` |
| `kPreviewWaypoints` | the same call, nothing runs | `accepted` = ok, `issues`, `segment_durations_s`, `path` (desired TCP positions, evenly decimated to <= 512, last kept), `duration_s` |
| `kStop`, `kEstop`, `kReset` | any mode (the loop decides) | `accepted` or the loop's reason |
| `kApplyConfig` | `path_a` config, `path_b` imperfections; same `control_rate_hz` | "applied" and a new session (generation + 1), or the host's error verbatim |
| `kExportSegment` | `t0_ns`, `t1_ns` (stamps of the Run CSV) | `out_path`, `rows`, `truncated` |
| (jog slot) | refresh changed, token = owner | only a rejection: `type` `kJog`, `command_id` = the refresh, the loop's text |

Watchdog: an owner that is lost (heartbeat, pid), or replaced or released while the loop tracks a
motion the studio started (jog, `move_joint`, `move_tcp`, `run_waypoints`), gets one stop with
deceleration; a lost owner is cleared once that motion is over. A motion the studio did not start
(the config's trajectory, an RViz goal accepted before the claim) runs on, and a lost owner is then
cleared at once.
Segment export: the host first exports the Recorder to its Run CSV as usual, then the rows of the
Run CSV with `t_ns` in `[t0, t1]` and its header line are copied byte for byte to
`<segment dir>/segment-<id>.csv` (controller: `<tmp>/cobot_<node name>`; bench: `<out>/segments`);
`truncated` = the first copied row is later than `t0 + period` (older rows were overwritten before
any export). Times in commands and responses (`t_start_ns`, `t0_ns`, `t1_ns`) are the loop's row
stamps, not `monotonic_ns`.

`cobot_bridge_cli <name> [--speed <pct 1..100, default 10>] [--timeout-s <s, default 60>] <step>...`
(`ros2 run cobot_ros2 cobot_bridge_cli cobot_controller ...` in Gazebo) is the headless client:

| Step | Effect |
| --- | --- |
| `claim`, `release` | take / give back authority; a thread beats every 50 ms from `claim` on |
| `pause:<s>`, `wait-standby` | sleep; wait for a standby sample |
| `stop`, `estop`, `reset` | the command, its response |
| `jog-joint:<1..7>:<+\|->:<s>` | jog for s seconds (refresh every 20 ms), release, wait for standby |
| `jog-tcp:<x\|y\|z\|rx\|ry\|rz>:<+\|->:<base\|tool>:<s>` | the same for the TCP |
| `move-joint:<q1_deg,...,q7_deg>` | `kMoveJoint`, then standby |
| `move-tcp:<x,y,z,roll_deg,pitch_deg,yaw_deg>` | `kMoveTcp` to that TCP pose in base_link, R = Rz(yaw) Ry(pitch) Rx(roll), then standby |
| `waypoints:<file>[:<count>]`, `preview:<file>` | the first count points of a waypoint file (TCP poses as written) |
| `export:<csv>` | segment `[t_start, standby + 1 s]` of the last motion step, copied to csv |

It prints one line per step, `<k> <step>: ok [detail]`, `<k> <step>: rejected: <reason>`,
`<k> <step>: failed: <reason>` (a motion step that ends in holding fails with `held: <reason>`)
or `<k> <step>: stopped (<manual_stop>)` (a move or waypoints step that ends in standby with a
manual stop other than `finished`: a stop, or the owner's heartbeat lost), and exits 0 when every
step is ok, 1 when one is not (later steps skipped), 2 on bad arguments, no bridge or a layout
mismatch. Authority is released on every exit path: SIGINT or SIGTERM ends the running step with
`failed: interrupted` (exit 1). A CLI frozen past the heartbeat timeout (SIGSTOP, a stalled
host) hands authority back when it resumes instead of beating again, so the motion it started is
stopped.
