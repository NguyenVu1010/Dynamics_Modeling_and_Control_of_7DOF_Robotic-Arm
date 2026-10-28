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

## Model and fixed-joint merge

`Model<N>` holds `N` revolute joints. For each joint: `T_parent_joint`, axis, limits
`q_min, q_max, qd_max, tau_max`, `damping`; for each child link: mass `m`, centre of mass
`c` in the link frame, inertia `I_c` about the centre of mass in the link frame.

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
doubles printed with `%.17g`. Columns in this order:

```
t_ns,mode,
q1..q7, qd1..qd7, qref1..qref7, qdref1..qdref7, tau1..tau7,
px,py,pz, rx,ry,rz, pdx,pdy,pdz, rdx,rdy,rdz,
e1..e6, step_us
```

| Columns | Meaning |
| --- | --- |
| `t_ns` | `Timestamp` of the cycle |
| `mode` | one of `idle`, `tracking`, `holding`, `finished`, `fault` |
| `q*`, `qd*` | measured joint position (rad) and velocity (rad/s) |
| `qref*`, `qdref*` | CLIK reference `q_d`, `q̇_d` |
| `tau*` | commanded torque (N·m), after clamping |
| `px..pz`, `rx..rz` | position (m) and `so3_log(R_b_e)` (rad) of `T_b_e` |
| `pdx..pdz`, `rdx..rdz` | same for `T_b_d` |
| `e1..e6` | CLIK `task_error`, order `[v; ω]` |
| `step_us` | wall time of `ControlLoop::step()` (µs); written as 0 when `deterministic: true` so that runs compare byte for byte (timing comes from `cobot_bench timing`) |

### Summary CSV

One row per run of `cobot_bench montecarlo` / `sweep`:

```
run_id,param,level,seed,final_pos_err_m,final_rot_err_rad,rms_joint_err_rad,peak_tau_ratio,sat_time_s,held,passed
```

`held` and `passed` are `0`/`1`; `param` is `none` for Monte Carlo runs.
