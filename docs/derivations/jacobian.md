# Forward kinematics, geometric Jacobian and Jdot*qdot

Implementation: `core/kinematics/src/kinematics.cpp`. Frames and names follow
`docs/conventions.md`; everything below is expressed in `b`.

## Forward kinematics

Joint `i` (`i = 0..N-1`) has the URDF origin `T_parent_joint_i` (joint frame in the parent
link frame at `q = 0`) and the unit axis `a_i` in the joint frame. With `T_b_{-1} = I`:

```
T_b_jnt_i = T_b_{i-1} * T_parent_joint_i          joint frame, before the rotation
T_b_i     = T_b_jnt_i * Rot(a_i, q_i)             link frame i
T_b_e     = T_b_{N-1} * T_last_tip
```

The rotation is about the joint origin, so the joint origin and axis in `b` are

```
p_i = p(T_b_jnt_i) = p(T_b_i)
z_i = R(T_b_jnt_i) a_i = R_b_i a_i                (a_i is invariant under Rot(a_i, .))
```

## Geometric Jacobian

A revolute joint `i` turning at `qd_i` moves the tip with linear velocity
`z_i qd_i x (p_e - p_i)` and angular velocity `z_i qd_i`. Column `i` of `J_e_b` is

```
J_i = [ z_i x (p_e - p_i) ; z_i ]
```

## Jdot*qdot

`Jdot qd` is the tip spatial acceleration `[a_e; alpha_e]` when `qdd = 0`. Differentiate each
column. Link `i` turns with

```
w_i = w_{i-1} + z_i qd_i,  w_{-1} = 0
```

and `z_i` is fixed in link `i` (also in link `i-1`), so `d/dt z_i = w_i x z_i = w_{i-1} x z_i`.
The joint origin `p_i` is fixed in link `i-1`, so its velocity follows

```
v_0 = 0,  v_i = v_{i-1} + w_{i-1} x (p_i - p_{i-1}),  v_e = v_{N-1} + w_{N-1} x (p_e - p_{N-1})
```

Then `d/dt (p_e - p_i) = v_e - v_i` and

```
Jdot qd = sum_i [ (zdot_i x (p_e - p_i) + z_i x (v_e - v_i)) qd_i ;  zdot_i qd_i ],
zdot_i  = w_i x z_i
```

Two O(N) passes (velocities, then the sum), fixed-size arrays only, no finite differences.
The angular part equals `sum_i w_{i-1} x z_i qd_i`, the derivative of `w_{N-1}` at `qdd = 0`.

Checks (`tests/unit/kinematics`): closed forms of a planar 2-link arm, and on the real arm
central differences of FK (for `J`) and of `J(q + qd t) qd` in `t` (for `Jdot qd`).
