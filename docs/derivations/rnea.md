# Động lực học ngược: Newton–Euler đệ quy (RNEA)

Cài đặt: `core/dynamics/src/rnea.cpp` (`rnea`), `core/dynamics/src/dynamics.cpp`
(`gravity_torque`, `coriolis_torque`, `forward_dynamics`). Ký hiệu khung theo
`docs/conventions.md`; mọi vector dưới đây biểu diễn trong `b` (kiểu Craig/Siciliano, vector
3D, không dùng đại số không gian 6D, xem ADR-004).

## Dữ liệu vào

Động học thuận (`docs/derivations/jacobian.md`) cho với mỗi khớp `i = 0..N-1`:
gốc khớp `p_i`, trục đơn vị `z_i`, và hướng link `R_i = R_b_i`. Tham số quán tính của link `i`
(đã gộp các link nối bằng khớp cố định, xem `core/model`) là khối lượng `m_i`, tâm khối `com_i`
và tensor quán tính quanh tâm khối `I_ci`, cả hai trong khung `i`. Trong `b`:

```
r_i  = R_i com_i                 c_i - p_i, from joint origin i to the CoM of link i
I_i  = R_i I_ci R_i^T            inertia about the CoM, axes of b
```

## Lượt đi (gốc -> ngọn)

Trọng lực được đưa vào như gia tốc của đế `a_{-1} = -g` (g là `gravity_b`), nhờ đó không cần
cộng `m_i g` vào từng link. Với `w_{-1} = w_dot_{-1} = 0`:

```
a_i     = a_{i-1} + w_dot_{i-1} x (p_i - p_{i-1}) + w_{i-1} x (w_{i-1} x (p_i - p_{i-1}))
          (a_0 = -g: joint origin 0 is fixed in b)
w_dot_i = w_dot_{i-1} + z_i qdd_i + w_{i-1} x z_i qd_i
w_i     = w_{i-1} + z_i qd_i
a_ci    = a_i + w_dot_i x r_i + w_i x (w_i x r_i)
F_i     = m_i a_ci
N_i     = I_i w_dot_i + w_i x (I_i w_i)
```

`p_i` cố định trong link `i-1`, nên gia tốc của nó dùng `w_{i-1}, w_dot_{i-1}`. Đạo hàm của
`z_i qd_i` là `z_i qdd_i + (w_{i-1} x z_i) qd_i` vì `z_i` cố định trong link `i-1`
(viết `w_i x z_i` cũng như nhau vì `z_i x z_i = 0`).

## Lượt về (ngọn -> gốc)

`f_i`, `n_i` là lực và mômen (quanh `p_i`) link `i-1` tác dụng lên link `i`. Với
`f_N = n_N = 0`:

```
f_i   = f_{i+1} + F_i
n_i   = n_{i+1} + (p_{i+1} - p_i) x f_{i+1} + N_i + r_i x F_i
tau_i = z_i . n_i
```

Ma sát nhớt của khớp (`damping` trong URDF) **không** nằm trong `rnea`; nó chỉ được trừ trong
`forward_dynamics`.

## Các trường hợp đặc biệt

```
G(q)        = rnea(q, 0, 0, g)
C(q, qd) qd = rnea(q, qd, 0, 0)
qdd         = M(q)^-1 (tau - rnea(q, qd, 0, g) - D qd)      LDLT, D = diag(damping)
```

`forward_dynamics` gọi `rnea` một lần với `qdd = 0` để có `C qd + G`, rồi `mass_matrix` (CRBA,
`crba.md`) và giải bằng `Eigen::LDLT` cỡ cố định. Không cấp phát heap: mọi mảng trung gian là
`std::array<Vec3, N>` trên stack.

## Kiểm chứng

`tests/unit/dynamics/`: con lắc đơn và tay máy phẳng 2 khâu (Spong) theo công thức đóng,
`rnea = M qdd + C qd + G`, `Mdot - 2C` phản đối xứng, bảo toàn động năng, và fixture KDL
`tests/data/reference_dynamics_kdl.csv`.
