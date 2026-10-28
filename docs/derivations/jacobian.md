# Động học thuận, Jacobian hình học và Jdot*qdot

Cài đặt: `core/kinematics/src/kinematics.cpp`. Khung và tên gọi theo `docs/conventions.md`;
mọi đại lượng dưới đây biểu diễn trong `b`.

## Mô hình và chỗ gộp khớp cố định

`parse_urdf_chain` (`core/model`) chỉ giữ các khớp quay, và `make_model` chép chuỗi đó sang
`Model<N>`: `model.joints` gồm đúng `N` khớp quay theo thứ tự base -> tip. Mỗi khớp cố định được
gộp vào khung lân cận: `<origin>` của nó được nhân vào
`T_parent_joint` của khớp quay kế tiếp, còn các khớp cố định sau khớp quay cuối cùng được nhân
vào `T_last_tip`. Vì vậy `e` là gốc của `link_7` sau khi gộp khớp cố định, và mọi phép đệ quy
dưới đây chỉ chạy qua `N` khớp quay.

## Động học thuận

Khớp `i` (`i = 0..N-1`) có gốc URDF `T_parent_joint_i` (khung khớp trong khung link cha tại
`q = 0`) và trục đơn vị `a_i` trong khung khớp. Với `T_b_{-1} = I`:

```
T_b_jnt_i = T_b_{i-1} * T_parent_joint_i          joint frame, before the rotation
T_b_i     = T_b_jnt_i * Rot(a_i, q_i)             link frame i
T_b_e     = T_b_{N-1} * T_last_tip
```

Phép quay đi qua gốc khớp, nên gốc khớp và trục khớp trong `b` là

```
p_i = p(T_b_jnt_i) = p(T_b_i)
z_i = R(T_b_jnt_i) a_i = R_b_i a_i                (a_i is invariant under Rot(a_i, .))
```

## Jacobian hình học

Khớp quay `i` quay với tốc độ `qd_i` làm đầu công cụ có vận tốc dài
`z_i qd_i x (p_e - p_i)` và vận tốc góc `z_i qd_i`. Cột `i` của `J_e_b` là

```
J_i = [ z_i x (p_e - p_i) ; z_i ]
```

## Jdot*qdot

`Jdot qd` là gia tốc không gian của đầu công cụ `[a_e; alpha_e]` khi `qdd = 0`. Lấy đạo hàm
từng cột. Link `i` quay với

```
w_i = w_{i-1} + z_i qd_i,  w_{-1} = 0
```

và `z_i` cố định trong link `i` (cũng cố định trong link `i-1`), nên
`d/dt z_i = w_i x z_i = w_{i-1} x z_i`. Gốc khớp `p_i` cố định trong link `i-1`, nên vận tốc
của nó là

```
v_0 = 0,  v_i = v_{i-1} + w_{i-1} x (p_i - p_{i-1}),  v_e = v_{N-1} + w_{N-1} x (p_e - p_{N-1})
```

Khi đó `d/dt (p_e - p_i) = v_e - v_i` và

```
Jdot qd = sum_i [ (zdot_i x (p_e - p_i) + z_i x (v_e - v_i)) qd_i ;  zdot_i qd_i ],
zdot_i  = w_i x z_i
```

Hai lượt O(N) (vận tốc, rồi tổng), chỉ dùng mảng kích thước cố định, không dùng sai phân số.
Phần góc bằng `sum_i w_{i-1} x z_i qd_i`, chính là đạo hàm của `w_{N-1}` khi `qdd = 0`.

Kiểm chứng (`tests/unit/kinematics`): biểu thức đóng của tay máy phẳng 2 khâu, và trên tay máy
thật là sai phân trung tâm của FK (cho `J`) và của `J(q + qd t) qd` theo `t` (cho `Jdot qd`).
