# Luật điều khiển: CLIK bậc hai trên tham chiếu

Cài đặt: `core/planning/src/clik.cpp` (`planning::ClikReference<N>`). Thiết kế mục 5.4, ADR-007.
Khung và tên gọi theo `docs/conventions.md`; mọi đại lượng dưới đây biểu diễn trong `b`, twist
và hàng Jacobian theo thứ tự `[v; ω]`. Phần computed torque (tầng điều khiển) thuộc feature
`control` và được bổ sung vào tệp này khi feature đó vào.

## Luật CLIK

Tầng tham chiếu không đọc trạng thái đo sau `reset()`. Với `q = q_ref`, `q̇ = q̇_ref` hiện tại và
điểm quỹ đạo `(T_b_d, ẋ_d, ẍ_d)`:

```
e      = pose_error(T_b_d, FK(q)) = [p_d - p(q); so3_log(R_d R(q)^T)]
ė      = ẋ_d - J(q) q̇
a_task = ẍ_d + K_d ė + K_p e - J̇(q, q̇) q̇
q̈      = J#_λ a_task + (I - J#_λ J) (-k_null ∇h(q) - k_damp q̇)
```

`K_p = diag(kp)`, `K_d = diag(kd)` (YAML `clik.kp`, `clik.kd`). Khi `J#_λ` là giả nghịch đảo đúng
(`λ = 0`) và `J` đủ hạng hàng, `J q̈ = a_task`, nên với `ė = ẋ_d - J q̇` và
`ë = ẍ_d - J q̈ - J̇ q̇` sai số thoả `ë + K_d ė + K_p e = 0` (xấp xỉ bậc nhất cho phần hướng,
vì đạo hàm của `so3_log` chỉ bằng `ω_d - ω` gần `e = 0`). Với `kp = 100`, `kd = 20` mỗi thành
phần là hệ bậc hai giảm chấn tới hạn, `ω_n = 10 rad/s`. Số hạng thứ hai nằm trong không gian
không của `J`, nên không đổi `J q̈` và không làm lệch tác vụ.

`J̇ q̇` lấy từ `kinematics::jdot_qdot` (lan truyền gia tốc với `q̈ = 0`, `docs/derivations/jacobian.md`).

## Tích phân và giới hạn khớp

Bước `step(x_d, dt)` tính `q̈` từ `q_ref, q̇_ref` hiện tại rồi tích phân semi-implicit Euler:

```
q̇_ref += q̈ dt
q_ref += q̇_ref dt
```

Sau đó mỗi khớp `i`: nếu `q_i ≥ q_max_i` thì `q_i = q_max_i` và `q̇_i = min(q̇_i, 0)`; nếu
`q_i ≤ q_min_i` thì `q_i = q_min_i` và `q̇_i = max(q̇_i, 0)`. Vận tốc đi ra khỏi giới hạn được giữ.
`task_error` lưu `e` của bước vừa tính (trước khi tích phân). `reset(q)` đặt `q_ref = clamp(q)`,
`q̇_ref = q̈_ref = 0`, `e = 0`.

## Giả nghịch đảo giảm chấn

```
J#_λ = J^T (J J^T + λ² I_6)^-1                 N ≥ 6, LDLT 6x6
     = (J^T J + λ² I_N)^-1 J^T                 N < 6, LDLT NxN
```

Hai dạng bằng nhau với mọi `λ > 0` (đẳng thức `J^T (J J^T + λ² I)^-1 = (J^T J + λ² I)^-1 J^T`).
Dạng thứ hai dùng cho các mô hình test ít khớp (tay phẳng 2 khớp có `J` 6x2 hạng 2): ở `λ = 0`
`J J^T` suy biến nhưng `J^T J` thì không.

`λ` theo số điều kiện `κ = σ_max / σ_min` trên `min(6, N)` giá trị kỳ dị của `J`
(`Eigen::JacobiSVD` trên ma trận cố định `6xN`, không cấp phát; `κ = ∞` khi `σ_min = 0`):

```
λ = 0                                              κ ≤ cond_threshold
λ = lambda_max · min(1, (κ - cond_threshold) / cond_threshold)   ngược lại
```

`λ` liên tục theo `κ`, tăng tuyến tính từ 0 tại `cond_threshold` tới `lambda_max` tại
`2 · cond_threshold` và giữ ở đó. Khi `λ > 0`, `‖J#_λ‖ ≤ 1 / (2λ)`, nên `q̈` bị chặn gần điểm kỳ dị
(test: `|q̈| ≤ 1e3` ở cấu hình `κ ≈ 5e4`). Cái giá là `I - J#_λ J = λ² (J^T J + λ² I)^-1` không còn
là phép chiếu đúng, nên số hạng không gian không gây một sai số tác vụ nhỏ, chỉ khi `κ` lớn.

## Hàm phạt giới hạn khớp

Với `q̄_i = (q_min_i + q_max_i) / 2` và `r_i = q_max_i - q_min_i`:

```
h(q)   = Σ_i ((q_i - q̄_i) / r_i)² / (2N)
∂h/∂q_i = (q_i - q̄_i) / (r_i² N)
```

`h ≥ 0`, bằng 0 tại giữa dải, mỗi khớp chạm giới hạn góp `1/(8N)`; chia cho `r_i²`
làm các khớp có dải khác nhau bị phạt cùng thang. Khớp không có giới hạn hữu hạn (khớp
continuous) có `∂h/∂q_i = 0`. Trong không gian không, `-k_null ∇h - k_damp q̇` là hệ bậc hai
kéo `q` về giữa dải với giảm chấn `k_damp`; với `k_null = 1`, `k_damp = 2` và gradient cỡ `1e-2`,
chuyển động chậm (khớp 1 từ 2.8 rad đi khoảng 7e-3 rad trong 5 s) và không vượt quá.

## Chỉnh gain computed torque (MVP)

Luật computed torque `τ = M(q)(q̈_ref + k_d ė + k_p e) + C q̇ + G` cho động học sai số
`ë + k_d ė + k_p e = 0` trên mô hình danh định. Khi có nhiễu mô-men `d` mà mô hình không biết
(ma sát, sai số độ phân giải lệnh, sai lệch hệ số mô-men), sai số xác lập của khớp `i` xấp xỉ

    e_i ≈ d_i / (M_ii k_p,i)

nên độ cứng thực tế là `M_ii k_p,i` (Nm/rad). Cổ tay có `M_ii` nhỏ, và với gain cũ của bản
Python (`k_p,7 = 20`, tức `ω = 4.5 rad/s`), ma sát Coulomb 0.24 Nm của profile realistic đẩy
khớp 7 lệch khoảng 0.5 rad RMS (E2E-02: 42 mm, 44°).

Gain mới đặt mọi khớp tắt dần tới hạn ở `ω = 80 rad/s`: `k_p = ω² = 6400`, `k_d = 2ω = 160`.
Độ cứng tăng khoảng 320 lần ở khớp 7. Vòng 1 kHz có trễ khoảng 3 chu kỳ (cảm biến 1, lệnh 1,
hằng số thời gian động cơ 1 ms). Ở tần số cắt cỡ `k_d = 160 rad/s`, trễ đó gây lệch pha khoảng
`0.003 · 160 = 0.48 rad`, tức 27°, vẫn còn biên pha.

| ω (rad/s) | E2E-02, seed 7 (vị trí / hướng / RMS khớp) |
|---|---|
| 4.5 (gain cũ, khớp 7) | 42 mm / 44° / 0.489 rad: trượt |
| 30 | 2.5° / 0.067 rad: trượt |
| 60 | 0.31 mm / 0.55° / 0.018 rad: đạt sát ngưỡng |
| **80** | **0.17 mm / 0.31° / 0.010 rad: đạt** |
| 100 | 0.11 mm / 0.19° / 0.007 rad: đạt, ít biên pha hơn |

Bảng trên đo khi URDF còn thiếu armature, nên `M_77` chỉ khoảng 1.8e-4 kg·m². Đó là lý do
thật khiến cổ tay nhạy với nhiễu tới mức cần `ω` cao. Sau khi thêm armature (`M_77` khoảng
0.23 kg·m², ADR-005), độ cứng cổ tay tăng khoảng 1300 lần, và E2E-02 với `ω = 80` còn 0.060 mm
/ 1.8e-4 rad (docs/results/realism/README.md). Gain vẫn giữ `ω = 80`: biên pha đã kiểm ở trên
không phụ thuộc quán tính, vì computed torque nhân `M(q)` vào luật điều khiển. Nếu muốn giảm
gain, chạy lại sweep để có số liệu.

## Bù ma sát

Ma sát khớp của plant (profile realistic) là Coulomb cộng nhớt, `τ_f = c ⊙ sign(q̇) + b ⊙ q̇`.
Computed torque không biết `τ_f`, nên theo mục trên nó là nhiễu `d` đẩy khớp lệch
`e_i ≈ d_i / (M_ii k_p,i)`, lớn nhất ở cổ tay. Bộ điều khiển cộng thêm feed-forward

    τ_ff = c ⊙ tanh(q̇_ref / ε) + b ⊙ q̇_ref
    τ = clamp(τ_CT + τ_ff, -τ_max, τ_max)

với `c = coulomb_nm`, `b = viscous_nm_s_rad`, `ε = velocity_eps_rad_s` trong
`cobot.friction_compensation` (`config/default.yaml`).

- Dùng vận tốc **tham chiếu** `q̇_ref` chứ không dùng `q̇` đo: nhiễu và lượng tử hóa cảm biến
  không đi qua `tanh` dốc, và khi khớp đứng yên quanh `q̇ ≈ 0` mô-men không đảo dấu liên tục
  (không chattering). Khi bám tốt `q̇ ≈ q̇_ref`, nên phần còn lại là sai số bám nhỏ.
- `tanh(q̇/ε)` thay `sign(q̇)`: liên tục, lẻ, `|tanh| ≤ 1`, bằng `±1` khi `|q̇| ≫ ε`. Với
  `ε = 0.01 rad/s`, `|q̇_ref| = 0.03 rad/s` đã cho 99.5 % lực Coulomb.
- Giá trị danh định trong `default.yaml` là 80 % ma sát của profile realistic: nhận dạng không
  bao giờ đúng tuyệt đối, và bù thiếu an toàn hơn bù thừa (bù thừa đổi dấu nhiễu, có thể gây
  dao động quanh điểm dừng). Phần 20 % còn lại vẫn do `k_p`, `k_d` xử lý.
- `config/default.yaml` bật bù (`enabled: true`). Bù chỉ được bật sau khi SimPlant đã áp ma sát
  Coulomb dạng dính-trượt ổn định (docs/derivations/sim_plant_friction.md); trước đó, cổ tay trong
  mô phỏng gần như không có ma sát Coulomb, nên bù vào sẽ thành nhiễu chứ không phải bù.
  Thiếu mục `friction_compensation` hoặc để trống (`friction_compensation:`) đều cho mặc định tắt.
- Chỉ áp dụng ở chế độ tracking; hold và fault giữ nguyên. Profile ideal không có ma sát nên
  các kịch bản ideal (E2E-01, E2E-07) dùng `tests/e2e/configs/ideal_world.yaml` với
  `enabled: false`; bù khi plant không có ma sát là tự đưa nhiễu vào.
- `q̇_ref` không hữu hạn cho `τ_ff = 0`; mô-men cuối luôn bị kẹp vào `τ_max` của URDF.
