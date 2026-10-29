# Quỹ đạo Cartesian nhiều điểm

Cài đặt: `core/planning/src/waypoint_trajectory.cpp` (`planning::WaypointTrajectory`). Thiết kế
`docs/design/2026-10-29-waypoint-gui-design.md` mục 6.1. Khung và tên gọi theo
`docs/conventions.md`; twist theo thứ tự `[v; ω]`, biểu diễn trong `b`.

## Đoạn và nút

Nút 0 là tư thế bắt đầu `(R_0, p_0)`; nút `i + 1` là điểm `waypoints[i]`. Đoạn `i` đi từ nút
`i` tới nút `i + 1` trong thời lượng `T_i`, với `τ = (t - t_i) / T_i ∈ [0, 1)`. Thời điểm
`t = node_time_s(i)` thuộc đoạn sau, nên lấy mẫu tại đó trả đúng tư thế của điểm (không có sai
số làm tròn). Trước 0 giữ tư thế bắt đầu, từ `T` trở đi giữ điểm cuối, twist và đạo hàm bằng 0.

Mỗi đoạn lưu

```
Δp = p_{i+1} - p_i                       (m, trong b)
φ  = so3_log(R_i^T R_{i+1}),  |φ| ≤ π    (rad, trong khung đầu đoạn; luôn đi đường ngắn)
```

`|Δp|` hay `|φ|` không quá `1e-12` được coi bằng 0: điểm trùng nhau cho đoạn đứng yên.

## Hermite bậc 5

Cơ sở Hermite bậc 5 với đạo hàm bậc hai bằng 0 ở hai đầu:

```
s   = 10τ³ - 15τ⁴ + 6τ⁵        (0 → 1, đứng yên hai đầu)
h10 = τ - 6τ³ + 8τ⁴ - 3τ⁵      (tốc độ 1 ở đầu, 0 ở cuối)
h11 = -4τ³ + 7τ⁴ - 3τ⁵         (tốc độ 0 ở đầu, 1 ở cuối)
```

Vị trí, với `v_a`, `v_b` là vận tốc tại hai nút (trong b):

```
p(t) = p_i + s Δp + T (h10 v_a + h11 v_b)
ṗ    = (s'/T) Δp + h10' v_a + h11' v_b
p̈    = (s''/T²) Δp + (h10'' v_a + h11'' v_b) / T
```

(dấu phẩy là đạo hàm theo `τ`). Hướng dùng véc tơ quay so với đầu đoạn, với `u_a`, `u_b` là tốc
độ đổi của véc tơ quay ở hai đầu:

```
φ(t) = s φ + T (h10 u_a + h11 u_b),    R(t) = R_i exp(φ(t))
```

`φ(1) = φ` nên đoạn tới đúng `R_{i+1}`. Khi `v_a = v_b = u_a = u_b = 0` đây chính là
`CartesianTrajectory` (quintic điểm-điểm): `p = p_i + s Δp`, `R = R_i exp(s φ)`.

## Vận tốc góc và đạo hàm

Với `R = R_i exp(φ)`: `Ṙ R^T = [R_i J_l(φ) φ̇]×`, trong đó Jacobian trái

```
J_l(φ) = I + a [φ]× + b [φ]×²,   a = (1 - cos θ)/θ²,   b = (θ - sin θ)/θ³,   θ = |φ|
```

Do đó

```
ω = R_i (φ̇ + a φ×φ̇ + b φ×(φ×φ̇))
α = R_i (φ̈ + ȧ φ×φ̇ + a φ×φ̈ + ḃ φ×(φ×φ̇) + b (φ̇×(φ×φ̇) + φ×(φ×φ̈)))
```

với `ȧ = (a'(θ)/θ)(φ·φ̇)`, `ḃ = (b'(θ)/θ)(φ·φ̇)` và

```
a'(θ)/θ = (θ sin θ - 2(1 - cos θ)) / θ⁴        ≈ -1/12 + θ²/180 - θ⁴/6720
b'(θ)/θ = (3 sin θ - 2θ - θ cos θ) / θ⁵        ≈ -1/60 + θ²/1260 - θ⁴/60480
```

Dưới `θ = 0.1` rad dùng chuỗi Taylor (sai số cắt cỡ `1e-13`) để tránh triệt tiêu. Khi `φ̇ ∥ φ`
(mọi đoạn đứng yên hai đầu) các tích có hướng bằng 0 và `ω = R_i φ̇`, đúng như
`CartesianTrajectory`.

## Tốc độ tại nút

Tại điểm đầu, điểm `stop` và điểm cuối: `v = ω = 0`. Tại điểm lướt qua `k` giữa đoạn `i` (vào)
và `i + 1` (ra), kiểu Catmull-Rom, mỗi phần riêng:

```
v_k = (Δp_i + Δp_{i+1}) / (T_i + T_{i+1})                    (0 nếu một trong hai Δp = 0)
ω_k = (R_i φ_i + R_k φ_{i+1}) / (T_i + T_{i+1})              (0 nếu một trong hai φ = 0)
```

`R_i φ_i` là véc tơ quay của đoạn biểu diễn trong b. Đổi sang tốc độ của véc tơ quay:

```
u_b (cuối đoạn i)    = J_l(φ_i)^{-1} R_i^T ω_k
u_a (đầu đoạn i + 1) = R_k^T ω_k                             (J_l(0) = I)
J_l(φ)^{-1} y = y - ½ φ×y + c φ×(φ×y),   c = (1 - (θ/2) cot(θ/2)) / θ²  ≈ 1/12 + θ²/720 + θ⁴/30240
```

`J_l^{-1}` hữu hạn với mọi `θ ≤ π` (`cot(π/2) = 0`), nên đoạn gần 180° vẫn hữu hạn. Hai phía
nút cùng cho `ṗ = v_k` và `ω = ω_k`, nên quỹ đạo C1. Đạo hàm bậc hai của `p` và `φ` bằng 0 ở
nút; `α` có thể nhảy ở điểm lướt qua (số hạng `J̇_l φ̇`), nhưng tại nút có `φ̇ = 0` thì
`α = 0`, nên điểm đầu, điểm `stop` và điểm cuối đứng yên cả twist lẫn đạo hàm.

## Thời lượng tự tính

Đoạn không có thời lượng của người dùng bắt đầu với

```
T = max(0.1, 15/8 · |Δp|/v_max, √(10/√3 · |Δp|/a_max), 15/8 · θ/w_max, √(10/√3 · θ/α_max))
```

(đỉnh tốc độ và gia tốc của quintic đứng yên hai đầu). Vì tốc độ nút phụ thuộc thời lượng, sau
đó lặp: tính tốc độ nút, lấy mẫu mỗi đoạn tự tính trên lưới 257 điểm, tỉ số

```
r = max(|v|/v_max, √(|a|/a_max), |ω|/w_max, √(|α|/α_max))
```

và nếu `r > 0.999` thì `T ← T (r/0.999 + 1e-3)`. Kéo dài một đoạn chỉ làm giảm tốc độ nút (mẫu
số `T_i + T_{i+1}` tăng), nên vòng lặp dừng; giới hạn 200 vòng. Biên 0.1 % chừa cho đỉnh nằm giữa
hai điểm lưới (sai số lưới cỡ `1e-4`). Thời lượng người dùng nhập dùng nguyên, không kiểm giới
hạn ở đây (`validate_path` phán xét).
