# Ma sát trong SimPlant: chiếu dính-trượt sau bước RK4

Cài đặt: `core/dynamics/src/sim_plant.cpp` (`apply_friction`). Bước tích phân `h` (1 ms),
`M(q)` từ CRBA (`crba.md`), `c_i` ma sát Coulomb, `b_i` ma sát nhớt (PlantEffects cộng
damping của URDF) của khớp `i`.

## Vì sao không để Coulomb trong RK4

Mô hình cũ đưa `c ⊙ tanh(q̇ / 1e-3)` vào vế phải của RK4. Quanh `q̇ = 0` đó là một ma sát
nhớt có hệ số `c / 1e-3`, và RK4 tường minh chỉ ổn định khi `h · (c / 1e-3) / I ≲ 2.8`.
Với cổ tay (khớp 7: `c = 0.24 N m`, quán tính hiệu dụng `≈ 1.8e-4 kg m²`):

```
h · c / (1e-3 · I) = 1e-3 · 240 / 1.8e-4 ≈ 1300
```

nên vận tốc dao động quanh 0, mô-men Coulomb trung bình gần bằng 0 và khớp gần như không có
ma sát khô. Khớp 5-6 cũng vượt ngưỡng (`≈ 24`). Làm `tanh` mềm hơn thì khớp bò (creep) dưới
ngưỡng; làm cứng hơn thì càng mất ổn định. Không có độ rộng nào đúng cả hai.

## Mô hình mới

1. RK4 tích phân động lực học vật rắn với `τ_applied`, không có ma sát của các khớp có
   `c_i > 0` (damping URDF của các khớp đó được triệt tiêu trong vế phải). Khớp không có
   Coulomb giữ ma sát nhớt trong RK4 như cũ.
2. Sau bước, ở `q⁺`, tính `M(q⁺)` và `W = M⁻¹` (LDLT). Ma sát của khớp `i` trong cả bước là
   một xung khớp `-λ_i e_i`. Mọi xung cùng đổi vận tốc: `q̇' = q̇ - W λ` (áp qua cả cột của
   `W`, không chỉ sửa `q̇_i`). Với từng khớp có `c_i > 0`:

   ```
   q̇'_i ≠ 0  ->  λ_i = h (c_i sign(q̇'_i) + b_i q̇'_i)
   q̇'_i = 0  ->  |λ_i| ≤ h c_i                         (ma sát tĩnh)
   ```

   Đây là một bài toán dính-trượt liên kết (các khớp ảnh hưởng nhau qua `W_ik`). Giải bằng
   Gauss-Seidel chiếu trên `λ` (khởi động từ `λ` của bước trước, tối đa 30 lượt): mỗi khớp giải
   bài toán một chiều của nó, coi xung các khớp khác là cố định. Gọi `v` là vận tốc khớp `i`
   khi bỏ xung của chính nó, `m_i = 1 / W_ii`, `Δv_i = h c_i / m_i`:

   ```
   |v| ≤ Δv_i  ->  v' = 0                              (dính)
   |v| > Δv_i  ->  v' = (v - sign(v) Δv_i) / (1 + h b_i / m_i)     (trượt, cùng dấu với v)
   ```

   rồi cập nhật vận tốc mọi khớp qua cột `i` của `W`. Các lượt chỉ dùng để tìm tập khớp dính
   và dấu của khớp trượt. Khi đã có tập đó, `λ` thỏa một hệ tuyến tính:

   ```
   khớp dính i:     (W λ)_i = q̇_RK4,i
   khớp trượt i:    λ_i + h b_i (W λ)_i = h (c_i s_i + b_i q̇_RK4,i)
   khớp không Coulomb:  λ_i = 0
   ```

   Hệ này được giải chính xác (LU). Nếu nghiệm khớp với tập (`|λ_i| ≤ h c_i` cho khớp dính,
   `q̇'_i` đúng dấu `s_i` cho khớp trượt) thì dùng nó; nếu không (hết số lượt giữa lúc chuyển
   trạng thái) thì giữ kết quả Gauss-Seidel. Khớp dính được đặt `q̇'_i = 0` đúng bằng 0.

3. Vị trí theo hiệu chỉnh tăng dần, cho mọi khớp: `q += h/2 · (q̇_mới - q̇_RK4)`.
   Khi trượt, vị trí giữ độ chính xác của RK4. Khi dính từ trạng thái nghỉ với gia tốc hằng,
   RK4 cho `q_RK4 = q + h² a / 2`, `q̇_RK4 = h a`, nên hiệu chỉnh trả `q` về đúng chỗ cũ.

## Ổn định và các tính chất

- Phép chiếu không bao giờ đảo dấu vận tốc và `|v'| ≤ |v|` với mọi `h`, `c_i`, `b_i`, `m_i`:
  ổn định vô điều kiện. Ma sát nhớt ở dạng ẩn (Euler lùi), hệ số `1 / (1 + h b / m) ∈ (0, 1]`.
- Với nhiều khớp, nghiệm liên kết có `λ_i q̇'_i ≥ 0` (xung cùng dấu với vận tốc sau), nên
  `KE(q̇) = KE(q̇') + λᵀ q̇' + ½ λᵀ W λ ≥ KE(q̇')`: động năng không bao giờ tăng, với mọi `h`.
- Dính thật, cả khi nhiều khớp liên kết: nếu `τ` giữ được mọi khớp dưới ngưỡng thì nghiệm
  "tất cả dính" thỏa bài toán, hệ tuyến tính trả về đúng nó, và mọi khớp có `q̇' = 0` chính
  xác; vị trí chỉ lệch ở mức làm tròn. Đo trên tay máy 7 bậc (gravity 0, tư thế home, ma sát
  thực tế ở mọi khớp, `τ` bằng 30/60/90 % ngưỡng): `q̇ = 0` ở cả 2000 bước, `|q - q0| < 1e-8`
  rad sau 2 s. Lượt Gauss-Seidel đơn lẻ (phiên bản trước) cho khớp 1-6 bò 0.013-0.038 rad,
  vì xung của khớp sau làm khớp đã dính trước đó chạy lại. Nếu để ma sát nhớt trong RK4, nó
  tác dụng lên vận tốc trung gian `≈ h τ / m` mà phép chiếu chỉ xóa sau đó, và vị trí bò
  `≈ h² / 12 · (b / m) · v` mỗi bước.
- Vì sao ma sát nhớt của khớp có Coulomb cũng ra khỏi RK4: trong một bước trượt, RK4 đẩy vận
  tốc lên tới `Δv_i` (1.3 rad/s ở cổ tay) trên giá trị thật trước khi Coulomb kéo về, nên
  `b q̇` tính trong RK4 bị thổi phồng (ở 0.5 rad/s, mô-men nhớt hiệu dụng gấp khoảng 2.3 lần).
- Trượt đều với mô-men không đổi (một khớp): vận tốc tăng đúng `h (τ - c) / m` mỗi bước.
- Năng lượng: với xung `J = -λ`, `q̇' = q̇ - W λ`, `ΔKE = Jᵀ (q̇ + q̇') / 2 = -Σ_i λ_i (q̇_i + q̇'_i) / 2`,
  nên công suất ma sát trung bình là `Σ c_i |q̇_i| + b_i q̇_i²` đúng như mô hình. Áp xung qua cả
  cột `W e_i` (chứ không chỉ sửa `q̇_i`) là điều giữ cân bằng này trên tay máy có liên kết
  quán tính.

## Xấp xỉ

- Liên kết giữa các khớp không nhỏ: trên tay máy này `W_ik / W_kk` của khớp 5-6 so với khớp
  2 và 4 lên tới 2.4-4.6. Vì vậy bài toán được giải liên kết (Gauss-Seidel + hệ tuyến tính
  chính xác), không xử lý từng khớp độc lập. Mọi khớp dùng cùng `M(q⁺)` cuối bước RK4.
- Ma sát là bậc một theo `h` (xung đặt cuối bước), phần vật rắn vẫn là RK4 bậc bốn.
- Khớp chạm giới hạn cứng được xử lý sau phép chiếu, như trước.
- Khác với thiết kế ban đầu: ma sát nhớt của khớp có Coulomb được đưa vào phép chiếu (dạng ẩn) thay vì ở
  lại trong RK4 (lý do ở mục trên), và xung được áp qua cả cột `M⁻¹` thay vì sửa riêng `q̇_i`
  (để giữ cân bằng năng lượng).
