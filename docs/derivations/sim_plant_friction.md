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
2. Sau bước, ở `q⁺`, tính `M(q⁺)` và LDLT. Với từng khớp `i` có `c_i > 0`, ma sát trong cả bước
   là một xung khớp `-λ_i e_i`. Nó đổi vận tốc mọi khớp một lượng `-λ_i M⁻¹ e_i`, riêng khớp
   `i` một lượng `-λ_i / m_i` với quán tính hiệu dụng `m_i = 1 / (M⁻¹)_ii`. Gọi `v` là vận
   tốc khớp trước, `v'` sau:

   ```
   λ_i = h (c_i sign(v') + b_i v'),        v' = v - λ_i / m_i
   ```

   Đặt `Δv_i = h c_i / m_i`:

   ```
   |v| ≤ Δv_i  ->  v' = 0                              (dính: ma sát tĩnh cân bằng phần còn lại)
   |v| > Δv_i  ->  v' = (v - sign(v) Δv_i) / (1 + h b_i / m_i)     (trượt, cùng dấu với v)
   ```

3. Vị trí theo quy tắc hình thang: `q += h/2 · (q̇_mới - q̇_RK4)`; riêng khớp có Coulomb
   `q_i = q_i,đầu + h/2 · (q̇_i,đầu + q̇_i,mới)`.

## Ổn định và các tính chất

- Phép chiếu không bao giờ đảo dấu vận tốc và `|v'| ≤ |v|` với mọi `h`, `c_i`, `b_i`, `m_i`:
  ổn định vô điều kiện. Ma sát nhớt ở dạng ẩn (Euler lùi), hệ số `1 / (1 + h b / m) ∈ (0, 1]`.
- Dính thật: dưới ngưỡng `|τ| < c` khớp có `v' = 0` chính xác và, nhờ hình thang với
  `q̇_đầu = q̇_mới = 0`, vị trí không đổi một bit. Nếu để ma sát nhớt trong RK4, nó tác dụng
  lên vận tốc trung gian `≈ h τ / m` mà phép chiếu chỉ xóa sau đó, và vị trí bò
  `≈ h² / 12 · (b / m) · v` mỗi bước.
- Vì sao ma sát nhớt của khớp có Coulomb cũng ra khỏi RK4: trong một bước trượt, RK4 đẩy vận
  tốc lên tới `Δv_i` (1.3 rad/s ở cổ tay) trên giá trị thật trước khi Coulomb kéo về, nên
  `b q̇` tính trong RK4 bị thổi phồng (ở 0.5 rad/s, mô-men nhớt hiệu dụng gấp khoảng 2.3 lần).
- Trượt đều với mô-men không đổi: vận tốc tăng đúng `h (τ - c) / m` mỗi bước; vị trí hình thang
  chính xác với gia tốc hằng.
- Năng lượng: với xung `J = -λ_i e_i`, `ΔKE = Jᵀ (q̇ + q̇') / 2 = -λ_i (v + v') / 2`, nên công
  suất ma sát trung bình là `c |q̇| + b q̇²` đúng như mô hình. Áp xung qua cả cột `M⁻¹ e_i` (chứ
  không chỉ sửa `q̇_i`) là điều giữ cân bằng này trên tay máy có liên kết quán tính.

## Xấp xỉ

- Các khớp được xử lý lần lượt (theo chỉ số), mỗi khớp thấy vận tốc mà các khớp trước để lại,
  với cùng `M(q⁺)`. Đây là một lượt Gauss-Seidel của bài toán dính-trượt liên kết; với tay máy
  này các phần tử ngoài đường chéo của `M⁻¹` giữa các khớp cổ tay nhỏ so với đường chéo.
- Ma sát là bậc một theo `h` (xung đặt cuối bước), phần vật rắn vẫn là RK4 bậc bốn.
- Khớp chạm giới hạn cứng được xử lý sau phép chiếu, như trước.
