# Hộp số đàn hồi trong SimPlant

Cài đặt: `core/dynamics/src/sim_plant.cpp` (`spring`, `derivative`, `substep`) và
`core/dynamics/include/cobot/dynamics/harmonic_spring.hpp`. Thiết kế: realism-2 mục 6.2, 7, 8.
Chỉ có ở bench; Gazebo vẫn là khớp cứng.

## Mô hình

Khớp `j` đàn hồi khi `k3_j > 0`. Mọi góc phía động cơ quy về phía khâu (góc động cơ chia tỉ số
truyền), nên `θ_j` và `q_j` cùng đơn vị.

```
khâu:     M_link(q) q̈ + C(q, q̇) q̇ + G(q) = τ_l
          τ_l,j = τ_s,j                        (khớp đàn hồi)
          τ_l,j = τ_m,j − b_j q̇_j              (khớp cứng, như cũ)
động cơ:  B_j θ̈_j = τ_m,j − τ_s,j − τ_f,j
```

- `M_link` là ma trận khối lượng CRBA **không có** armature của các khớp đàn hồi; armature URDF
  (`cobot_armature`, đã quy về phía khâu) trở thành quán tính động cơ `B_j`.
- `τ_m` là mô-men sau trễ động cơ và giới hạn `tau_max` (như khớp cứng).
- `τ_f` (Coulomb + nhớt của PlantEffects, cộng damping URDF của khớp) tác dụng lên **động cơ**.

Lò xo:

```
e  = error_amp · sin(2 · ratio · θ + motor_phase)          (sai số truyền, chu kỳ π / ratio theo θ)
φ  = θ − q − e
φ' = φ − clamp(φ, −lost/2, +lost/2)                          (vùng chết rộng lost)
τ_s = H(φ') + d (θ̇ − q̇),     d = damping_ratio · 2 sqrt(k1 B)
```

`H` là đường cong ba đoạn của Harmonic Drive, lẻ theo `φ`:

```
|φ| ≤ φ1 = t1 / k1                    : k1 |φ|
φ1 < |φ| ≤ φ2 = φ1 + (t2 − t1) / k2   : t1 + k2 (|φ| − φ1)
|φ| > φ2                              : t2 + k3 (|φ| − φ2)
```

Thế năng là tích phân của `H`; với `d = 0` và không ma sát, tổng
`½ q̇ᵀ M_link q̇ + ½ Σ B_j θ̇_j² + Σ U(φ_j)` được bảo toàn (test năng lượng).

Hai khối một khớp (`J_l`, `B`, đoạn K1): `φ̈ = −k1 (1/B + 1/J_l) φ`, nên
`ω = sqrt(k1 (1/B + 1/J_l))`. Với SHG-14, `B = 0,22674`, `J_l = 0,2`: `ω = 210 rad/s` (33,5 Hz).

## Tích phân

Trạng thái `(q, q̇, θ, θ̇)`, RK4 cổ điển với `n` bước con trong mỗi chu kỳ `dt`:

```
ω_max² = max_j  k_max,j (1/B_j + (M_link⁻¹)_jj)     (tại q đầu bước)
n      = clamp(ceil(dt · ω_max / 0,05), 1, 64)
```

Với dao động tuyến tính, RK4 làm biên độ giảm cỡ `(ω h)⁶ / 144` mỗi bước con; `ω h ≤ 0,05` cho
khoảng `1e-10` mỗi bước con, tức `|ΔE| / E0` cỡ `1e-6` sau 2 s ở 210 rad/s (5 bước con mỗi chu
kỳ), dưới ngưỡng `1e-4` của test. Số bước con tăng theo `ω_max`, nên khâu cổ tay nhẹ làm bước
chậm hơn; vòng điều khiển không đổi.

Sau mỗi bước con:

1. **Ma sát động cơ** (khớp đàn hồi có `c > 0`): bài toán dính-trượt một chiều với quán tính `B`,
   dùng lại `slide_or_stick` (`sim_plant_friction.md`): `Δv = h c / B`, nhớt ẩn `h b / B`, vị trí
   sửa theo quy tắc hình thang. Đầu bước con, nếu `θ̇ = 0` và `|τ_m − τ_s| ≤ c` thì động cơ bị giữ
   cả bước con (`θ̈ = 0` ở mọi giai đoạn RK4), nên khi dính `θ` và `q` không trôi một bit nào.
2. **Khớp cứng có Coulomb** trong tay máy có khớp đàn hồi: phép chiếu liên kết cũ (`apply_friction`)
   với `M_link`, chỉ trên các khớp cứng.
3. **Chặn cứng** của khâu như cũ (kẹp vị trí, bỏ vận tốc hướng ra ngoài). Phía động cơ không bị
   kẹp; lò xo giữ nó. Ngoài ra, đầu bước con, một khâu đang nằm ở chặn (`q̇ = 0`) mà gia tốc tự do
   đẩy ra ngoài thì được khoá (`q̈_j = 0`: hàng và cột `j` của `M_link` thay bằng đơn vị) suốt bước
   con. Không có khoá này, các giai đoạn RK4 thấy khâu trôi qua chặn rồi bị kẹp lại, mỗi lần kẹp
   nén lò xo thêm mà không tốn công, và năng lượng tăng dần khi động cơ đẩy vào chặn.

`spring_torque()` là `τ_s` tính lại tại trạng thái cuối bước. Khớp cứng báo `q_motor = q`,
`qd_motor = q̇`, `spring_torque = tau_applied`. Khi không khớp nào đàn hồi, `step` đi đúng nhánh
RK4 cũ, nên kết quả trùng từng bit với trước tính năng (test `TransmissionRigid`).

## Đo

`SimJointInterface` điền `q_motor`, `qd_motor`, `motor_valid = true`. Bộ trang trí với
`sensor.position_source: motor` lấy `q_motor`/`qd_motor` làm vị trí và vận tốc thô khi
`motor_valid`, ngược lại dùng khâu. Trần mô-men theo tốc độ dùng `qd_motor` khi hợp lệ.

## Bù độ xoắn tĩnh phía bộ điều khiển

Vòng vị trí khép bằng encoder động cơ (`position_source: motor`) ổn định ở ω = 80 rad/s, nhưng
giữ `θ` (quy về phía khâu) đúng tham chiếu, nên khâu thật lệch đúng bằng độ xoắn tĩnh
`φ = q_motor − q`, tới cỡ `1e-3` rad ở khớp 2 và 4 tại home (TCP lệch cỡ 1,6 mm). Ở trạng thái
tĩnh lò xo mang mô-men trọng lực, `τ_s(φ) = g(q)`, nên bộ điều khiển ước lượng vị trí khâu bằng

```text
q̂_j = q_motor,j − φ⁻¹_j(g_j(q_motor))
```

với `φ⁻¹ = harmonic_spring_deflection`, nghịch đảo đúng của `harmonic_spring_torque` (tuyến tính
từng đoạn, lẻ, cùng các điểm gãy `t1`, `t2`), độ cứng danh định ở `cobot.deflection_comp` (tách
khỏi `imperfections.transmission` của plant để nghiên cứu sai lệch), `g` của mô hình bộ điều khiển
(có tải danh định). `ControlLoop` thay `q` bằng `q̂` ngay khi nhận trạng thái có `q_from_motor`
(chỉ bộ trang trí đặt cờ này), trước kiểm tra an toàn, CLIK, mô-men và log; `q̇` giữ nguyên. Một
lần tính trọng lực mỗi chu kỳ, không cấp phát. Encoder khâu, hoặc tắt bù: trùng từng bit với trước.

Giới hạn: `g` được tính tại `q_motor` thay vì `q` thật, sai cỡ `|∂g/∂q| φ / k` (≲ `1e-5` rad);
không bù sai số truyền động, lost motion, độ xoắn động (quán tính, ma sát khi chuyển động); sai độ
cứng danh định để lại phần tương ứng của độ xoắn (cứng thật bằng 0,8 lần danh định: còn 20 %).

Đo (E2E-01, khớp đàn hồi catalog, mọi thứ khác lý tưởng, max_j |q_thật − q_ref| trong 0,5 s cuối,
TCP thật ở chu kỳ cuối):

| Plant | Bù | Sai số khâu thật (rad) | TCP (mm) |
| --- | --- | --- | --- |
| danh định | tắt | 9,910e-4 | 1,60 |
| danh định | bật | 1,197e-6 | 0,0014 |
| mềm hơn 20 % | tắt | 1,239e-3 | 2,00 |
| mềm hơn 20 % | bật | 2,480e-4 | 0,40 |
