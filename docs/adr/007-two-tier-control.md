# ADR-007: Luật điều khiển hai tầng: CLIK bậc hai trên tham chiếu → computed torque khớp

- Trạng thái: đề xuất, chờ bàn
- Ngày: 2026-10-29
- Liên quan: thiết kế mục 2 (G3), 5.4, 10; ADR-004, ADR-008; `docs/derivations/control.md`

## Bối cảnh

Bản cũ có 4 tần số: quỹ đạo Cartesian 20 Hz → IK vận tốc bằng giả nghịch đảo có tối ưu giới hạn
khớp → computed torque + PD 100 Hz → node giữ vị trí 300 Hz. Tiêu chí G3 đo cả sai số đầu cuối
lẫn RMS sai số bám khớp; tiêu chí thứ hai chỉ có nghĩa khi có một quỹ đạo khớp tham chiếu rõ
ràng. Luận văn kể mạch "mô hình hoá động lực học → computed torque", nên luật điều khiển chính
phải dùng M, C, G của ADR-004. Cấu trúc hai tầng là quyết định của người thiết kế sau vòng review.

## Quyết định

Một vòng 1 kHz trong `ControlLoop::step()`, mỗi bước:

1. HAL đọc `q, q̇` (và `τ_đo` nếu có).
2. **`planning::CartesianTrajectory`**: `x_d, ẋ_d, ẍ_d` tại `t`. Luật thời gian quintic `s(t)`
   trên `[0, T]`, vị trí đường thẳng, hướng `R(t) = R_0 · Exp(s · Log(R_0ᵀ R_T))`. Sau `T` giữ
   `x_T`, vận tốc và gia tốc bằng 0.
3. **`planning::ClikReference`** (tầng tham chiếu, không nhìn trạng thái đo): tính
   `T_b_e(q_d)`, `J(q_d)`, `J̇q̇_d`; `e = [p_d − p(q_d); Log(R_d R(q_d)ᵀ)]`, `ė = ẋ_d − J q̇_d`;
   `q̈_d = J⁺_λ (ẍ_d + K_e,d ė + K_e,p e − J̇q̇_d) + (I − J⁺_λ J) q̈_null`, với `J⁺_λ` giả nghịch
   đảo giảm chấn (`λ` theo số điều kiện, `lambda_max`, `cond_threshold` trong YAML),
   `q̈_null = −k_null ∇h(q_d) − k_damp q̇_d`, `h` phạt khoảng cách tới giới hạn khớp. Tích phân
   semi-implicit Euler `q̈_d → q̇_d → q_d`, kẹp `q_d` trong giới hạn. Khởi tạo `q_d = q` đo khi
   nhận đích.
4. **`control::ComputedTorque`** (tầng điều khiển):
   `τ = M(q)(q̈_d + K_d(q̇_d − q̇) + K_p(q_d − q)) + C(q, q̇) q̇ + G(q) + D q̇`, `K_p, K_d` đường
   chéo trong YAML, kẹp theo `tau_max` của URDF.
5. HAL ghi `τ`; một `LogRecord` vào bộ đệm vòng.
6. **An toàn**: khi `q` cách giới hạn ít hơn biên mềm (`safety.soft_margin_deg`), `|q̇| > qd_max`,
   HAL quá hạn (`safety.hal_timeout_ms`) hoặc dữ liệu NaN, chuyển sang **giữ**
   (`control::HoldController`: PD về `q` lúc chuyển + G) và báo `LoopMode::kHolding`. Không bao
   giờ ghi NaN ra HAL.

Số đo G3: sai số đầu cuối từ FK của `q` đo; RMS bám khớp = RMS của `q_d − q`; số đo phụ `‖e‖` của
CLIK tách lỗi tầng tham chiếu khỏi lỗi tầng điều khiển.

**Câu hỏi mở cho chủ dự án:** chấp nhận cấu trúc hai tầng, hay một trong các phương án dưới đây?
Mặc định trong lúc chờ: hai tầng. Hàm `h(q)` và cách tính `λ` do người xây dựng chọn và ghi trong
`docs/derivations/control.md`.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Hai tầng: CLIK bậc hai trên tham chiếu → computed torque khớp (mặc định) | `q_d` tái lập được, test không cần động lực học; RMS bám khớp có nghĩa; giữ mạch computed torque | Đang chọn; sai số CLIK cộng vào sai số đầu cuối, cần số đo `‖e‖` riêng |
| Resolved acceleration trực tiếp trên `q` đo (computed torque không gian tác vụ) | Một tầng, sai số đầu cuối hội tụ trực tiếp | Không có quỹ đạo khớp tham chiếu nên RMS bám khớp vô nghĩa; nhiễu đo đi thẳng vào nghịch đảo Jacobian; khó tách lỗi |
| CLIK bậc một + sai phân số `q̇_d, q̈_d` như bản cũ | Đơn giản, đã có | Sai phân khuếch đại nhiễu và gây gai mô-men; `q̈_d` không liên tục |
| Giữ node giữ vị trí riêng 300 Hz | Quen | Hai bộ điều khiển tranh lệnh; thay bằng chế độ giữ trong cùng vòng |
| Euler cho sai số hướng | Dễ hiểu | Kỳ dị gimbal; dùng log SO(3) theo `docs/conventions.md` |

## Hệ quả

Mỗi tầng test riêng: CLIK trên `Model` không cần `dynamics`, computed torque với `SimPlant` biết
trước đáp án. Gains có hai bộ (`clik.*` và `computed_torque.*`) cần tinh chỉnh; giá trị khởi điểm
lấy từ bản cũ và ghi lại cách chỉnh ở giai đoạn `mvp-results`.
