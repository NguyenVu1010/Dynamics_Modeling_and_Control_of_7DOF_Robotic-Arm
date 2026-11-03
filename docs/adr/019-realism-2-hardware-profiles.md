# ADR-019: Realism-2, giới hạn khớp theo eRob, nguồn nhiễu phần cứng mới và ba profile mốc

- Trạng thái: chấp nhận (2026-11-02)
- Ngày: 2026-11-02
- Liên quan: thiết kế `docs/design/2026-11-02-realism-2-design.md`; ADR-005, ADR-011, ADR-015

## Bối cảnh

ADR-015 đưa vào lớp nhiễu cấu hình được (cảm biến, động cơ, ma sát, sai mô hình, jitter). Rà soát
lại cho thấy ba lỗ hổng làm kết luận "chịu được bao nhiêu nhiễu" kém tin cậy:

1. Thiếu các hiệu ứng lớn nhất của khớp harmonic drive và của bus EtherCAT: đàn hồi hộp số, sai số
   truyền động, độ rơ; giới hạn dòng, tốc độ tăng mô-men, back-EMF, sụt áp nguồn chung, bất đối xứng
   theo chiều, vùng chết; mất gói, trễ dồn, quá giờ; tải có quán tính và lệch tâm.
2. Giới hạn khớp trong URDF (87/12 N·m, 2,5/3,0 rad/s) chép từ Franka, không khớp với động cơ eRob
   (ratio 100) mà khối lượng và armature của repo đang dùng.
3. Giá trị nhiễu là giả định chung, không gắn với sản phẩm thật nào.

## Quyết định

- **Giới hạn khớp theo eRob** (khớp 1–2 eRob110H, 3–4 eRob90H, 5–7 eRob70H): ba mức mô-men mỗi khớp.
  Định mức R (87/52/10 N·m) là ngưỡng liên tục cho chỉ số mới `rms_tau_ratio`; đỉnh khởi động/dừng P
  (204/107/36 N·m) là `tau_max` của bộ điều khiển và kiểm tra đường đi (khoá `limits.peak_torque_nm`,
  `limits.rated_torque_nm`); tức thời M (369/191/70 N·m) là `effort` trong URDF, plant và Gazebo cắt
  cứng ở đó. Tốc độ 2,1 rad/s (khớp 1–4) và 3,1 rad/s (5–7), chờ datasheet bản H.
- **Nguồn nhiễu mới** trong `imperfect::ImperfectionConfig`, mặc định 0 = tắt, profile cũ cho kết quả
  y hệt:
  - động cơ (decorator): bất đối xứng theo chiều, vùng chết, giới hạn tốc độ tăng mô-men, đường cong
    mô-men theo tốc độ (chỉ khi kéo), sụt áp bus chung `V = V_nom − R·I`;
  - truyền thông và thời gian: tần số servo (giữ bậc không), mất gói theo từng gói servo (cả đo lẫn
    lệnh), đợt trễ lặp mẫu cuối, bước quá giờ (bench);
  - hộp số (plant bench): hai khối với lò xo ba đoạn theo catalog Harmonic Drive, sai số truyền động
    `2 × ratio` chu kỳ mỗi vòng, lost motion, encoder phía khâu hoặc phía động cơ;
  - tải: lệch tâm và quán tính quả cầu đặc.
- Thứ tự trong chuỗi mô-men: scale (± theo chiều) → gợn → vùng chết → độ phân giải → giữ theo servo
  → trễ lệnh → tốc độ tăng → trần mô-men theo tốc độ và điện áp → (plant) trễ motor, ma sát, hộp số.
- Mỗi nhóm nguồn ngẫu nhiên mới có luồng `mt19937_64` riêng (seed kèm hằng số riêng), nên bật một
  nguồn không làm đổi số ngẫu nhiên của nguồn khác.
- **Ba profile mốc**, cùng cơ khí và giới hạn eRob, chỉ mượn chất lượng cảm biến, truyền động, truyền
  thông: `premium_fr3.yaml` (Franka Research 3), `mid_kr1205.yaml` (Kassow KR1205, gần robot của dự
  án nhất về tầm với và tải), `budget_xarm7.yaml` (UFactory xArm 7, servo 250 Hz). Mỗi nhóm ghi nguồn
  và mức độ: công bố, suy ra, hay giả định.
- **Ngắt an toàn khi mất gói là kết quả cần chứng minh**: `safety.hal_timeout_ms` giữ 5 ms cho mọi
  profile; `ControlLoop` ghi lý do vào giữ (`non_finite`, `stamp_backwards`, `state_timeout`,
  `joint_limit`, `overspeed`); Summary CSV có `hold_time_s`, `hold_reason`; `cobot_bench sweep` quét
  được khoá cấu hình `config.safety.hal_timeout_ms`.
- **Cùng một bộ điều khiển** cho mọi profile. Nếu hộp số đàn hồi làm bộ điều khiển dao động, bộ hệ số
  mặc định được chỉnh **một lần** cho mọi profile, báo cáo số đo trước và sau.
- **Cập nhật 2026-11-03:** đo ở #48 cho thấy vòng PD đo vị trí phía khâu qua khớp đàn hồi ít giảm
  chấn không ổn định ở mọi ω dùng được (giới hạn Routh ω ≲ 1 rad/s), nên không chỉnh hệ số. Thay
  vào đó: cả ba profile dùng `sensor.position_source: motor`, và bộ điều khiển bù độ xoắn tĩnh
  `q̂ = q_motor − φ⁻¹(g(q))` với độ cứng danh định (tính năng `deflection-comp`); ω giữ 80 rad/s.
- **Cập nhật 2026-11-03 (#51):** đo với trễ cảm biến và trễ lệnh của ba profile cho thấy ω = 80 làm
  bản cao cấp và trung bình rung không tắt, bản giá rẻ dừng giữ. Thay cho "cùng một bộ điều khiển
  cho mọi profile", mỗi profile có cấu hình bộ điều khiển riêng `config/<profile>.yaml`, chỉ khác hệ
  số computed torque: `premium_fr3` ω = 50, `mid_kr1205` ω = 40, `budget_xarm7` ω = 20 rad/s;
  `config/default.yaml` (phần cứng lý tưởng) giữ ω = 80. Xem `docs/lessons/02-dieu-khien.md` mục 5.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Nhân đều mọi nhiễu theo một hệ số | Một tham số, dễ trình bày | Không nói được nguồn nào giới hạn độ chính xác |
| Sao chép nguyên robot mốc (hình học, khối lượng) | Số so sánh được trực tiếp với hãng | Không còn là robot của dự án |
| Bỏ ngắt khi mất gói ở hạng rẻ (thời gian chờ theo tần số servo) | Hạng rẻ chạy hết quỹ đạo | Che mất tính năng an toàn; chủ dự án muốn chứng minh nó |
| Hệ số điều khiển riêng cho từng profile | Mỗi profile đạt tốt nhất | So sánh không còn công bằng |
| Hộp số đàn hồi trong Gazebo ngay | Demo sát hơn | Phải đổi động lực khớp của Gazebo; để sau |
| Giảm hệ số để vòng đo phía khâu ổn định | Không đổi cấu trúc | Cần ω ≲ 1 rad/s; ω = 10 đã trượt G3 với khớp cứng |
| Hai vòng (vận tốc theo động cơ, vị trí theo khâu) | Bám đúng góc khâu | Tính năng điều khiển mới, để sau |

## Hệ quả

- Số liệu E2E và test đóng băng đổi theo giới hạn khớp mới (PR có nhãn `test-change`, ghi lý do
  từng số).
- Hợp đồng Run CSV thêm cột `hold`; Summary CSV thêm `rms_tau_ratio`, `hold_time_s`, `hold_reason`.
- Plant bench chậm hơn khi bật hộp số đàn hồi (bước tích phân con); vòng điều khiển không đổi.
- Ngưỡng E2E cho ba profile đo trước, chủ dự án duyệt.
- Khoá giảm chấn hộp số là `transmission.damping_ratio` (mặc định 0,01 của `2·sqrt(k1·B)`), thay cho
  `damping_nm_s_rad` trong bản thiết kế, để biểu diễn được cả giảm chấn bằng 0.
