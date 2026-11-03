# Thiết kế: realism-2, nguồn nhiễu sát phần cứng và ba profile theo robot mốc

- Trạng thái: đã duyệt (2026-11-02)
- Ngày: 2026-11-02
- Liên quan: `docs/design/2026-10-28-cpp-rework-design.md` (mục 11.1), ADR-005, ADR-011, ADR-015;
  ADR mới ADR-019 (realism-2) viết trong PR đầu của loạt này. Làm **trước**
  `docs/design/2026-11-02-cobot-studio-design.md` (ADR-020 bộ ghi log, ADR-021 ứng dụng Qt,
  ADR-022 cầu nối bộ nhớ chia sẻ và điều khiển tay).

## 1. Ý định và lý do

Chủ dự án muốn trình bày robot chịu được nhiễu tới đâu mà vẫn trong vùng chính xác. Kết luận đó chỉ
có giá trị khi nhiễu mô phỏng **sát phần cứng thật**. Rà soát hiện trạng cho thấy ba lỗ hổng:

1. Thiếu các hiệu ứng lớn nhất của khớp harmonic drive và của bộ điều khiển trên Pi 5: đàn hồi hộp
   số, sai số truyền động, độ rơ; động cơ bị giới hạn dòng, tốc độ tăng mô-men, back-EMF, sụt áp
   nguồn chung, bất đối xứng theo chiều, vùng chết; mất gói, trễ dồn, quá giờ EtherCAT; tải có quán
   tính và lệch tâm.
2. Giới hạn khớp trong URDF (mô-men 87/12 N·m, tốc độ 2,5/3,0 rad/s) chép từ Franka Panda/FR3
   (0,855 m, 3 kg), không phải từ động cơ eRob mà khối lượng và armature của repo đang dùng.
3. Giá trị nhiễu phần lớn là giả định chung chung, không gắn với một sản phẩm thật.

## 2. Mục tiêu, không mục tiêu, phản mục tiêu

Mục tiêu:

- **G1. Giới hạn khớp theo eRob** (ratio 100; mapping hiện có: khớp 1–2 eRob110H, 3–4 eRob90H,
  5–7 eRob70H).
- **G2. Nguồn nhiễu mới** (mục 6), mỗi nguồn cấu hình bằng khoá YAML, tắt được (giá trị 0 = lý
  tưởng, profile `ideal` cho kết quả **y hệt** hiện nay), quét được bằng `cobot_bench sweep`.
- **G3. Ba profile mốc**: `premium_fr3.yaml` (Franka Research 3), `mid_kr1205.yaml` (Kassow
  KR1205, gần robot của chủ dự án nhất), `budget_xarm7.yaml` (UFactory xArm 7). Mỗi khoá ghi nguồn
  gốc: số công bố, suy ra, hay giả định.
- **G4. Kết quả**: chạy E2E bench và báo cáo độ nhạy cho cả ba profile, đưa vào `docs/results/`.

Không mục tiêu:

- Đàn hồi hộp số và sai số truyền động trong **Gazebo** (cần đổi động lực khớp của Gazebo): để sau.
  Các nguồn nằm trong decorator (động cơ, truyền thông, cảm biến) tự có hiệu lực trên đường Gazebo.
- Mô hình nhiệt đầy đủ của động cơ: chỉ có chỉ số RMS mô-men so với định mức.
- Sao chép nguyên ba robot mốc: chỉ mượn **chất lượng** cảm biến, truyền động, truyền thông; cơ khí,
  khối lượng và giới hạn vẫn là của robot chủ dự án với động cơ eRob.
- Bộ điều khiển mới hay bù cho các nguồn mới: realism-2 chỉ làm plant khó hơn.

Phản mục tiêu:

- Profile `ideal` hay các profile cũ đổi kết quả mà không có lý do ghi trong PR.
- Một giá trị giả định trình bày như số công bố.
- Bước điều khiển (`ControlLoop::step`) chậm đi vì realism-2 (mọi hiệu ứng nằm ở plant hoặc
  decorator).

## 3. Ràng buộc

- Như dự án: C++17, đường nóng không cấp phát và không exception, lõi không ROS, tất định theo seed,
  E2E-03 so CSV bằng byte, 9 check CI bắt buộc, test đóng băng chỉ đổi với nhãn `test-change`,
  mỗi PR một tính năng.
- Ba profile chạy **cùng một bộ điều khiển và cùng tần số vòng 1 kHz** để so sánh công bằng.

## 4. Đã chốt với chủ dự án

1. Làm realism-2 trước ứng dụng `cobot_studio`.
2. Lấy thông số dòng eRob phổ biến làm mốc cho động cơ/hộp số.
3. Robot thương mại gần nhất về kích cỡ và tầm với là **Kassow KR1205** (1,2 m, 5 kg, 7 trục).
4. Ba profile theo ba robot mốc ở ba hạng: Franka FR3 / Kassow KR1205 / xArm 7, với bảng giá trị
   ở mục 7.
5. Danh sách nguồn nhiễu ở mục 6 (giới hạn động cơ, hộp số, truyền thông, tải có quán tính).
6. **Ngắt an toàn khi mất gói là một tính năng cần chứng minh**, không phải lỗi cần né. Thời gian
   chờ `safety.hal_timeout_ms` giữ mặc định 5 ms cho mọi profile (cùng một bộ điều khiển), là một
   tham số quét được, và sau này chỉnh được từ ứng dụng `cobot_studio`.
7. **Ổn định với khớp đàn hồi**: đo trước với hệ số hiện tại (computed-torque ω = 80 rad/s, kp = ω²,
   kd = 2ω) trên ba profile. Nếu có profile dao động hoặc mất ổn định thì chỉnh **bộ hệ số mặc định
   một lần cho mọi profile** (vẫn một bộ điều khiển chung), không chỉnh riêng từng profile. Báo cáo
   ghi kết quả cả trước và sau khi chỉnh. Đáp án test đóng băng đổi theo nằm trong PR có nhãn
   `test-change`, ghi lý do.
   **Cập nhật 2026-11-03 (sau #48):** đo cho thấy với encoder phía khâu không có ω nào từ 10 đến
   80 rad/s ổn định (giới hạn Routh của hệ hai khối ít giảm chấn: ω ≲ 1 rad/s), nên bỏ bước chỉnh
   hệ số. Chủ dự án chọn: khi có hộp số đàn hồi, **vòng vị trí dùng encoder phía động cơ** và bộ
   điều khiển **bù độ xoắn tĩnh** `q̂ = q_motor − φ⁻¹(g(q))` theo độ cứng danh định; hệ số giữ
   ω = 80. Phân tích ở `docs/lessons/02-dieu-khien.md` mục 4.

## 5. Quyết định của người thiết kế (chủ dự án có thể đổi)

- **Ba mức mô-men** mỗi khớp theo eRob:

  | Khớp | Định mức R | Đỉnh khởi động/dừng P | Tức thời M |
  | --- | --- | --- | --- |
  | 1–2 (110H) | 87 N·m | 204 N·m | 369 N·m |
  | 3–4 (90H) | 52 N·m | 107 N·m | 191 N·m |
  | 5–7 (70H) | 10 N·m | 36 N·m | 70 N·m |

  Bộ điều khiển và kiểm tra đường đi dùng **P** làm `tau_max` (mô-men dùng được khi chuyển động);
  plant cắt cứng ở **M** (URDF `effort`); **R** là ngưỡng liên tục cho chỉ số mới
  `rms_tau_ratio = max_j RMS(τ_j) / R_j` trong Summary CSV. Khoá mới `limits.rated_torque_nm`,
  `limits.peak_torque_nm` trong `config/default.yaml`.
- **Tốc độ khớp**: 2,1 rad/s cho khớp 1–4, 3,1 rad/s cho khớp 5–7 (eRob 20–30 vòng/phút; hai
  nguồn chưa thống nhất, ghi "chờ datasheet bản H").
- **Hạng giá rẻ ở 250 Hz**: vòng điều khiển vẫn 1 kHz; khoá `comm.servo_rate_hz` giữ (zero-order
  hold) cả phép đo lẫn lệnh, chỉ cập nhật mỗi `1000 / servo_rate_hz` chu kỳ.
- **Gợn mô-men**: giữ khoá cũ `ripple_periods_per_rev` (tương thích); các profile mới đặt gợn của
  động cơ ở giá trị nhỏ và đưa phần tuần hoàn của harmonic drive vào sai số truyền động (2 chu kỳ
  mỗi vòng trục vào, tức `2 × ratio` mỗi vòng trục ra).
- **Ngắt khi mất gói liên tiếp**: không thêm khoá riêng; dùng `safety.hal_timeout_ms` (5 ms) hiện
  có: mất gói quá thời gian đó thì `ControlLoop` chuyển sang giữ như hiện nay. Với hạng giá rẻ
  (250 Hz, mỗi mẫu lặp 4 ms) một gói mất đã đủ ngắt; đó là kết quả cần báo cáo ("bộ điều khiển phát
  hiện và dừng an toàn"), không chỉnh để tránh.
- **Chỉ số ngắt**: Summary CSV thêm `hold_time_s` (thời điểm vào giữ, trống nếu không giữ) và
  `hold_reason` (lý do `ControlLoop` đã có: dữ liệu cũ, dấu thời gian lùi, ...). Báo cáo realism-2
  ghi tỉ lệ lần chạy bị ngắt qua các seed cho mỗi profile.
- **`safety.hal_timeout_ms` quét được**: `cobot_bench sweep` cho phép tham số thuộc cấu hình bộ
  điều khiển (không chỉ nhóm nhiễu), để vẽ "tỉ lệ ngắt theo thời gian chờ" ở hạng giá rẻ; ví dụ
  `[5, 8, 12, 20]` ms.
- **Profile cũ** `realistic.yaml`, `pessimistic.yaml` giữ nguyên giá trị (E2E đang dựa vào chúng);
  chúng chỉ đổi kết quả vì G1 (giới hạn khớp), không vì nguồn mới.

## 6. Nguồn nhiễu mới

Khoá thêm vào `imperfect::ImperfectionConfig` (scalar áp cho mọi khớp, danh sách 7 phần tử cho từng
khớp, như hiện nay). Thứ tự trong chuỗi lệnh mô-men của decorator:
`scale (± theo chiều) → vùng chết → độ phân giải → giữ theo servo_rate → trễ lệnh → giới hạn tốc độ
tăng → trần mô-men theo tốc độ và điện áp bus → (plant) trễ motor, ma sát, hộp số`.

### 6.1 Động cơ (`actuator.*`, decorator)

| Khoá | Ý nghĩa |
| --- | --- |
| `torque_scale_asym_frac` | hệ số mô-men chiều âm = chiều dương × (1 − asym) |
| `deadband_nm` | \|τ lệnh\| dưới ngưỡng thì τ ra = 0 |
| `torque_slew_nm_s` | giới hạn \|dτ/dt\| của mô-men ra |
| `knee_speed_rad_s`, `zero_torque_speed_rad_s` | đường cong mô-men–tốc độ: trần = P tới knee, giảm tuyến tính về 0 ở zero-torque (tỉ lệ với V_bus / V_nom) |
| `bus.v_nom_v`, `bus.r_source_ohm`, `bus.v_min_v` | sụt áp nguồn chung: `V = V_nom − R·I`, `I = Σ max(τ_j ω_j, 0) / (η V)` (η = 0,85), kẹp ở `v_min_v`; ghép mọi khớp |

### 6.2 Hộp số (`transmission.*`, plant)

- **Đàn hồi**: mỗi khớp thành hai khối (rotor + armature phía động cơ, khâu phía tải) nối bằng lò xo
  **ba đoạn tuyến tính như catalog Harmonic Drive**: `stiffness_k1/k2/k3_nm_rad` với điểm gãy
  `torque_t1/t2_nm` (khoá `damping_nm_s_rad`, mặc định 1 % tới hạn). `k3 = 0` nghĩa là cứng tuyệt
  đối (mô hình hiện tại, kết quả y hệt). Test tần số riêng dùng biên độ nhỏ trong đoạn K1.
- **Sai số truyền động**: lệch góc `error_amp_rad · sin(2·ratio·q + φ)`, φ rút từ seed.
- **Lost motion**: vùng chết `lost_motion_rad` trong lò xo (hysteresis khi đổi chiều).
- **Vị trí encoder**: `sensor.position_source: link | motor`; `motor` đo góc phía động cơ chia
  ratio, nên thấy được đàn hồi và độ rơ.
- Bước tích phân con trong `SimPlant` khi cần cho ổn định (tần số riêng khoảng 20–30 Hz với
  armature hiện có); người xây dựng chọn số bước con, test năng lượng chứng minh ổn định.

### 6.3 Truyền thông và thời gian (`comm.*`, `timing.*`, decorator và host bench)

| Khoá | Ý nghĩa |
| --- | --- |
| `comm.servo_rate_hz` | 0 = mỗi chu kỳ; khác 0 thì giữ đo và lệnh như mục 5 |
| `comm.packet_loss_prob` | xác suất **mỗi gói servo** (mỗi chu kỳ khi `servo_rate_hz = 0`, mỗi `1000 / servo_rate_hz` chu kỳ khi khác 0): đo không cập nhật và lệnh không tới (động cơ giữ lệnh cũ) |
| `comm.latency_burst_prob`, `comm.burst_cycles` | đợt trễ: **lặp lại mẫu đo cuối** trong `burst_cycles` chu kỳ (dấu thời gian không bao giờ lùi) |
| `timing.overrun_prob`, `timing.overrun_cycles` | bước quá giờ: plant chạy thêm `overrun_cycles · dt` trước khi lệnh mới có hiệu lực (bench) |

### 6.4 Tải (`model.*`)

`payload_offset_m` (x, y, z trong khung `link_7`) và `payload_radius_m` (quán tính quả cầu đặc).
Mặc định 0 = chấm khối lượng tại gốc như hiện nay.

## 7. Ba profile

Cơ khí chung: hình học, khối lượng, armature của repo; giới hạn eRob mục 5; độ cứng hộp số theo
Harmonic Drive SHG-25/20/14, K3 khoảng `[5.0e4, 5.0e4, 2.5e4, 2.5e4, 0.71e4, 0.71e4, 0.71e4]` N·m/rad
(**giả định**). Người xây dựng đối chiếu catalog HD, lấy K1/K2/K3 và T1/T2 theo catalog, được sửa số
mà không cần hỏi lại, miễn ghi nguồn (tên catalog, trang) trong YAML và PR.

| Khoá | premium_fr3 | mid_kr1205 | budget_xarm7 | Cơ sở |
| --- | --- | --- | --- | --- |
| sensor.encoder_bits | 20 | 19 | 17 | xArm 17 bit (bên thứ ba); eRob 19 bit, tuỳ chọn 20 |
| sensor.q_noise_std_rad | 5e-6 | 1e-5 | 5e-5 | suy ra từ LSB |
| sensor.q_offset_std_rad | 2e-4 | 1e-3 | 2e-3 | giả định: chất lượng hiệu chuẩn |
| sensor.qd_filter_hz | 200 | 100 | 50 | giả định |
| sensor.sensor_delay_cycles | 1 | 1 | 1 | trễ thêm của hạng rẻ đến từ servo_rate |
| sensor.position_source | motor | motor | motor | vòng vị trí khép bằng encoder động cơ (mục 4.7, cập nhật 2026-11-03); encoder khâu không ổn định qua khớp đàn hồi |
| actuator.torque_scale_std | 0.02 | 0.05 | 0.10 | FR3 theo iiwa ±2 % (công bố); còn lại giả định |
| actuator.torque_scale_asym_frac | 0.005 | 0.01 | 0.03 | giả định |
| actuator.torque_ripple_frac | 0.01 | 0.02 | 0.05 | giả định (vòng mô-men triệt gợn) |
| actuator.deadband_nm | 0 | 0.001·R | 0.005·R | giả định (FR3 có cảm biến mô-men) |
| actuator.torque_cmd_resolution_nm | R/1000 | R/1000 | R/250 | CiA402 phần nghìn; hạng rẻ giả định |
| actuator.motor_time_constant_s | 0.0005 | 0.001 | 0.002 | giả định |
| actuator.command_delay_cycles | 1 | 1 | 1 | như sensor_delay |
| actuator.torque_slew_nm_s | P / 1 ms | P / 2 ms | P / 5 ms | giả định |
| actuator.knee / zero_torque_speed (1–4; 5–7) | 1.6, 2.6; 2.2, 3.6 | như trái | như trái | suy ra từ eRob 20–30 vòng/phút |
| bus (V_nom / R / V_min) | 48 / 0.05 / 40 | 48 / 0.10 / 40 | 48 / 0.25 / 40 | eRob 48 V ±10 % (công bố); điện trở giả định |
| friction.coulomb_nm (1–2, 3–4, 5–7) | 1.3, 0.8, 0.3 | 1.7, 1.0, 0.2 | 3.5, 2.1, 0.4 | FR3 theo nhận dạng Panda (bài báo); 2 % / 4 % R |
| friction.viscous_nm_s_rad (1–4, 5–7) | 0.3, 0.05 | 0.5, 0.1 | 1.0, 0.2 | giả định |
| transmission.error_amp_rad | 2.9e-4 | 4.4e-4 | 8.7e-4 | HD 1–1,5 arc-min (một phần công bố); 3 arc-min giả định |
| transmission.lost_motion_rad | 2.9e-4 | 2.9e-4 | 8.7e-4 | HD CSG-GH 2,9e-4 rad (công bố); hạng rẻ giả định |
| model.link_mass_scale_std | 0.02 | 0.05 | 0.10 | giả định |
| model.com_shift_std_m | 0.002 | 0.005 | 0.010 | giả định |
| timing.period_jitter_std_us | 5 | 20 | 100 | RT ±4 µs (công bố); SOEM user-space ~100 µs (bài báo) |
| comm.servo_rate_hz | 0 | 0 | 250 | xArm servo 250 Hz (hãng) |
| comm.packet_loss_prob | 1e-5 | 1e-4 | 1e-3 | giả định |
| comm.latency_burst_prob / burst_cycles | 1e-5 / 1 | 1e-4 / 2 | 1e-3 / 5 | giả định |
| timing.overrun_prob / overrun_cycles | 1e-5 / 1 | 1e-4 / 1 | 1e-3 / 3 | giả định |

`model.payload_kg` là 0 trong cả ba (quét riêng). Nguồn tham khảo (URL từng số) ghi trong trường
`source` của mỗi nhóm trong YAML và trong ADR-019.

## 8. Kiểm thử

- Mỗi hiệu ứng: test đáp án biết trước (vùng chết, giới hạn tăng mô-men cho thời gian lên đúng
  `ΔP / slew`, trần mô-men theo tốc độ, sụt áp khi hai khớp cùng tăng tốc, lò xo một khớp có tần số
  riêng đúng `√(K / J)`, lost motion tạo hysteresis đúng bề rộng, giữ theo servo_rate, mất gói giữ
  lệnh cũ).
- Ngắt: ở servo 250 Hz, không mất gói thì **không** ngắt với 5 ms; mất đúng một gói thì vào giữ
  trong khoảng `(timeout, timeout + 1 chu kỳ]` sau mẫu mới cuối cùng, `hold_reason` đúng; đổi
  `hal_timeout_ms` lên 12 ms thì một gói mất không ngắt nữa.
- Giá trị 0 của mọi khoá mới cho kết quả **byte-identical** với trước (E2E-03 trên profile cũ
  sau khi đã cập nhật đáp án vì G1).
- Năng lượng: khớp đàn hồi không ma sát, không lệnh, bảo toàn năng lượng trong sai số tích phân.
- Không cấp phát sau khởi tạo; tất định theo seed.
- Ba profile nạp được, tên khoá trùng schema; `config/sweeps/hardware.yaml` có các tham số mới.
- Đáp án test/E2E đổi vì G1 (giới hạn khớp) nằm trong PR riêng, nhãn `test-change`, ghi lý do.
- Ổn định với khớp đàn hồi (mục 4.7): trên mỗi profile, quỹ đạo mặc định không có dao động kéo dài
  (biên độ dao động sai số khớp sau khi tới đích giảm dần, không vào giữ vì lý do khác ngoài mất
  gói); với encoder phía động cơ và bù độ xoắn bật, sai số khâu thật sau đích nhỏ hơn rõ rệt so với
  khi tắt bù.

## 9. Chia PR (dự kiến)

1. `docs`: ADR-019, bản thiết kế này; bản thiết kế `2026-11-02-cobot-studio-design.md`.
2. `joint-limits`: URDF/config theo eRob (P, M, R, tốc độ), `rms_tau_ratio`; cập nhật đáp án.
3. `actuator-limits`: bất đối xứng, vùng chết, tăng mô-men, trần theo tốc độ, sụt áp bus.
4. `comm-timing`: servo_rate, mất gói, trễ dồn, quá giờ.
5. `transmission`: đàn hồi hai khối, sai số truyền động, lost motion, vị trí encoder; đo ổn định
   với hệ số hiện tại (mục 4.7).
5b. ~~`retune`~~ bỏ (mục 4.7, cập nhật 2026-11-03), thay bằng `deflection-comp`: bù độ xoắn tĩnh
   phía bộ điều khiển khi vị trí đo từ encoder động cơ; đo sai số khâu thật trước/sau.
6. `payload-inertia`: tải lệch tâm có quán tính.
7. `profiles`: ba YAML, tham số quét mới, chạy E2E và báo cáo độ nhạy vào `docs/results/realism2/`,
   README.

## 10. Để lại cho người xây dựng

- Cách tích phân hai khối (bước con, phương pháp), miễn qua test năng lượng và tần số riêng.
- Cách tính dòng tổng cho sụt áp (mô hình công suất nêu ở 6.1 là tối thiểu; thêm tổn hao đồng nếu
  muốn), miễn tắt được và tất định.
- Tên struct C++ và cách chia file trong `imperfect/` và `dynamics/`.
- Ngưỡng E2E mới cho từng profile: đo trước, đề xuất trong PR `profiles`, chủ dự án duyệt.
