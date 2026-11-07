# ADR-023: Kiểm nghiệm kịch bản phần cứng, kết luận ba mức có điều kiện

- Trạng thái: chấp nhận (2026-11-03)
- Ngày: 2026-11-03
- Liên quan: thiết kế `docs/design/2026-11-02-cobot-studio-design.md` mục 10 (giai đoạn C);
  ADR-015, ADR-019, ADR-020, ADR-021; `docs/lessons/03-danh-gia-va-bao-cao.md` mục 4

## Bối cảnh

Realism-2 cho dự án ba profile phần cứng có nguồn gốc từng thông số, mô hình hộp số đàn hồi,
truyền thông, động cơ, tải, cùng Monte Carlo, quét và ngưỡng E2E. Tính năng `profiles` (#51) cho
thấy dự án đã đưa ra được một kết luận có giá trị cho người chọn phần cứng: bản giá rẻ (servo
250 Hz, mô-men gửi từ máy chủ) không ổn định ở mọi hệ số, cần vòng điều khiển trong driver.

Chủ dự án chốt hướng (2026-11-03): `cobot_studio` trở thành công cụ kiểm nghiệm kịch bản phần
cứng. Người dùng chọn model phần cứng, thuật toán, tải, thông số; công cụ kết luận hệ thống có đạt
yêu cầu của nhiệm vụ hay không.

Cũng #51 cho thấy ba cách một kết luận mô phỏng có thể sai:

1. **Hệ số chưa chỉnh cho phần cứng đó.** Với ω = 80 chung, bản cao cấp trượt 0,6–2,7 mm; với
   ω = 50 nó đạt 0,04–0,12 mm. Một kết luận "không đạt" khi đó là kết luận về hệ số, không phải
   về phần cứng.
2. **Đạt sát ngưỡng.** Bản trung bình, hạt giống 3: biên độ dao động chỉ giảm từ 1,457e-4 xuống
   1,438e-4 rad. Một chữ "đạt" ở đây không nói gì về lần chạy thứ tư.
3. **Thông số giả định.** Một phần thông số của profile là giả định (`source` ghi `assumed`); kết
   luận chỉ đúng tới mức các giả định đúng.

## Quyết định

**Định vị.** Công cụ sàng lọc và so sánh phần cứng trước khi mua hoặc chế tạo. Không phải chứng
nhận: không thay đánh giá an toàn theo ISO 10218 hay ISO/TS 15066, không thay đo trên robot thật.
Mọi báo cáo ghi câu này ở đầu.

**Đầu vào của một lần kiểm nghiệm**, mỗi phần một file YAML có kiểm tra khoá như ADR-009:

- **Model phần cứng** `config/hardware/<model>.yaml`: tham chiếu profile nhiễu
  (`config/imperfections/<profile>.yaml`), giới hạn khớp, và bộ hệ số mặc định đã chỉnh cho nó.
  Ba model đầu tiên là ba profile của ADR-019.
- **Thuật toán** `control.algorithm`: chọn trong danh sách đăng ký lúc build (computed torque, PD
  cộng bù trọng lực, vòng vị trí trong driver, ...). Chọn lúc chạy, không đổi bước điều khiển nóng
  (không cấp phát, không exception, ADR-008).
- **Nhiệm vụ** `config/tasks/<task>.yaml`: quỹ đạo hoặc danh sách điểm (ADR-018), tải (khối lượng,
  lệch tâm, bán kính), và **yêu cầu**: sai số vị trí và góc cuối, sai số bám RMS, thời gian chu
  kỳ, tỉ lệ dừng an toàn chấp nhận được, tỉ lệ đạt mục tiêu.

**Tự chỉnh hệ số trước khi kết luận.** Với mỗi tổ hợp model và thuật toán, công cụ quét hệ số (với
computed torque: ω, kp = ω², kd = 2ω), chọn mức tốt nhất theo sai số bám, với điều kiện không dừng
an toàn, thời gian bão hoà mô-men dưới 5 %, và dao động sau đích tắt dần. Hệ số đã chọn và bảng
quét nằm trong báo cáo. Người dùng tắt được bước này để kiểm nghiệm đúng hệ số của mình.

**Kết luận ba mức từ Monte Carlo.** Mặc định 100 lần chạy (hạt giống 1..100), mỗi lần đánh giá
theo yêu cầu của nhiệm vụ. Tỉ lệ đạt có khoảng tin cậy Wilson 95 %:

| Kết luận | Điều kiện |
| --- | --- |
| **ĐẠT** | cận dưới tỉ lệ đạt ≥ mục tiêu (mặc định 95 %), và giá trị xấu nhất của mỗi chỉ số ≤ (1 − biên) × ngưỡng, biên mặc định 20 % |
| **KHÔNG ĐẠT** | cận trên tỉ lệ đạt < mục tiêu |
| **SÁT NGƯỠNG, CẦN ĐO THẬT** | các trường hợp còn lại |

Với 100 lần chạy đều đạt, cận dưới Wilson là 0,963; với 30 lần chạy đều đạt chỉ là 0,886, nên 30
lần không bao giờ đủ để kết luận ĐẠT ở mục tiêu 95 %. Chế độ xem nhanh (ít lần chạy) được phép
nhưng chỉ cho ra SÁT NGƯỠNG hoặc KHÔNG ĐẠT, và báo cáo ghi rõ là xem nhanh.

**Báo cáo** (Markdown và JSON, cùng nội dung):

- kết luận, tỉ lệ đạt và khoảng tin cậy, giá trị xấu nhất từng chỉ số so với ngưỡng;
- **danh sách giả định**: mọi thông số có `source` là `assumed` hoặc `derived`, kèm câu hỏi "kết
  luận có đổi không nếu thông số này lệch": dùng phân tích độ nhạy sẵn có (ADR-015) trên các
  thông số đó;
- hệ số đã chọn và bảng quét;
- nguyên nhân chính khi KHÔNG ĐẠT (nguồn nhiễu đóng góp nhiều nhất, lý do dừng an toàn) và gợi ý
  (ví dụ "cần vòng điều khiển trong driver", "tăng tần số servo");
- phiên bản: commit, file đầu vào và hash của chúng.

**Điều kiện tiên quyết.** Thuật toán "vòng vị trí trong driver" (driver chạy vòng vị trí và dòng
điện ở vài kHz, máy chủ gửi điểm đặt vị trí) làm trước kết luận đầu tiên. Thiếu nó thì công cụ
luôn kết luận KHÔNG ĐẠT cho lớp cobot giá rẻ vì một lý do kiến trúc mà phần cứng thật không có.

**Kiểm chứng mô hình (sim-to-real).** Một bước sau: nạp log robot thật, nhận dạng ma sát, độ cứng,
trễ, so với profile, và ghi độ lệch vào báo cáo. Đến lúc đó, kết luận chỉ dựa trên profile.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Kết luận đạt/không đạt hai mức theo ngưỡng E2E | Đơn giản, đã có | Không phân biệt đạt chắc với đạt sát ngưỡng; một hạt giống đạt không nói gì về lần sau (#51) |
| Kết luận trên hệ số người dùng nhập, không tự chỉnh | Minh bạch, nhanh | Kết luận về hệ số chứ không về phần cứng: ω = 80 làm bản cao cấp trượt, ω = 50 làm nó đạt (#51) |
| Số lần chạy cố định nhỏ (3 hạt giống như E2E) | Nhanh | Không đủ thống kê: 3 lần đều đạt cho cận dưới Wilson 0,44 |
| Giữ dự án là bộ điều khiển kèm bench | Phạm vi nhỏ | Bỏ phí phần profile và Monte Carlo đã có; câu hỏi "phần cứng này có đủ không" là câu người dùng thật cần trả lời |

## Hệ quả

- `cobot_studio` có thêm tab Kiểm nghiệm (thiết kế mục 10); `cobot_bench` có lệnh `verify` chạy
  không giao diện, cho cùng kết luận (phản mục tiêu: studio và bench khác nhau).
- Thời gian: 100 lần chạy khớp đàn hồi, mỗi lần vài giây, cộng bước tự chỉnh hệ số; cần chạy song
  song nhiều tiến trình và hiện tiến độ.
- Bộ điều khiển phải có giao diện chọn thuật toán; computed torque hiện tại thành một mục trong
  danh sách, kết quả bit-identical (E2E-03).
- Profile phải có `source` cho mọi nhóm (đã có từ #51); model phần cứng mới thiếu nguồn gốc thì
  không nạp được.
- Phải đo: thời gian bước khi chọn thuật toán qua cấu hình (PF-01 không đổi), độ ổn định của kết
  luận khi đổi dãy hạt giống (100 hạt giống khác cho cùng kết luận trên ba model mốc).

## Bổ sung (2026-11-07): PR controller-registry

- **Danh sách đăng ký đóng.** `cobot/control/algorithm.hpp` có bảng hằng `kAlgorithms` (tên YAML,
  mô tả một dòng) và `enum class Algorithm : std::uint8_t` (`kComputedTorque = 0`,
  `kPdGravity = 1`; giá trị cố định, chỉ thêm vào cuối). `TrackingController<N>` giữ mọi luật theo
  giá trị, `compute()` là một `switch` trên enum, `noexcept`; không hàm ảo, không
  `std::function`, không cấp phát. "Đăng ký lúc build" = một dòng trong bảng và một nhánh trong
  `switch`. `ControlLoop` thay `ComputedTorque` bằng `TrackingController` và có thêm
  `algorithm()`; phần còn lại của bước (feed-forward ma sát, clamp, hold, fault, log) không đổi.
- **Khoá cấu hình.** `cobot.control.algorithm` (tuỳ chọn, mặc định `computed_torque`; tên lạ báo
  lỗi kèm danh sách đã đăng ký) và `cobot.pd_gravity: {kp, kd}` (bắt buộc khi chọn `pd_gravity`,
  ngoài ra vẫn được đọc và kiểm tra). Bốn file cấu hình bộ điều khiển ghi rõ
  `control: {algorithm: computed_torque}`.
- **PD cộng bù trọng lực.** `τ = J ⊙ (k_p ⊙ (q_ref − q) + k_d ⊙ (q̇_ref − q̇)) + G(q)`, `k_p` 1/s²,
  `k_d` 1/s, `J = diag(M(home_q))` của mô hình bộ điều khiển, tính một lần khi dựng (câu hỏi C6):
  cùng `ω` là cùng băng thông như computed torque, tự chỉnh quét một tham số mỗi luật
  (docs/derivations/control.md "PD cộng bù trọng lực"). Trên plant lý tưởng, `ω = 80`, sai số bám
  RMS lớn nhất theo khớp là 8.78e-6 rad với PD+G và 6.66e-7 rad với computed torque.
- **Hold không đổi.** `HoldController` vẫn ở `ControlLoop` với hệ số computed torque cho cả hai
  thuật toán, nên đường hold giữ nguyên từng bit.
- **Kiểm bit-identical.** `tools/run_csv_hashes.sh` chạy năm Run CSV (E2E-01, 02, 03, 04, 10) với
  bản Release của main 6161dcb và của nhánh trên cùng một máy (WSL Ubuntu 24.04, x86_64): sha256
  của cả năm file bằng nhau. Không lưu hash vào repo vì arm64 (FMA) và x86_64 khác nhau ở bit cuối.
  Trong CI, test vàng `test_loop_config_regression` và các test so bit của `TrackingController`
  và `ControlLoopRegistry` giữ điều này.
- **PF-01.** `cobot_bench timing --config config/default.yaml --steps 10000`, Release, 5 lần main
  và 5 lần nhánh xen kẽ; trung vị p50 / p95 / max: main 4.726 / 4.967 / 117.5 µs, nhánh
  4.733 / 4.992 / 84.3 µs. Chênh +0.007 µs (p50) và +0.025 µs (p95), trong dung sai +0.25 / +0.35
  µs: đạt. PD+G (cùng lệnh, `algorithm: pd_gravity`, 3 lần, không chặn): p50 4.729 µs, p95 4.995 µs.

## Bổ sung (2026-11-07): PR drive-loop (vòng vị trí trong driver)

- **Thuật toán `drive_position`.** `Algorithm::kDrivePosition = 2`: host (ControlLoop 1 kHz) gửi
  điểm đặt vị trí, feed-forward vận tốc và feed-forward mômen (`cobot.drive.torque_feedforward`);
  driver mô phỏng đóng vòng vị trí trên encoder động cơ ở `cobot.drive.rate_hz` (4 kHz),
  `k_p = J ω²`, `k_d = 2 ζ J ω`, `J = diag(M(home_q))` (docs/derivations/control.md "Vòng vị trí
  trong driver"). Hold cũng đi qua driver.
- **Thay đổi HAL (câu hỏi C5).** `JointCommand<N>` có thêm `mode` (`DriveMode::kTorque` mặc định,
  `kPosition`), `q`, `qd` và `seq`; mọi nơi tạo và dùng lệnh mômen giữ nguyên `tau`.
  `ImperfectJointInterface` chỉ áp servo hold và trễ lệnh lên gói điểm đặt; phần tạo dạng mômen
  chuyển sang `imperfect::ActuatorStage` (cùng biểu thức, cùng thứ tự, đầu ra từng bit như cũ).
- **Điểm tích hợp: driver nằm giữa bus và plant, trong `SimJointInterface::advance()`.** (1) Chỉ
  chủ plant chạy được nhiều bước con mỗi chu kỳ (cả jitter và overrun). (2) Hiệu ứng bus của
  profile (servo hold, mất gói, burst trễ, trễ lệnh) nằm trong decorator và phải áp lên gói điểm
  đặt, nên driver đứng sau decorator. (3) Driver thật đọc thẳng encoder động cơ, không qua trễ cảm
  biến và servo hold của host. (4) Khi host trễ (overrun), driver vẫn điều chỉnh như trên phần cứng.
  (5) Các luật mômen không gắn driver nên chạy giống hệt từng bit. Đã loại: driver trong
  `ControlLoop` (chạy ở tốc độ host trên phép đo trễ, tức đường mômen từ host), trong `SimPlant`
  (plant chỉ là vật lý), một lớp decorator mới (không chia bước con được plant).
- **Chỉ số mômen.** Trong lần chạy `drive_position`, `tau` của Run CSV chỉ là feed-forward gửi
  kèm điểm đặt; `peak_tau_ratio`, `sat_time_s` và `rms_tau_ratio` của tóm tắt C++ dùng mômen đặt
  vào động cơ `taum` (PD của driver cộng feed-forward). Khâu chấp hành của driver chạy với chu kỳ
  bước con thực `h` (giới hạn slew theo `h` cả khi chu kỳ bị jitter).
- **Host.** Engine bench (run, montecarlo, sweep) và `timing` gắn driver; `cobot_bench serve` và
  `cobot_controller` từ chối `drive_position` (câu hỏi C4, chưa có mô hình driver ở đó).
- **`config/budget_xarm7_drive.yaml`.** `budget_xarm7.yaml` cộng `control.algorithm:
  drive_position`, khối `drive` (4 kHz, `ω = 80` rad/s sau khi quét 20..150, `ζ = 1`, lọc 500 Hz,
  `inverse_dynamics`) và `safety.hal_timeout_ms: 20` (câu hỏi C2: với 5 ms, một gói 250 Hz bị mất
  hay một burst 5 chu kỳ đã gây `state_timeout` với mọi luật; 20 ms = 5 chu kỳ servo, driver giữ
  điểm đặt cuối trong lúc đó). E2E-12 giữ nguyên kết luận mômen từ host; E2E-14 là cùng profile với
  vòng trong driver.
- **Số liệu budget, seeds 1-3.** Trước (computed torque từ host, E2E-12): hold trên mọi seed. Sau
  (`drive_position`, `ω = 80`): không hold; `final_pos_err_m` 2.9e-4 / 1.20e-3 / 3.45e-3,
  `final_rot_err_rad` 4.1e-3 / 2.5e-3 / 8.9e-3, `rms_joint_err_rad` 3.4e-3 / 1.2e-3 / 4.0e-3, trong
  biên 20 %. Biên độ dao động sau đích (`q − q_ref` trên 0.5 s) khoảng 3e-4 rad ở cả hai cửa sổ,
  bằng mức nhiễu đo (`q_noise_std_rad` 5e-5, encoder 17 bit), không phải dao động của vòng.
