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

## Bổ sung (2026-11-08): PR hardware-catalog (model phần cứng và nhiệm vụ)

- **"Giới hạn khớp" của model (điểm chưa rõ 9).** Chỉ là hai mức mômen `limits: {rated_torque_nm,
  peak_torque_nm}`, áp vào `cobot.limits`; giới hạn vị trí và vận tốc vẫn lấy từ URDF trong giai đoạn
  C (câu hỏi C10). `pipeline::config_for` = file bộ điều khiển gốc + `config.control.algorithm` +
  limits + overrides của thuật toán, qua `load_config_with`. `pd_gravity` dùng khoá `kp`, `kd` tường
  minh cùng `ω` với computed torque của model (chưa có khoá ảo). `drive_position` chỉ có ở budget, là
  đúng phần khác nhau giữa `budget_xarm7_drive.yaml` và `budget_xarm7.yaml`.
- **Nguồn gốc (điểm chưa rõ 10, câu hỏi C8).** Văn bản `source` của profile không đọc được theo từng
  khoá và không có khoảng, nên model có `provenance`: mỗi khoá đang bật một mục `published`, `derived`
  hoặc `assumed`, thiếu thì model không nạp được; mục cho khoá không bật, mục trùng, loại lạ cũng là
  lỗi. `derived` và `assumed` cần đúng một trong `range` (tuyệt đối, khoá mà profile ghi là một số) và
  `scale` (hệ số nhân mọi phần tử, khoá mà profile ghi là danh sách, chứa 1, cận dưới >= 0); khoá bool
  và lựa chọn không nhận cả hai. Khoá đang bật do `pipeline::active_imperfection_keys` quyết định (quy
  tắc trong header và `docs/conventions.md`). Loại lấy từ văn bản `source` và ADR-019, văn bản không
  nêu tên khoá thì là `assumed`. Các khoảng là phán đoán kỹ thuật, chờ chủ dự án duyệt.
- **"Thời gian chu kỳ" (điểm chưa rõ 11).** `cycle_time_s` là thời gian ổn định: từ dòng đầu tiên đến
  khi sai số vị trí và góc của TCP vào và ở lại trong ngưỡng cuối, xét từng lần chạy. PR này chỉ lưu
  giá trị; `verify` đo nó. Ngưỡng của hai nhiệm vụ là mức E2E-12, đang ở dạng đề xuất (câu hỏi C13).
- **Tải (điểm chưa rõ 12, câu hỏi C9).** Plant luôn mang tải của nhiệm vụ (`model.payload_*`). Với
  `known: true` (mặc định) bộ điều khiển cũng được báo qua mục tuỳ chọn mới `cobot.payload: {kg,
  offset_m, radius_m}`; `known: false` thì không. `pipeline::controller_model` nhập tải vào khâu cuối,
  nên mọi host và bước kiểm đường đều thấy; phép nhập nằm trong một hàm `imperfect::merge_payload`,
  dùng chung với `perturb_model`. 0 kg không đi qua phép nhập nên mô hình giữ nguyên từng bit.
- **Không đổi bước nóng.** Không đổi `control_loop`, không thêm cột Run CSV. Sha256 của năm Run CSV
  (E2E-01, 02, 03, 04, 10) bằng nhau giữa main và nhánh trên cùng một máy.

## Bổ sung (2026-11-08): PR auto-tune (tự chỉnh hệ số)

- **Một hệ số cho mỗi luật.** `computed_torque`: `config.computed_torque.omega`; `pd_gravity`: khoá ảo
  mới `config.pd_gravity.omega` (`kp = ω²`, `kd = 2ω`, > 0, đặt cùng `kp` hoặc `kd` của mục đó là lỗi),
  xử lý ở đúng chỗ của khoá ảo cũ; `drive_position`: `config.drive.omega_rad_s`. Mức mặc định 20..150
  bước 10 (luật mômen), 20..200 bước 10 (driver); hạt giống mặc định 1..10. `with_gain` bỏ những gì
  model đã đặt cho hệ số đó (cùng khoá, và `kp`/`kd` của mục khi là khoá ảo) rồi thêm mức cần chạy.
- **Dao động sau đích.** Lấy đúng đại lượng của `tests/unit/loop_drive_profiles`: max theo khớp của
  biên độ đỉnh-đỉnh của `q - q_ref` trong 0,5 s đầu và 0,5 s cuối của đoạn giữ `[đích, hết]`, đích là
  thời lượng quỹ đạo (cả quỹ đạo waypoint); đoạn giữ dưới 1 s thì dùng hai nửa. Lần chạy bị giữ
  (holding) hoặc không có dòng sau đích cho NaN. "Tắt dần": cuối <= đầu, hoặc cuối <= sàn nhiễu
  `5e-4` rad (lượng tử encoder và nhiễu lúc đứng yên, đo được 2,9e-4 đến 4,1e-4 rad ở PR drive-loop);
  NaN không tắt dần. Các trường mới (`run_time_s`, `swing_first_rad`, `swing_last_rad`) chỉ nằm trong
  `RunSummary`, không thêm cột Summary CSV hay Run CSV.
- **Chấp nhận được và chọn.** Một mức chấp nhận được khi không seed nào bị giữ, `sat_time_s / run_time_s`
  lớn nhất < 0,05 và mọi seed tắt dần; chọn mức có trung vị `rms_joint_err_rad` nhỏ nhất, bằng nhau
  thì lấy mức thấp hơn. Không có mức nào thì không chọn, và lý do đếm số mức trượt từng tiêu chí.
- **Bão hoà của lần chạy driver (điểm chưa rõ 7).** `summarize` đã tính `sat_time_s` trên `taum` so
  với P cho `drive_position` từ PR drive-loop (`TorqueSource::kMotor`), nên PR này không đổi gì; giá
  trị của computed torque giữ nguyên.
- **Chạy song song.** Các lần chạy đi qua một hàng đợi nhỏ (`bench/work_queue.hpp`), kết quả lưu theo
  (chỉ số mức, chỉ số seed), nên `--jobs 1` và `--jobs 2` cho cùng từng byte `tune.csv`. Đã đọc đường
  chạy từ `run_profile` xuống (nạp YAML và waypoint, URDF, Recorder, SimPlant, các RNG của
  imperfection, metrics): không có biến `static` hay toàn cục thay đổi được; mỗi lần chạy sở hữu
  đối tượng của nó.
- **Thời gian và bộ nhớ đo được** (WSL Ubuntu 24.04, x86_64, 12 CPU, Release, `jobs` mặc định 8,
  `reference_move`, 10 seed): premium_fr3 CT 14 mức 97 s, mid_kr1205 CT 97 s, budget_xarm7 CT 97 s,
  budget_xarm7 drive 19 mức 244 s; RSS đỉnh 104 MB mỗi lần tune (dòng của một lần chạy bỏ đi ngay khi
  đã tóm tắt). Mức được chọn: premium CT 50, mid CT 40, budget CT không có (mọi mức bị giữ), budget
  drive 50.

## Bổ sung (2026-11-08): PR verify (6a, kết luận ba mức)

- **Tách PR 6.** PR 6a (này): kết quả từng lần chạy, thời gian ổn định, khoảng Wilson, kết luận, Monte
  Carlo sau bước tune, báo cáo JSON + MD, lệnh `cobot_bench verify`, E2E-15. PR 6b (`verify-analysis`):
  độ nhạy theo từng giả định, nguyên nhân và gợi ý, bảng so sánh nhiều model/luật (G16), E2E-16/17.
  Chỗ nối cho 6b: `assumptions` liệt kê các mục provenance `assumed`/`derived` với `sensitivity: "not
  run"`, `causes` và `suggestions` để trống.
- **Thời gian ổn định (điểm chưa rõ 11).** `cycle_time_s` đọc là thời gian từ dòng đầu tiên tới lúc TCP
  vào và ở lại trong ngưỡng cuối (`final_pos_err_m`, `final_rot_err_deg`) quanh **đích cuối**, tức tư thế
  mong muốn `T_b_d` của dòng đánh giá (dòng của `final_pos_err_m`), trên mọi dòng từ lúc đó tới dòng đánh
  giá. Lần vào cuối cùng được tính; ra ngoài ở dòng đánh giá hoặc lần chạy bị giữ cho NaN; ở trong ngay
  từ dòng đầu cho 0. Chỉ nằm trong `RunSummary::settle_time_s` (không thêm cột CSV); chỉ tính khi có
  ngưỡng (`SettleThresholds` trên `run_with` và `summarize`), nên tune và các Run CSV không đổi.
- **Đạt từng lần (điểm chưa rõ 13).** Không bị giữ, và sai số vị trí, góc (độ), rms khớp, thời gian ổn
  định đều `<=` yêu cầu; NaN là trượt. Giá trị tệ nhất lấy trên các lần không bị giữ.
- **Wilson và kết luận (14, C11, C14).** FAIL khi cận trên Wilson của tỉ lệ đạt < `target_pass_rate`
  hoặc cận dưới Wilson của tỉ lệ giữ > `max_hold_rate`; PASS khi không ở chế độ nhanh, cận dưới tỉ lệ
  đạt `>= target_pass_rate`, tỉ lệ giữ (điểm) `<= max_hold_rate` và mọi giá trị tệ nhất `<=` giới hạn
  biên `(1 - margin_frac) * ngưỡng`; còn lại MARGINAL. **Chế độ nhanh** (`--quick` hoặc dưới 100 lần
  chạy) không bao giờ PASS, vẫn có thể FAIL. Một hàm `bench::verdict` dùng chung cho CLI và studio.
- **Chạy song song (15, C7).** Các lần chạy đi qua `run_queue`, kết quả lưu theo chỉ số seed; báo cáo
  không chứa thời gian thực, nên `--jobs 1` và `--jobs 3` cho cùng từng byte `report.json` và
  `report.md`. Tiến độ và huỷ chỉ được đọc giữa các lần chạy.

## Bổ sung (2026-11-08): PR verify-analysis (6b, độ nhạy, nguyên nhân, so sánh)

- **Độ nhạy của từng giả định (điểm chưa rõ 16, C12).** Mỗi mục provenance loại `assumed` hoặc
  `derived` có `range` chạy hai biến thể của hồ sơ: khoá ở `lo` và ở `hi`; có `scale` thì mọi phần tử
  của giá trị trong hồ sơ nhân với `lo` và với `hi` (đầu bằng đúng 1 vẫn chạy); mọi khoá khác giữ
  nguyên. Khoá bool và khoá lựa chọn ghi `"not varied"`. Mỗi đầu chạy `min(sensitivity_seeds, runs)`
  seed (mặc định 20) từ `seed0` với hệ số đang dùng; mốc so sánh là chính các seed đó của Monte Carlo
  chính (không chạy thêm). `"may change"` khi ở một đầu tỉ lệ đạt nằm phía bên kia `target_pass_rate`
  so với tỉ lệ đạt chính (bằng đúng mục tiêu tính là phía trên, như dấu `>=` của PASS), hoặc khi một đầu
  có lần bị giữ mà mốc không có; còn lại `"stable"`. Bật mặc định, tắt bằng `--no-sensitivity` và luôn
  tắt ở chế độ nhanh (báo cáo ghi `"not run"`).
- **Nguyên nhân và gợi ý (điểm chưa rõ 17), chỉ khi KHÔNG ĐẠT.** Mỗi nhóm `sensor, actuator,
  friction, model, timing, comm, transmission` có khoá hoạt động (quy tắc của
  `active_imperfection_keys`, `model.payload_*` thuộc nhiệm vụ) chạy với các khoá đó ở giá trị tắt
  (0, `false`, `link`; `sensor.qd_filter_hz` và `transmission.ratio` giữ nguyên vì bộ nạp không nhận
  0) trên `min(ablation_seeds, runs)` seed. Nhóm mà khi bỏ đi số lần đạt tăng nhiều nhất là nguyên
  nhân chính (hoà thì liệt kê hết); không nhóm nào làm tăng thì báo cáo ghi "no single group explains
  the failure". Kèm số lần bị giữ theo lý do của Monte Carlo chính. Bảng gợi ý cố định, theo thứ tự:
  1. luật mô-men (computed_torque, pd_gravity), `comm.servo_rate_hz > 0`, và `state_timeout` +
     `overspeed` chiếm quá nửa số lần bị giữ -> cần vòng vị trí trong driver (drive_position);
  2. nguyên nhân chính `comm` -> tăng tần số servo hoặc giảm mất gói;
  3. `sensor` -> encoder mịn hơn hoặc ít nhiễu cảm biến hơn;
  4. `transmission` -> truyền động cứng hơn hoặc ít độ rơ hơn;
  5. `friction` -> nhận dạng và bù ma sát;
  6. tune không tìm được hệ số chấp nhận được -> mở rộng các mức hoặc đổi luật.
- **report.json.** Schema vẫn `1`: khoá của 6a giữ thứ tự và nghĩa; khoá mới chỉ nằm trong
  `assumptions[]` (`base`, `low`, `high`) và `causes` (khi KHÔNG ĐẠT là đối tượng `{groups,
  hold_reasons}`, còn lại `[]`). Bộ chấm E2E-15 chỉ đọc các khoá cấp trên.
- **So sánh nhiều model và thuật toán (G16).** `--hardware a,b` và `--algorithm x,y` lấy tích Đề-các,
  cặp mà model không liệt kê ghi `not listed`, không có cặp nào thì thoát 1 trước mọi lần chạy. Các
  cặp chạy lần lượt, mỗi cặp với `--jobs` (`bench::compare`, studio dùng lại). Hơn một cặp: mỗi cặp ghi
  `<out>/<model>__<algorithm>/` đúng từng byte như một lần gọi đơn, thêm `comparison.json` và
  `comparison.md`; một cặp giữ bố cục cũ.
- **Seed (điểm chưa rõ 18).** E2E-17 chạy lại E2E-16 trên seed 101..200 và mong cùng kết luận.
- **Thời gian nightly.** E2E-16 đo trên máy cục bộ với `--jobs 4`: 7210 s (khoảng 120 phút), RSS tối đa 56 MB;
  E2E-17 cùng khối lượng trên seed khác. Vì vậy hai kịch bản có tag riêng `verify-nightly` và chạy
  trong job `verify` của nightly, mỗi kịch bản một runner song song (giới hạn 300 phút); job cũ giữ
  120 phút. E2E-16 chạy độ nhạy với 10 hạt giống (`sensitivity_seeds: 10`), E2E-17 chỉ so kết luận
  nên tắt độ nhạy (`no_sensitivity: true`). Tag `nightly` vẫn chỉ gồm Monte Carlo và quét như test
  của nó quy định.

## Bổ sung (2026-11-08): PR studio-verify (tab Kiểm nghiệm của cobot_studio)

- **Một bộ ghi cho CLI và studio.** Bước nạp và kiểm đầu vào (`bench::load_verify_inputs`: task,
  từng model và file đầu vào, cặp) và bước ghi "tất cả hoặc không" (`bench::write_verify_outputs`:
  `report.{json,md}`, `tune.csv`, `comparison.{json,md}` qua file tạm rồi đổi tên; một cặp thì
  ghi thẳng vào `--out`) chuyển từ `main.cpp` vào thư viện bench (`src/bench/verify_outputs.hpp`).
  `cobot_bench verify` và job của studio (`cobot/studio/verify_job.hpp`, `run_verify_job`) gọi
  cùng hai hàm và cùng `bench::compare`, nên cùng thông báo lỗi và cùng từng byte; các test CLI
  của #81/#82 giữ nguyên và vẫn qua.
- **Hủy.** Job truyền cờ hủy của mình vào `VerifyOptions::cancel`; bộ ghi chỉ chạy sau mọi lượt
  chạy, nên hủy ở bất kỳ lúc nào trả "cancelled" và không có file nào. Thư mục mặc định là
  `<work_dir>/verify_NNNN/` (số trống đầu tiên), không bao giờ là repo.
- **Kết quả chỉ đọc.** Theo phản hồi của chủ dự án (2026-11-07: ô hiển thị trông như ô nhập), vùng
  kết quả của tab chỉ có QLabel, bảng `NoEditTriggers` không khung focus và một nhãn kết luận tô
  màu (xanh PASS, đỏ FAIL, hổ phách MARGINAL); không QLineEdit hay QSpinBox. Tab đọc số từ các
  `ComparisonRow`, không đọc lại file MD.
