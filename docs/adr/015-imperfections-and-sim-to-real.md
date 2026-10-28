# ADR-015: Mô hình không hoàn hảo cấu hình được và phân tích độ nhạy sim-to-real

- Trạng thái: đề xuất
- Ngày: 2026-10-29
- Liên quan: thiết kế mục 11.1, 11.2; ADR-005, ADR-009, ADR-012, ADR-013, ADR-014

## Bối cảnh

Chủ dự án yêu cầu: sửa đổi được nhiễu đầu vào, báo cáo sai số, và đánh giá độ chính xác của cảm
biến và động cơ ảnh hưởng thế nào tới kết quả, để chuẩn bị chuyển từ mô phỏng sang robot thật. Mô
phỏng lý tưởng (đo hoàn hảo, mô-men chính xác, mô hình đúng) cho kết quả không nói gì về phần
cứng. Cần một lớp gây lỗi có kiểm soát, tái lập được, dùng chung cho bench và Gazebo.

## Quyết định

**Module `core/imperfect`** (namespace `cobot::imperfect`, phụ thuộc `hal`, `dynamics`, `common`).
Ngẫu nhiên dùng `std::mt19937_64` seed từ YAML; không cấp phát trong `step`.

| Nhóm | Tham số (mỗi khớp, trừ khi ghi khác) | Áp vào |
| --- | --- | --- |
| Cảm biến vị trí | `encoder_bits`, `q_noise_std_rad`, `q_offset_std_rad` (rút một lần mỗi lần chạy) | đọc `q` |
| Cảm biến vận tốc | `qd_source`: `true` hoặc `diff` (sai phân `q` đã lượng tử + lọc `qd_filter_hz`), `qd_noise_std_rad_s` | đọc `q̇` |
| Trễ đo | `sensor_delay_cycles` | đọc |
| Động cơ | `torque_scale_std` (rút một lần), `torque_ripple_frac` + `ripple_periods_per_rev`, `torque_cmd_resolution_nm`, `motor_time_constant_s`, `command_delay_cycles` | ghi `τ` |
| Ma sát chưa mô hình | `coulomb_nm`, `viscous_nm_s_rad` | plant |
| Sai lệch mô hình | `link_mass_scale_std`, `com_shift_std_m`, `payload_kg` ở đầu cuối | plant |
| Thời gian | `period_jitter_std_us` | bước plant trong bench |

- `ImperfectJointInterface<N>`: decorator bọc bất kỳ `hal::JointInterface<N>` (cảm biến khi
  `read`, động cơ khi `write`). Bench bọc `SimJointInterface`; plugin `ros2_control` bọc
  state/command bằng cùng lớp, nên một profile cho cùng nhiễu ở hai nơi.
- `perturb_model(model, mismatch, rng)`: plant dùng mô hình bị nhiễu, bộ điều khiển dùng mô hình
  danh định. Với Gazebo, `tools/fix_urdf_inertia.py` có tuỳ chọn sinh URDF nhiễu từ cùng profile.
- Ma sát và trễ động cơ nằm trong `SimPlant` (bench) và trong decorator (Gazebo, xấp xỉ theo `q̇`
  đo).
- Profile `config/imperfections/{ideal,realistic,pessimistic}.yaml`, khoá gốc `imperfections:`,
  nhóm `sensor, actuator, friction, model, timing`; mỗi giá trị là số (mọi khớp) hoặc danh sách 7;
  mỗi nhóm có `source:`, số chưa có nguồn ghi `assumed`. `ideal` = tất cả bằng 0.

**Báo cáo và độ nhạy.**

- `cobot_bench run | montecarlo | sweep`. `sweep` thay đổi một yếu tố mỗi lần (5–7 mức, các tham
  số khác ở profile nền), mô tả trong `config/sweeps/<tên>.yaml`. Mỗi lần chạy một dòng Summary
  CSV (`docs/conventions.md`).
- `eval/cobot_eval/report.py`: báo cáo Markdown + PNG một lần chạy, thoát mã khác 0 khi trượt.
- `eval/cobot_eval/sensitivity.py`: đường "mức → chỉ số", biểu đồ tornado, và **bảng yêu cầu phần
  cứng**: với mỗi tham số, mức xấu nhất mà chỉ số vẫn đạt G3 (ví dụ "encoder ≥ 16 bit", "trễ
  lệnh ≤ 2 chu kỳ"). Bảng nằm trong `docs/results/`.
- Monte Carlo: tỷ lệ đạt, phân vị 50/95 của từng chỉ số.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Nhiễu trong từng module (FK, controller) | Không cần lớp mới | Lẫn lỗi vào code được kiểm chứng; không bật tắt được một chỗ |
| Plugin nhiễu riêng của Gazebo | Gần vật lý | Bench không dùng được; hai mô hình nhiễu lệch nhau |
| Chỉ Monte Carlo, không sweep | Một chế độ | Không chỉ ra tham số nào quyết định; không ra được bảng yêu cầu phần cứng |
| Sobol / độ nhạy toàn cục | Bắt tương tác giữa tham số | Tốn hàng nghìn lần chạy; một yếu tố mỗi lần đủ cho bảng yêu cầu |
| `std::random_device` | Không cần seed | Không tái lập; E2E-02/03 cần seed cố định |

## Hệ quả

Bộ điều khiển không đổi; mọi lỗi nằm ở biên HAL và plant. Kết quả sim-to-real của MVP là bảng yêu
cầu phần cứng có thể so với datasheet. Đổi lại, các số trong profile `realistic` phần lớn là
`assumed` cho đến khi có datasheet hoặc đo thật, và báo cáo phải ghi rõ điều đó.
