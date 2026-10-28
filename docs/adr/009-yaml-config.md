# ADR-009: Cấu hình: YAML là nguồn tham số duy nhất của lõi

- Trạng thái: đề xuất
- Ngày: 2026-10-29
- Liên quan: thiết kế mục 5.6, 11.1, 11.2; ADR-002, ADR-015

## Bối cảnh

Bản cũ hard-code gains trong code Python và đường dẫn tham số ngoài repo. Lõi mới chạy dưới
Gazebo và bench, cả hai phải dùng cùng tham số; mọi số chỉnh được phải có đơn vị và được kiểm tra
trước khi vòng điều khiển chạy.

## Quyết định

- `config/default.yaml` (khoá gốc `cobot:`) là nguồn tham số duy nhất của lõi: `urdf_path`,
  `base_link`, `tip_link`, `gravity_b`, `control_rate_hz`, `deterministic`, `home_q_deg`,
  `trajectory.{goal_position_m, goal_rpy_deg, duration_s, hold_s}`,
  `clik.{kp, kd, k_null, k_damp, lambda_max, cond_threshold}`, `computed_torque.{kp, kd}`,
  `safety.{soft_margin_deg, hal_timeout_ms}`, `log.output_dir`.
- Profile không hoàn hảo trong `config/imperfections/<profile>.yaml`, sweep trong
  `config/sweeps/<tên>.yaml` (ADR-015).
- **Chỉ `pipeline` parse YAML** (yaml-cpp): `load_config(path) → Result<Config>`,
  `load_imperfections(path) → Result<ImperfectionConfig>`. Các module khác nhận struct.
- Khoá lạ là lỗi. Mọi giá trị được kiểm tra khoảng và độ dài vector lúc nạp; thông điệp lỗi nêu
  tên khoá đầy đủ (ví dụ `cobot.computed_torque.kp[3]`).
- Góc trong YAML dùng hậu tố `_deg`; `pipeline` đổi sang radian. `home_q_deg` phải nằm trong giới
  hạn URDF (lưu ý `joint_4` có khoảng `[-180°, -22.9°]`, nên `q = 0` không hợp lệ); pytest kiểm tra
  biên ≥ 5°.
- `ros2_node` nạp cùng file qua tham số ROS `config_path`, không chép giá trị sang file tham số ROS.
- Hành vi mới đi sau một khoá YAML có mặc định giữ hành vi cũ.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Tham số ROS 2 (`.yaml` của `ros2 param`) | Có sẵn công cụ đổi lúc chạy | Bench và test không có ROS; trái ADR-003 |
| JSON | Có sẵn trong Python | Không có chú thích; tham số của bản cũ đã mất vì JSON ngoài repo |
| Hằng số trong code | Nhanh | Mỗi lần chỉnh gains phải biên dịch lại; không truy được giá trị đã chạy |
| Mỗi module tự parse YAML | Gần nơi dùng | yaml-cpp lan khắp lõi; khó kiểm tra khoá lạ toàn cục |

## Hệ quả

Mọi lần chạy có thể tái lập từ một file cấu hình cộng một profile cộng một seed. Thêm một tham số
cần sửa ba nơi (YAML, struct, kiểm tra), và test cấu hình phải được cập nhật trong PR có nhãn
`test-change` nếu khoá đổi tên.
