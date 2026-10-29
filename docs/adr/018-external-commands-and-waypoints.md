# ADR-018: Lệnh ngoài và quỹ đạo nhiều điểm (action trong controller, panel RViz2)

- Trạng thái: chấp nhận (2026-10-30)
- Ngày: 2026-10-30
- Liên quan: thiết kế `docs/design/2026-10-29-waypoint-gui-design.md`; ADR-003, ADR-007,
  ADR-008, ADR-011, ADR-012

## Bối cảnh

MVP bám một quỹ đạo cố định đọc từ cấu hình, khởi động là chạy. Chủ dự án muốn đưa vào một
điểm cuối hoặc danh sách điểm, xem trước, chạy, dừng, rồi xem sai số. Lệnh từ bên ngoài cũng là
điều kiện cần cho Pi5 sau này. Ràng buộc giữ nguyên: lõi không ROS, vòng 1 kHz không cấp phát,
không ngoại lệ, các test đã đóng băng không đổi.

## Quyết định

- **Lõi:** `planning::WaypointTrajectory` (tối đa 32 điểm, lưu cố định, lướt C1 qua điểm giữa,
  dừng ở điểm `stop` và điểm cuối, hướng nội suy trên SO(3)); một điểm `stop` trùng
  `CartesianTrajectory`. `pipeline::validate_path` chạy CLIK ngoại tuyến trên đúng mô hình của
  controller, báo `unreachable`, `joint_limit`, `joint_velocity`, `torque` theo từng điểm và chỉ
  kéo dài đoạn tự tính thời lượng (×1.25 mỗi lần, tối đa ×3).
- **Vòng điều khiển:** `command_source: config | external`. Chế độ `config` giữ nguyên đường
  code MVP. `external` vào `kStandby` (CLIK + computed torque giữ một tư thế), `load` chỉ nhận ở
  standby, dừng là co giãn thời gian dọc đường đã kiểm tra (`r = 1 − smoothstep`), `reset` thoát
  chốt an toàn khi nguyên nhân đã hết.
- **ROS:** action `FollowWaypoints`, service `PreviewWaypoints` và `~/reset` sống trong
  `cobot_controller` theo mẫu `joint_trajectory_controller`. Callback chạy ngoài luồng thời gian
  thực; quỹ đạo vào `update()` qua `realtime_tools::RealtimeBuffer`; hủy goal và reset là cờ
  atomic; snapshot trạng thái lấy bằng `try_lock` nên `update()` không bao giờ chờ. Số liệu kết
  quả (sai số cuối, RMS, đỉnh mô-men) tích lũy trong `update()` không cấp phát.
- **Giao diện:** panel RViz2 `cobot_gui/WaypointPanel` (C++/Qt) với interactive marker; mô hình
  dữ liệu của bảng tách khỏi Qt để test không cần màn hình. `sim.launch.py rviz:=true`.
- **File điểm** YAML dùng chung cho GUI, `cobot_bench run --waypoints` và E2E (E2E-10 bench,
  E2E-11/11b Gazebo).

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Node ROS riêng nhận lệnh, gửi quỹ đạo sang controller qua topic | Tách biệt | Thêm một đường dữ liệu và độ trễ; kiểm tra và chạy dễ dùng hai mô hình khác nhau |
| MoveIt 2 + MoveIt Servo | Có sẵn IK, tránh va chạm, GUI | Lõi không còn tự viết; quỹ đạo khớp thay vì Cartesian; nặng cho Pi5 |
| GUI web (rosbridge + JS) hoặc Qt riêng | Không cần RViz2 | Phải tự vẽ robot và marker 3D; RViz2 đã có sẵn trong stack |
| Phanh theo twist khi Dừng | Đơn giản | Rời khỏi đường đã kiểm tra, có thể vào vùng chưa xét giới hạn |

## Hệ quả

- Xem trước và Chạy dùng cùng hàm, cùng đầu vào, cùng mô hình, nên không thể khác nhau.
- `ControlLoop` có thêm accessor `last_record()` (chỉ đọc) để controller tích lũy số liệu kết quả.
- E2E-11b dùng ngưỡng đứng yên 0.03 rad/s thay vì 0.01 trong thiết kế: vận tốc ghi trong CSV là
  vận tốc đo (encoder lượng tử, vi phân, lọc 100 Hz), nhiễu lúc đứng yên có độ lệch chuẩn
  0.0047 rad/s; 0.03 ≈ 6σ. Chờ chủ dự án duyệt.
- Chưa có test mức Gazebo cho service preview và reset; nằm trong PR bổ sung test.
