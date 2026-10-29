# ADR-011: Mô phỏng: Gazebo Harmonic qua `gz_ros2_control`, effort, bước 1 ms

- Trạng thái: đề xuất, chờ bàn
- Ngày: 2026-10-29
- Liên quan: thiết kế mục 3, 5.5, 11.3 (E2E-07), 11.5; D4 của chủ dự án; ADR-005, ADR-012

## Bối cảnh

Bản cũ chạy Gazebo Classic với ROS 1, đã hết hỗ trợ. WSL Ubuntu 24.04 của máy dev đã có ROS 2
Jazzy và Gazebo Sim 8.15.0 (Harmonic) chạy headless được (D4). Bộ điều khiển dùng mô-men, nên
Gazebo phải nhận lệnh effort và chạy cùng nhịp 1 kHz với vòng điều khiển. Mesh STL tổng 33 MB.

## Quyết định

- Gói ament `cobot_sim` trong `sim/`: `urdf/cobot30_5.urdf.xacro` = URDF đã sửa (ADR-005) + khối
  `<ros2_control>` (`gz_ros2_control/GazeboSimSystem`; mỗi `joint_1..joint_7`: command `effort`,
  state `position`, `velocity`, `effort`) + plugin `gz_ros2_control-system`.
- World `worlds/empty.sdf`: `max_step_size 0.001`, `real_time_factor 1`, mặt sàn, robot gắn cố
  định vào world.
- `config/controllers.yaml`: `update_rate: 1000`; `joint_state_broadcaster`; `effort_passthrough`
  (`JointGroupEffortController`) chỉ cho smoke test; `cobot_ros2/CobotController` thêm ở giai
  đoạn `ros2-controller`.
- `launch/sim.launch.py`: `gui:=false` mặc định (headless `-s`), `robot_state_publisher`, spawn từ
  `/robot_description`, cầu `/clock`, spawner controller.
- Mesh giữ nguyên cho visual. Mesh URI là `package://cobot_sim/meshes/`. Không dùng Git LFS.
- Kiểm tra không cần Gazebo chạy: `tools/ci/check_sim_assets.sh` (xacro → `check_urdf` →
  `gz sdf -p`). Smoke test `tools/wsl/sim.sh smoke`: hai controller `active`, `/joint_states` có
  7 tên, cánh tay rơi theo trọng lực khi mô-men bằng 0.
- CI: job `gazebo` (image ROS 2 Jazzy + Gazebo amd64) build colcon và chạy E2E-07. Điều này thay
  câu "không có job ROS/Gazebo trong CI" ở mục 6 của thiết kế.

**Câu hỏi mở cho chủ dự án:** collision dùng chính mesh STL hay hình khối đơn giản? Mặc định
trong lúc chờ: dùng mesh; đổi sang hình khối nếu Gazebo không giữ được hệ số thời gian thực 1
với bước 1 ms.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Collision bằng mesh (mặc định) | Không phải làm thêm hình; hình dạng đúng | Đang chọn; có thể chậm, va chạm mesh–mesh kém ổn định |
| Collision bằng hình khối (trụ, hộp) | Nhanh, ổn định | Phải dựng tay 9 hình; MVP chưa có va chạm nên lợi ích chưa thấy |
| Không có collision | Nhanh nhất | Robot xuyên sàn và tự xuyên khi demo; khó giải thích |
| Gazebo Classic | Bản cũ đã dùng | Hết hỗ trợ; không có trên Jazzy |
| MuJoCo / PyBullet | Nhanh, chính xác | Không nối được `ros2_control` sẵn; ngoài môi trường đã dựng (D4) |
| Lệnh vị trí/vận tốc | Ổn định hơn | Bỏ qua chính động lực học mà đồ án muốn kiểm chứng |

## Hệ quả

Demo Gazebo dùng đúng URDF với bộ điều khiển. Bước vật lý 1 ms trùng nhịp điều khiển nên không
có jitter topic. Đổi lại, kết quả Gazebo phụ thuộc bộ giải vật lý của nó; so sánh với `SimPlant`
(E2E-01 so với E2E-07) cho biết chênh lệch đó.

## Kết quả MVP (2026-10-29)

E2E-07 đạt trên Gazebo Harmonic 8 qua gz_ros2_control, giao tiếp effort, bước 1 ms: sai số vị trí cuối 0.000 mm, RMS khớp 3.7e-6 rad. Chạy 17 s thời gian mô phỏng mất khoảng 23 s thời gian thực trong WSL. Job CI gazebo chạy trong image ros:jazzy. Vẫn chờ bàn: có cần thêm kịch bản Gazebo với nhiễu (hiện chỉ có profile ideal) hay không.
