# ADR-012: Hai điểm cắm: `ros2_control` cho Gazebo, `hal::JointInterface` cho bench; Pi5 để mở

- Trạng thái: đề xuất, chờ bàn
- Ngày: 2026-10-29
- Liên quan: thiết kế mục 2 (G4), 5.5; ADR-003, ADR-008, ADR-011, ADR-014, ADR-015

## Bối cảnh

`ControlLoop` phải chạy với ba loại plant: Gazebo (qua `gz_ros2_control`), mô phỏng nội bộ
`SimPlant` (test, bench, không ROS), và sau này robot thật trên Pi5 với actuator EtherCAT. Bản cũ
nối các bước qua topic ROS, gây jitter và bốn tần số khác nhau.

## Quyết định

- **Điểm cắm 1, `ros2_control`:** `ros2_node/` là một `controller_interface::ControllerInterface`
  (`cobot_ros2/CobotController`). `update()` được `controller_manager` gọi ở 1 kHz, đọc state
  interface (`position`, `velocity`, `effort`) của 7 khớp, gọi `ControlLoop::step()`, ghi command
  interface `effort`. MVP: quỹ đạo lấy từ YAML, bắt đầu khi controller được kích hoạt; trạng thái
  và sai số publish 100 Hz. Sau MVP: topic `~/target_pose` (`geometry_msgs/PoseStamped`) qua
  realtime buffer.
- **Điểm cắm 2, `hal::JointInterface<N>`:** interface trong lõi (`read(JointState&)`,
  `write(const JointCommand&)`, trả `Status`). Hiện thực MVP: `pipeline::SimJointInterface`
  (bọc `SimPlant`) cho bench và test.
- Decorator `imperfect::ImperfectJointInterface<N>` bọc được cả hai đường: bench bọc
  `SimJointInterface`; plugin bọc state/command bằng cùng lớp (ADR-015).
- Lõi không biết đường nào đang dùng.

**Ghi chú hiện thực (feature `ros2-controller`):** gói `cobot_ros2` build lõi từ `../core` ngay
trong gói (`add_subdirectory`), không cần cài lõi riêng, nên một lệnh `tools/wsl/sim.sh build`
(và image `docker/ros.Dockerfile`) build đủ `sim/` và `ros2_node/`; `sim.sh tidy` chạy clang-tidy
trên `ros2_node/src` (job `gazebo`, vì job `lint` không có header ROS). Tham số: `config_path`,
`imperfections_path` (mặc định là bản cài trong `share/cobot_ros2/config`, kèm bản URDF ở
`share/cobot_ros2/sim/urdf` để `urdf_path` tương đối vẫn đúng), `joints`, `log_path` (để trống:
`results/<run-id>/run.csv` dưới thư mục làm việc của tiến trình controller manager, không dùng
`log.output_dir` vì đường đó tính theo thư mục của file config, tức nằm trong cây cài đặt colcon).
`sim.launch.py` nhận `config_path`, `imperfections_path`, `log_path` và chuyển cho spawner bằng
`--param-file` (file tạm, xoá khi launch tắt). `on_activate` đọc trạng thái đo được qua
`ImperfectJointInterface` rồi gọi `ControlLoop::start`; đọc lỗi thì kích hoạt thất bại, nên quỹ
đạo luôn bắt đầu từ tư thế thật; trước đó `gz_ros2_control` giữ khớp ở tư thế home
(`hold_joints`). Stamp của trạng thái = thời điểm kích hoạt (`get_node()->now()`) + số chu kỳ ×
chu kỳ điều khiển, không lấy tham số `time` của `update()`: `time` theo đồng hồ ROS (`/clock` khi
mô phỏng), có lúc đứng yên hàng trăm chu kỳ trong khi interface vẫn được đọc mỗi chu kỳ, và
`ControlLoop` sẽ vào chế độ giữ vì stamp cũ. Vì thế `cobot.control_rate_hz` phải bằng
`update_rate` của `controller_manager` (kiểm tra lúc configure). Trạng thái (`~/status`,
`std_msgs/String` là chế độ vòng) publish 10 Hz qua realtime publisher. Luồng ghi CSV flush sau
mỗi lô để dữ liệu còn nguyên khi Gazebo bị dừng. `tools/run_e2e.py` chờ ít nhất duration + hold +
5 s (đồng hồ thật) sau khi CSV xuất hiện, rồi chờ tiếp tới khi CSV phủ đủ duration + hold theo
thời gian mô phỏng (Gazebo có thể chạy chậm hơn thời gian thực), tối đa 3 lần mức chờ tối thiểu.

**Câu hỏi mở cho chủ dự án:** trên Pi5, chạy qua đường nào? Chưa chọn cho tới khi có phần cứng
(ADR-014); cả hai đường được giữ để không đóng lựa chọn nào.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Pi5 qua `ros2_control` + `ethercat_driver_ros2` | Cùng plugin với Gazebo; có sẵn driver CiA402 | Để mở: cần ROS 2 trên Pi5 RT, thêm tầng lịch trình của `controller_manager` |
| Pi5 qua binary trần + `JointInterface` EtherCAT tự viết (SOEM/IgH) | Ít tầng, kiểm soát thời gian tuyệt đối | Để mở: phải tự viết driver CiA402 và xử lý lỗi bus |
| Chỉ `ros2_control` (cả bench dùng mock hardware) | Một điểm cắm | Test và bench phải chạy `controller_manager`; trái ADR-003 |
| Chỉ HAL riêng (cả Gazebo qua plugin Gazebo tự viết) | Một điểm cắm, không ROS | Tự viết plugin Gazebo; mất hệ sinh thái `ros2_control`, RViz |
| Node topic như bản cũ | Quen | Jitter, không cùng nhịp với bước vật lý |

## Hệ quả

Bench và Gazebo dùng cùng `ControlLoop` và cùng profile nhiễu. Hai adapter phải được giữ mỏng và
cùng hành vi; E2E-01 (bench) và E2E-07 (Gazebo) dùng cùng quỹ đạo và ngưỡng để phát hiện lệch.
