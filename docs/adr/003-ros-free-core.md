# ADR-003: Lõi không ROS, adapter mỏng

- Trạng thái: chấp nhận (MVP, 2026-10-29)
- Ngày: 2026-10-29
- Liên quan: thiết kế mục 2 (phản mục tiêu), 5.1, 5.5; D1, D3 của chủ dự án; ADR-012

## Bối cảnh

Bản cũ trộn toán, điều khiển và giao tiếp ROS trong cùng node Python. Lõi mới phải chạy trong
Gazebo (qua ROS 2), trong bench không ROS, trong test đơn vị và sau này trên Pi5 có thể không có
ROS. Chủ dự án đã chốt lõi không ROS (D1) và phạm vi lần này là mô phỏng (D3).

## Quyết định

- `core/` không include bất kỳ header nào của ROS, Gazebo, KDL hoặc Pinocchio. Vi phạm là phản
  mục tiêu của thiết kế và bị review chặn.
- Mọi trao đổi với thế giới ngoài đi qua kiểu của lõi: `hal::JointState<N>`,
  `hal::JointCommand<N>`, `pipeline::Config`, `pipeline::LogRecord<N>`.
- `ros2_node/` (`cobot_ros2/CobotController`) là adapter mỏng: đọc state interface, gọi
  `ControlLoop::step()`, ghi command interface, publish trạng thái 100 Hz. Không có toán trong
  adapter.
- `bench/` là binary trần dùng cùng `ControlLoop` với `SimJointInterface`.
- Logger của lõi là interface; `ros2_node` nối nó vào logging của rclcpp.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Lõi là thư viện ROS 2 (rclcpp, message type trong API) | Ít code chuyển đổi | Test và bench phải có ROS; Pi5 trần không dùng được; khó test đơn vị |
| Giữ kiến trúc nhiều node nối topic như bản cũ | Quen thuộc | Jitter topic, 4 tần số khác nhau, không tất định; xem ADR-012 |
| Dùng `ros2_control` hardware interface làm HAL duy nhất | Một điểm cắm | Bench và test phải chạy `controller_manager`; xem ADR-012 |

## Hệ quả

Lõi test được bằng GTest thuần, build trong CI không cần ROS và trong container arm64 chỉ từ apt.
Đổi lại cần viết lớp chuyển đổi trong `ros2_node` và giữ cho nó mỏng; mọi logic điều khiển phải
nằm trong `pipeline` để Gazebo và bench chạy cùng một mã.
