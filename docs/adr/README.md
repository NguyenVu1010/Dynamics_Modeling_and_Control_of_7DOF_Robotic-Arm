# Architecture Decision Records

Mỗi ADR ghi một quyết định kỹ thuật: bối cảnh, lựa chọn, phương án đã loại và hệ quả. Thiết kế
gốc là `docs/design/2026-10-28-cpp-rework-design.md`; các ADR dưới đây tách các lựa chọn lớn của
thiết kế đó thành từng quyết định riêng.

## Quy tắc

- Một file một quyết định, đánh số tăng dần, tên `NNN-ten-ngan.md`, viết theo `000-template.md`.
- Mỗi ADR nêu phương án đã loại và lý do. ADR không có phương án đã loại là ADR chưa xong.
- Khi đổi ý, **không sửa** nội dung ADR cũ: viết ADR mới thay thế, và đổi trạng thái ADR cũ
  thành "thay thế bởi ADR-xxx". Được phép thêm mục "Bổ sung <ngày>" ghi số đo hoặc chi tiết
  hiện thực không đổi quyết định.
- Bắt buộc có ADR **trước khi**: thêm phụ thuộc ngoài, đổi build system hoặc bố cục target, đổi
  quy ước khung toạ độ/đơn vị (`docs/conventions.md`), thêm module mới trong `core/`, hoặc đi
  ngược một ADR đã chấp nhận (xem `CONTRIBUTING.md`).
- Trạng thái: `đề xuất` → `chấp nhận` → `thay thế bởi ADR-xxx`. `đề xuất, chờ bàn` nghĩa là còn
  câu hỏi mở cho chủ dự án; người xây dựng làm theo phương án mặc định ghi trong ADR cho tới khi
  chủ dự án chốt. `chờ phần cứng` nghĩa là chỉ ghi yêu cầu, chưa hiện thực.
- Các lựa chọn trong ADR là của người thiết kế trừ khi ghi "chủ dự án". ADR chỉ chuyển sang
  `chấp nhận` sau khi chủ dự án duyệt.

## Danh mục

| ADR | Quyết định | Trạng thái |
| --- | --- | --- |
| [001](001-language-and-standards.md) | Ngôn ngữ: C++17 cho lõi, Python 3.12 cho đánh giá và công cụ | đề xuất |
| [002](002-repo-layout-and-build.md) | Bố cục repo và build: CMake superbuild, target theo module, gói ament riêng | đề xuất |
| [003](003-ros-free-core.md) | Lõi không ROS, adapter mỏng | đề xuất |
| [004](004-self-written-dynamics.md) | Tự viết động lực học: RNEA 3D + CRBA, sáu bước kiểm chứng, fixture KDL chỉ trong test | đề xuất, chờ bàn |
| [005](005-urdf-model-and-mass-budget.md) | Mô hình từ URDF: parser tinyxml2 tự viết, gộp khớp cố định, sửa khối lượng | đề xuất, chờ bàn |
| [006](006-dof-template.md) | Số bậc tự do là tham số template `N`, alias `Arm7` | đề xuất |
| [007](007-two-tier-control.md) | Luật điều khiển hai tầng: CLIK bậc hai trên tham chiếu → computed torque khớp | đề xuất, chờ bàn |
| [008](008-threading-and-realtime.md) | Luồng và thời gian thực: lõi không tạo luồng, chế độ tất định | đề xuất |
| [009](009-yaml-config.md) | Cấu hình: YAML là nguồn tham số duy nhất của lõi | đề xuất |
| [010](010-testing.md) | Kiểm thử: các tầng test, kiểm chứng không cần Pinocchio, ngưỡng trong kịch bản | đề xuất |
| [011](011-simulation.md) | Mô phỏng: Gazebo Harmonic qua `gz_ros2_control`, effort, bước 1 ms | đề xuất, chờ bàn |
| [012](012-integration-points.md) | Hai điểm cắm: `ros2_control` cho Gazebo, `hal::JointInterface` cho bench; Pi5 để mở | đề xuất, chờ bàn |
| [013](013-logging-and-evaluation.md) | Log và đánh giá: CSV + Python, không rosbag | đề xuất |
| [014](014-pi5-interface.md) | Interface Pi5: EtherCAT CiA402 CST, 1 kHz, PREEMPT_RT (chỉ yêu cầu) | chờ phần cứng |
| [015](015-imperfections-and-sim-to-real.md) | Mô hình không hoàn hảo cấu hình được và phân tích độ nhạy sim-to-real | đề xuất |
| [016](016-frozen-tests-and-cicd.md) | Quy trình test trước, test bị khoá, CI/CD và bảo vệ `main` | đề xuất |
| [017](017-pi5-container.md) | Container arm64 sát Pi5: build, test dưới QEMU, image runtime trên GHCR | đề xuất |
