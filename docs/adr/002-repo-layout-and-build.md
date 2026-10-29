# ADR-002: Bố cục repo và build: CMake superbuild, target theo module, gói ament riêng

- Trạng thái: chấp nhận (MVP, 2026-10-29)
- Ngày: 2026-10-29
- Liên quan: thiết kế mục 5.1, 11.7; D5; ADR-013 của graph_slam (mẫu)

## Bối cảnh

Chủ dự án muốn tổ chức repo theo graph_slam (D5). Lõi phải build không cần ROS (CI mỏng, bench
trên Pi5), còn `ros2_node` và `sim` là gói ament build bằng colcon. Chiều phụ thuộc giữa các
module phải được ép bằng công cụ, không bằng kỷ luật.

## Quyết định

- Bố cục: `core/ ros2_node/ bench/ sim/ eval/ config/ docs/ docker/ tests/ tools/` như
  `CONTRIBUTING.md`.
- `CMakeLists.txt` gốc là superbuild không ROS: `core/` + `tests/` (+ `bench/` khi có). CMake
  ≥ 3.22, Ninja, ccache khi có. Option `COBOT_BUILD_TESTS` (ON), `COBOT_WERROR` (OFF),
  `COBOT_WITH_KDL_REF` (OFF). Mọi option/biến có tiền tố `COBOT_`.
- **Mỗi module một target** `cobot::<module>` (tên thật `cobot_<module>`): `common model
  kinematics dynamics planning control hal imperfect pipeline`. Module chưa có source là
  `INTERFACE`; thêm source thì đổi sang `STATIC`. Cờ chung trong `cmake/cobot_warnings.cmake`
  (`cobot_set_target_defaults`).
- Chiều phụ thuộc: `model → common`, `kinematics → model`, `dynamics → kinematics`,
  `hal → common`, `planning → kinematics`, `control → dynamics, hal`, `imperfect → hal, dynamics`,
  `pipeline → control, planning, hal, imperfect`. `ros2_node` và `bench` chỉ link `cobot::pipeline`.
- `so3_exp`, `so3_log`, `pose_error` nằm trong `common` (`cobot/common/lie.hpp`), không trong
  `kinematics`, để quỹ đạo Cartesian của `planning` không cần thân hàm của `kinematics` và các
  nhánh song song không chặn nhau (thiết kế 11.7).
- `SimJointInterface` (adapter `SimPlant` → `hal::JointInterface`) nằm trong `pipeline` để `hal`
  chỉ phụ thuộc `common`.
- `tests/unit/` và `tests/component/` tự thêm mọi thư mục con có `CMakeLists.txt`; một test
  dùng `cobot_add_test(name sources...)`.
- `sim/` là gói ament `cobot_sim`, `ros2_node/` là gói ament `cobot_ros2`; cả hai build bằng
  colcon (`tools/wsl/sim.sh build`), build dir ngoài cây nguồn (`$HOME/cobot-build/`).
- Phụ thuộc ngoài của lõi: Eigen 3.4, tinyxml2 (`model`), yaml-cpp (`pipeline`); test thêm GTest.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Một target `cobot::core` | Ít CMake | Không ép được chiều phụ thuộc; `planning` include nhầm `dynamics` không bị bắt |
| Chỉ colcon cho mọi thứ | Một cách build | CI và Pi5 phải cài ROS chỉ để build lõi; trái ADR-003 |
| `so3_*` trong `kinematics` | Gần nơi dùng FK | `planning-traj` phải chờ `kinematics` hiện thực xong; hai nhánh song song xung đột |
| Danh sách test viết tay trong `tests/unit/CMakeLists.txt` | Tường minh | Mọi nhánh song song cùng sửa một file, xung đột merge liên tục |

## Hệ quả

Mọi vi phạm phân lớp là lỗi link. Thêm module là vài dòng CMake nhưng cần ADR. Có hai cách build
(CMake gốc và colcon) phải giữ đồng bộ; `core/CMakeLists.txt` phải tự đứng được để `cobot_ros2`
tìm được `cobot::pipeline`.
