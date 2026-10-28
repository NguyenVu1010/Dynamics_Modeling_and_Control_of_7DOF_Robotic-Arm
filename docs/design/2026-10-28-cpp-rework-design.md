# Viết lại bộ điều khiển động lực học tay máy 7 bậc bằng C++ (thiết kế)

- Ngày: 2026-10-28
- Trạng thái: chờ duyệt
- Repo: `git@github.com:NguyenVu1010/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm.git`
- Mẫu tổ chức: `../graph_slam` (quy tắc phát triển, ADR, bố cục `core/ ros2_node/ bench/ sim/ eval/ tests/ tools/`)

## 1. Ý định và lý do

Bản hiện tại là một gói ROS 1 (catkin, Gazebo Classic) gồm 6 node Python nối nhau qua topic:
đọc tf → sinh quỹ đạo Cartesian (20 Hz) → IK vận tốc bằng giả nghịch đảo Jacobian có tối ưu
giới hạn khớp → mô-men theo computed torque `τ = M·q̈ + C·q̇ + G` cộng PD (100 Hz) → node giữ
vị trí dự phòng (300 Hz) → `JointGroupEffortController`. M, C, G là biểu thức đóng sinh từ Maple
(riêng M là 1 292 dòng). Tham số robot nằm trong một file JSON hard-code đường dẫn ngoài repo và
đã thất lạc.

Mục đích của bản viết lại:

1. **Một lõi C++ tự viết**, không ROS, không Pinocchio, tính động học và động lực học bằng
   thuật toán đệ quy thay cho biểu thức đóng, đủ nhanh để chạy 1 kHz trên một lõi Raspberry Pi 5.
2. **Một vòng điều khiển duy nhất trong một tiến trình** thay cho 6 node và 4 tần số khác nhau.
3. **Dự án tốt nghiệp dùng để phỏng vấn**: cấu trúc, ADR, test và số đo phải đủ để người đọc
   lạ hiểu được quyết định và tin được kết quả. Độ "khoe" nằm ở sự rõ ràng, không ở số dòng.

Người đọc đích: hội đồng và nhà tuyển dụng, rồi mới đến người tiếp tục phát triển .

## 2. Mục tiêu, không mục tiêu, phản mục tiêu

**Mục tiêu**

- G1. Lõi C++17 (`core/`) tính FK, Jacobian, IK, M, C, G, RNEA, động lực học thuận; mọi kích
  thước cố định lúc biên dịch; không cấp phát heap, không log, không exception trong vòng điều
  khiển.
- G2. Lõi đọc mô hình từ **URDF** (`cobot30_5.urdf`, xuất từ SolidWorks, có đủ khối lượng, tâm
  khối, tensor quán tính 9 link và giới hạn 7 khớp). Không còn DH, không còn JSON.
- G3. **MVP: bám một quỹ đạo duy nhất** trong Gazebo Harmonic + ROS 2 Jazzy (WSL đã có sẵn, đã
  kiểm tra `gz sim` 8.15.0 chạy headless được). Quỹ đạo khai báo cố định trong YAML: từ cấu hình
  nhà, đường thẳng 10 s tới đích của bản cũ `p = (−0,5; −0,6; 0,9) m`, RPY `(0; 0; 1,57)` trong
  `base_link`; cấu hình nhà do builder chọn, hợp lệ với giới hạn `joint_4` và xa kỳ dị, ghi trong
  YAML. Computed torque 1 kHz. Đạt khi: sai số vị trí đầu cuối đo tại t = 12 s (sau 2 s giữ)
  < 2 mm, hướng < 1°, RMS sai số bám khớp trên cả quãng < 0,01 rad, bench vòng điều khiển trên
  desktop p95 < 200 µs. Nhận đích mới qua topic lúc chạy là việc **sau MVP**.
- G4. Interface phần cứng (`JointInterface`) được định nghĩa trong lõi và có hai hiện thực:
  Gazebo (qua `ros2_control`) và mô phỏng nội bộ (`SimPlant`, dùng cho test và bench không cần
  Gazebo). Hiện thực EtherCAT cho Pi5 **chưa làm**, chỉ chừa chỗ.
- G5. Đánh giá bằng Python ngoài lõi (`eval/`): đọc CSV do vòng điều khiển ghi, tính các con số
  ở G3, vẽ đồ thị.
- G6. Tổ chức repo theo chuẩn graph_slam: `CONTRIBUTING.md`, `docs/adr/`, `docs/conventions.md`,
  clang-format/tidy, ruff, pre-commit, CI GitHub Actions, script build trong WSL.

**Không mục tiêu (lần này)**

- Driver EtherCAT, PREEMPT_RT, cross-compile cho Pi5. Chỉ giữ cho lõi không có phụ thuộc nào
  mà Pi5 không cài được bằng apt.
- Tránh va chạm, lập kế hoạch đường đi, quỹ đạo cong (`curvilinear` cũ), MoveIt.
- Điều khiển lực/impedance với môi trường, nhận dạng tham số động lực học.
- Giao diện người dùng ngoài RViz và đồ thị matplotlib.

**Phản mục tiêu** (thất bại dù code chạy)

- Lõi include một header ROS, Pinocchio, KDL hoặc Gazebo.
- Một ma trận kích thước động hoặc `std::vector` cấp phát trong `ControlLoop::step()`.
- Jacobian hoặc M, C, G không có test đối chiếu độc lập.
- Mô hình điều khiển và mô hình Gazebo lấy từ hai nguồn tham số khác nhau.
- Một quyết định kiến trúc không có ADR, hoặc một ADR không nêu phương án đã loại.

## 3. Ràng buộc

- Máy dev: Windows 11, không có toolchain C++; build và chạy sim trong WSL `Ubuntu-24.04`
  (12 lõi, 9 GB RAM, GCC 13.3, CMake 3.28, ROS 2 Jazzy desktop, Gazebo Sim 8.15.0 từ vendor
  package của Jazzy, `ros2_control`, `gz_ros2_control`, `effort_controllers`,
  `joint_trajectory_controller`, `kdl_parser`, `orocos-kdl` vendor, `xacro`, Eigen 3.4, GTest,
  yaml-cpp, tinyxml2, clang-format/tidy 18, ninja, ccache; kiểm tra 2026-10-28). Thiếu `libgmock-dev`
  và `ruff`, đưa vào `tools/wsl/setup.sh`. `sudo` cần mật khẩu nên mọi `apt install` do người
  dùng gõ.
- Thư mục dự án `C:\Users\NguyenVD\workspace\7dof_dynamic_control` hiện **chưa có `.git` riêng**;
  thư mục mẹ `workspace/` lại chứa một `.git` lạc không có commit nào. Giai đoạn 0 khởi tạo repo
  trong thư mục dự án và không đụng `.git` của `workspace/` (xem Q1).
- Lệnh build chạy từ PowerShell: `wsl -d Ubuntu-24.04 -- bash -lc "..."` (Git Bash làm hỏng đường
  dẫn `/mnt/c`). Làm việc nặng thì clone sang ext4 (`~/cobot`).
- Đích chạy thật sau này: Raspberry Pi 5 (Cortex-A76, aarch64, Debian/Ubuntu apt có Eigen, GTest,
  yaml-cpp, tinyxml2). Phần cứng khớp (theo tên mesh) là eRob110H/eRob90H của ZeroErr, EtherCAT
  CiA402 có chế độ mô-men.
- Robot: 7 khớp quay, 2 khớp cố định (`joint_6_2_fix`, flange). Giới hạn trong URDF; chú ý
  `joint_4` có khoảng `[-3.1416, -0.4]` nên **q = 0 không hợp lệ**; cấu hình nhà phải khai báo
  trong YAML và được kiểm tra nằm trong giới hạn. URDF có `damping = 0.003` mỗi khớp, ma sát 0.
- Mesh STL tổng 33 MB, đã nằm trong lịch sử git của repo cũ. Không dùng LFS.
- **Khối lượng trong URDF gốc sai** (quyết định sửa: chủ dự án; cách sửa: người thiết kế, xem 5.8).
  Thể tích mesh chia khối lượng URDF ra đúng 1000 kg/m³ ở mọi link, tức mật độ mặc định của
  SolidWorks. Tổng phần chuyển động chỉ 6,06 kg.
- `effort` của `joint_2` và `joint_4` trong URDF là 870 N·m, các khớp cùng cỡ là 87 N·m; gần như
  chắc là gõ thừa số 0. Builder đối chiếu với datasheet eRob và sửa trong cùng bước 5.8.
- Không có code, dữ liệu hay phần cứng của nơi làm việc trong repo.

## 4. Quyết định đã chốt với chủ dự án

| # | Quyết định | Nguồn |
| --- | --- | --- |
| D1 | Viết lại bằng C++; lõi không ROS | chủ dự án |
| D2 | Tự viết động lực học, không Pinocchio (cũng không KDL trong lõi) | chủ dự án |
| D3 | Phạm vi lần này: mô phỏng; Pi5 chỉ để interface chờ | chủ dự án |
| D4 | Môi trường sim: WSL đã dựng cho graph_slam (Jazzy + Harmonic) | chủ dự án |
| D5 | Tổ chức repo theo graph_slam | chủ dự án |
| D6 | Lõi template theo số bậc `N`; dự án dùng alias `Arm7 = Model<7>` | đề xuất thiết kế, chủ dự án đồng ý |
| D7 | Tiêu chí nghiệm thu G3 | đề xuất thiết kế, chủ dự án đồng ý |
| D8 | URDF là nguồn tham số hình học và quán tính duy nhất | đề xuất thiết kế, chưa bị phản đối |

Những lựa chọn còn lại dưới đây là **của người thiết kế**; mỗi lựa chọn lớn sẽ thành một ADR và chủ dự
án muốn bàn các ADR đó trước khi chúng chuyển sang "chấp nhận".

## 5. Kiến trúc

### 5.1 Bố cục repo

```
core/        C++17, không ROS. Namespace cobot::<module>, target cobot::<module>
  common/    kiểu cố định (VecN, MatN, Pose, Timestamp), Result, logger interface, hằng số
  model/     Model<N>: đọc URDF -> cây khớp phẳng; giới hạn; tham số quán tính; home
  kinematics/ FK, Jacobian hình học 6xN, Jdot*qdot, sai số pose SE(3)
  dynamics/  RNEA, CRBA, G, C*qdot, động lực học thuận (M^-1 qua LDLT), SimPlant (RK4)
  planning/  quỹ đạo Cartesian điểm-tới-điểm (quintic) + CLIK bậc hai sinh q_d, qd_d, qdd_d (null-space giới hạn khớp)
  control/   computed torque không gian khớp + chế độ giữ an toàn
  hal/       JointInterface (đọc q, qdot, tau_đo; ghi tau_lệnh), JointState, JointCommand; SimPlant adapter
  pipeline/  ControlLoop (một bước điều khiển), Config từ YAML (chỉ nơi đây parse YAML), CSV logger
ros2_node/   gói ament: plugin ros2_control ControllerInterface bọc ControlLoop; launch; params
bench/       binary không ROS: ControlLoop + SimPlant, đo p50/p95/CPU mỗi bước, ghi CSV
sim/         urdf/xacro (ros2_control tags), meshes, world SDF, cấu hình controller_manager
eval/        Python: đọc CSV, tính G3, vẽ; không import core
config/      default.yaml (gains, home, giới hạn mềm, tần số, đường dẫn URDF)
docs/        conventions.md, derivations/ (RNEA, CRBA, Jacobian, luật điều khiển), adr/, design/
tests/       unit/<module>, component/ (bài toán biết trước đáp án), data/ (fixture nhỏ)
tools/       wsl/setup.sh, wsl/build.sh, ci/
```

Chiều phụ thuộc cố định, linker ép qua target riêng từng module (như ADR-013 của graph_slam):
`pipeline → {control, planning} → {dynamics, kinematics} → model → common`; `planning` chỉ cần
`kinematics`; `control` cần `dynamics`; `hal` chỉ phụ thuộc `common`; `dynamics` phụ thuộc
`kinematics`. `ros2_node` và `bench` chỉ link `cobot::pipeline`.

Phụ thuộc ngoài của lõi: **Eigen 3.4** và **tinyxml2** (đọc URDF). `pipeline` thêm **yaml-cpp**.
Test thêm **GTest**. Không có gì khác. Mọi thứ đều có trong apt của Ubuntu 24.04 và Raspberry Pi OS.

### 5.2 Mô hình và quy ước toán

- `Model<N>`: mảng cố định `N` khớp quay, mỗi khớp có `T_parent_joint` (gốc khớp trong khung link
  cha, từ `<origin>`), trục quay đơn vị trong khung khớp, và link con với `m`, `c` (tâm khối trong
  khung link), `I_c` (tensor quán tính tại tâm khối, khung link), `damping`, giới hạn
  `q_min, q_max, qd_max, tau_max`. Khớp cố định được gộp vào link cha lúc nạp (cộng khối lượng,
  dời quán tính bằng Steiner, và hợp biến đổi của khớp cố định vào `<origin>` của khớp quay kế
  tiếp). Trong URDF này có hai khớp cố định: `fixed` (world → `base_link`, bỏ qua) và
  `joint_6_2_fix` nằm **giữa chuỗi** (`link_6 → link_6_2 → joint_7`), nên `link_6_2` gộp vào
  `link_6` và gốc của `joint_7` được nhân thêm biến đổi của `joint_6_2_fix`.
- Khung: `b` base_link (= world khi robot cố định), `i` khung link i, `e` end-effector (gốc
  `link_7` sau khi gộp). Tên biến đổi mang khung như graph_slam: `T_b_e`, `R_b_i`, `p_b_e`.
- Sai số hướng dùng vector trục-góc của `R_d · R_eᵀ` (log SO(3)), không dùng Euler trong vòng
  điều khiển. Quaternion chỉ ở biên I/O, thứ tự Hamilton (w, x, y, z), ghi trong chú thích.
- Jacobian hình học 6×N thứ tự `[v; ω]` biểu diễn trong `b`. Tên Jacobian `J_e_b`, đạo hàm theo
  thời gian tính qua `Jdot_qdot` (lan truyền gia tốc với q̈ = 0, không sai phân số).
- Đơn vị SI, radian. YAML được phép ghi độ với hậu tố `_deg`.
- Thời gian trong lõi là `Timestamp` (int64 ns) và `dt` là double giây; không so sánh double.

### 5.3 Động lực học (ADR riêng, quyết định D2 + chi tiết thiết kế)

- **RNEA** (Newton–Euler đệ quy, vector 3D kiểu Craig/Siciliano, không spatial algebra 6D, để
  sinh viên và người đọc theo được): `τ = rnea(q, q̇, q̈, g)`.
- **CRBA** cho `M(q)`; **G(q)** = `rnea(q, 0, 0, g)`; **C(q, q̇)·q̇** = `rnea(q, q̇, 0, 0)`. Không
  tính ma trận `C` đầy đủ trong vòng điều khiển; chỉ tính khi test tính chất `Ṁ − 2C` phản đối
  xứng (bằng ký hiệu Christoffel từ sai phân số của M, trong test).
- Độ phức tạp O(N) cho RNEA, O(N²) cho CRBA; với N = 7 đo trên desktop phải < 20 µs mỗi lần gọi.
  Mục tiêu Pi5 dưới 100 µs cho một bước `ControlLoop::step()` đầy đủ.
- Động lực học thuận trong `SimPlant`: `q̈ = M⁻¹(τ − C q̇ − G − D q̇)`, LDLT 7×7, tích phân RK4
  bước 1 ms, có giới hạn khớp cứng (dừng và triệt vận tốc). Dùng cho test vòng kín và bench; không
  thay Gazebo cho demo.
- **Kiểm chứng không cần Pinocchio** (bắt buộc trong `tests/`):
  1. Jacobian so với sai phân trung tâm của FK, sai số tương đối < 1e-6.
  2. `M` đối xứng (‖M − Mᵀ‖ < 1e-9), xác định dương (eigenvalue nhỏ nhất > 0) tại 1 000 cấu hình
     ngẫu nhiên seed cố định trong giới hạn khớp.
  3. `rnea(q, q̇, q̈)` = `M q̈ + C q̇ + G` tại cùng cấu hình (< 1e-9 tương đối).
  4. `Ṁ − 2C` phản đối xứng.
  5. Bảo toàn năng lượng: `SimPlant` không ma sát, không trọng lực, τ = 0, 10 s: biến thiên động
     năng tương đối < 1e-4 với RK4 1 ms.
  6. Đối chiếu với **fixture độc lập**: file `tests/data/reference_dynamics.csv` (100 bộ
     `q, q̇, q̈ → τ, M, G`) sinh **một lần** bằng KDL từ cùng URDF qua binary
     `tools/gen_reference_dynamics.cpp` (chỉ build khi `-DCOBOT_WITH_KDL_REF=ON`, tìm `orocos-kdl`
     trong `/opt/ros/jazzy`). Fixture được commit; test mặc định chỉ đọc file. Nếu tìm lại được
     `robot_parameters.json`, thêm fixture thứ hai từ biểu thức Maple cũ, chạy bằng Python một lần.
- Dẫn xuất viết trong `docs/derivations/rnea.md`, `crba.md`, `jacobian.md`: ký hiệu, quy ước
  khung, công thức lan truyền, chỗ gộp khớp cố định.

### 5.4 Động học ngược và luật điều khiển

Cấu trúc **hai tầng** (quyết định thiết kế sau vòng review, để tiêu chí "RMS sai số bám khớp"
của G3 có nghĩa và để giữ mạch kể "mô hình hoá động lực học → computed torque" của luận văn):

- **Tầng tham chiếu** (`planning`, không nhìn trạng thái đo): từ quỹ đạo Cartesian sinh quỹ đạo
  khớp tham chiếu `q_d, q̇_d, q̈_d` bằng CLIK bậc hai chạy trên chính tham chiếu (FK của `q_d`,
  không phải của `q` đo). Nó là một bộ sinh quỹ đạo khớp thuần tuý, tái lập được, không phụ
  thuộc plant, test được không cần động lực học.
- **Tầng điều khiển** (`control`): computed torque trong không gian khớp bám `q_d(t)`.

Một vòng 1 kHz, mỗi bước:

1. HAL đọc `q, q̇` (và `τ_đo` nếu có).
2. `planning.cartesian`: điểm quỹ đạo `x_d, ẋ_d, ẍ_d` (vị trí + trục-góc hướng) tại `t`. Luật
   thời gian quintic `s(t)` trên `[0, T]`, vị trí thẳng, hướng nội suy trục-góc
   `R(t) = R_0 · Exp(s · Log(R_0ᵀ R_T))`. Sau `T`, giữ `x_d = x_T`, `ẋ_d = ẍ_d = 0`.
3. `planning.clik`: với `q_d` hiện tại tính `T_b_e(q_d)`, `J(q_d)`, `J̇q̇_d`; sai số
   `e = [p_d − p(q_d); Log(R_d R(q_d)ᵀ)]`, `ė = ẋ_d − J q̇_d`;
   `q̈_d = J⁺_λ (ẍ_d + K_e,d ė + K_e,p e − J̇q̇_d) + (I − J⁺_λ J) q̈_null`, với `J⁺_λ` là giả nghịch
   đảo giảm chấn (`λ` theo số điều kiện, ngưỡng trong YAML), `q̈_null = −k_null ∇h(q_d) − k_damp q̇_d`,
   `h` là hàm phạt khoảng cách tới giới hạn khớp (kế thừa ý `joint_limit_optimizer.py` cũ, công
   thức trong `docs/derivations/control.md`). Tích phân `q̈_d → q̇_d → q_d` bằng bước `dt`
   (semi-implicit Euler), kẹp `q_d` trong giới hạn khớp. Khởi tạo `q_d = q` đo tại lúc nhận
   đích mới.
4. `control.computed_torque`: `τ = M(q)(q̈_d + K_d(q̇_d − q̇) + K_p(q_d − q)) + C(q, q̇) q̇ + G(q) + D q̇`,
   kẹp theo `tau_max` từ URDF. `K_p, K_d` đường chéo, trong YAML.
5. HAL ghi `τ`. `ControlLoop::step()` đẩy một bản ghi (t, q, q̇, q_d, q̇_d, τ, x(q), x_d, e, thời
   gian bước) vào bộ đệm vòng SPSC cố định. **Lõi không tạo luồng.** Host (`bench` hoặc plugin
   `ros2_control`) sở hữu luồng rút bộ đệm và ghi CSV (`pipeline::CsvSink`), nên khi chạy dưới
   `controller_manager` trên Pi5 PREEMPT_RT cách tổ chức luồng do host quyết, không do lõi.
6. An toàn: nếu `|q − q_min/max| < biên mềm` hoặc `|q̇| > qd_max` hoặc HAL quá hạn, chuyển sang
   chế độ **giữ** (PD về `q` lúc chuyển + G) và báo trạng thái. Không bao giờ ghi NaN ra HAL.

Số đo của G3 từ đó rõ nghĩa: sai số đầu cuối = `x(q) − x_T` từ FK của `q` đo; RMS bám khớp =
RMS của `q_d − q` trên cả quãng chạy; thêm một số đo phụ là sai số CLIK `‖e‖` (tham chiếu có
bám được đích không) để tách lỗi tầng tham chiếu khỏi lỗi tầng điều khiển.

Chế độ `deterministic: true`: một luồng, không timer thật, bước theo `dt` cố định; dùng trong
test và bench để tái lập bit-for-bit.

### 5.5 Nối với Gazebo: plugin `ros2_control`

`ros2_node/` là một **`controller_interface::ControllerInterface`** (`cobot_ros2/CobotController`):
`update()` được `controller_manager` gọi ở 1 kHz, đọc state interface (`position`, `velocity`,
`effort`) của 7 khớp từ `gz_ros2_control`, gọi `ControlLoop::step()`, ghi command interface
`effort`. MVP: quỹ đạo lấy từ YAML, bắt đầu khi controller được kích hoạt; trạng thái và sai số
publish 100 Hz. `robot_state_publisher` + RViz để xem. Sau MVP: topic `~/target_pose`
(`geometry_msgs/PoseStamped`) qua realtime buffer.

Vì sao chọn cách này thay vì node topic như bản cũ: vòng điều khiển chạy cùng tiến trình và cùng
nhịp với bước vật lý của Gazebo (bước 1 ms), không có jitter topic; trên Pi5 đúng plugin này chạy
được với hardware interface EtherCAT (`ethercat_driver_ros2`) **hoặc** `ControlLoop` chạy trong
một binary trần với `JointInterface` tự viết, tuỳ ADR sau. `hal::JointInterface` là điểm cắm cho
đường thứ hai; `ros2_control` là điểm cắm cho đường thứ nhất. Lõi không biết đường nào đang dùng.

Mô hình Gazebo: `sim/urdf/cobot30_5.urdf.xacro` = URDF gốc + khối `<ros2_control>` (effort
command, position/velocity/effort state) + plugin `gz_ros2_control`; mesh giữ nguyên cho visual,
collision tạm dùng cùng mesh (builder có thể thay bằng hình khối nếu Gazebo chậm). Bước vật lý 1 ms,
`update_rate: 1000`. Một world trống có mặt sàn; robot gắn cố định vào world.

### 5.6 Cấu hình, log, đánh giá

- `config/default.yaml` là nguồn tham số duy nhất của lõi: đường dẫn URDF, `home_q_deg`, `K_p`,
  `K_d`, `k_null`, `k_damp`, `lambda_max`, `cond_threshold`, biên mềm, `T_default`, tần số, cờ
  `deterministic`, đường dẫn CSV. Khoá lạ là lỗi; mọi giá trị được kiểm tra khoảng lúc nạp; thông
  điệp lỗi nêu tên khoá đầy đủ. `ros2_node` nạp cùng file qua tham số `config_path`.
- CSV một file mỗi lần chạy trong `results/<run-id>/`, không bao giờ trong cây nguồn.
- `eval/cobot_eval/`: `metrics.py` (sai số cuối, RMS bám khớp, p50/p95 thời gian bước),
  `plot.py` (q, τ, sai số theo thời gian), `report.py` in bảng so với ngưỡng G3 và thoát mã khác
  0 khi trượt. `pytest` cho `metrics.py` với CSV nhỏ trong `tests/python/data/`.

### 5.7 Luồng và thời gian thực

- Lõi **không tạo luồng**. Host sở hữu một luồng điều khiển (`controller_manager` hoặc `bench`)
  và một luồng ghi CSV; hai luồng trao đổi qua bộ đệm vòng SPSC cố định của `pipeline`.
- Pi5 sau này: PREEMPT_RT, `isolcpus`, `SCHED_FIFO`, khoá bộ nhớ; viết sẵn trong ADR nhưng không
  làm lần này.

### 5.8 Sửa khối lượng và quán tính trong URDF

Thành phần mỗi link (từ CSV của bộ xuất SolidWorks) và thể tích tính từ STL (tổng tứ diện có
dấu, mesh kín):

| Link | Thành phần | Thể tích (cm³) | Khối lượng cũ (kg) | Khối lượng mới (kg) | Hệ số |
| --- | --- | --- | --- | --- | --- |
| base_link | flange nhôm | 502,3 | 0,503 | 1,356 | 2,70 |
| link_1 | eRob110H | 1104,0 | 1,106 | 3,180 | 2,87 |
| link_2 | eRob110H | 1104,0 | 1,106 | 3,180 | 2,87 |
| link_3 | eRob90H + ống Link_23 | 1519,7 | 1,512 | 4,177 | 2,76 |
| link_4 | eRob90H | 706,1 | 0,708 | 1,980 | 2,80 |
| link_5 | eRob70H + ống Link_45 | 896,0 | 0,894 | 2,469 | 2,76 |
| link_6 | eRob70H | 322,3 | 0,323 | 0,920 | 2,85 |
| link_6_2 | eRob70H | 322,3 | 0,323 | 0,920 | 2,85 |
| link_7 | đầu cuối nhôm | 84,0 | 0,084 | 0,227 | 2,69 |
| **Tổng chuyển động** | | | **6,06** | **17,05** | |

Cách tính:

- Actuator lấy khối lượng datasheet của ZeroErr (bảng thông số eRob v2.0): eRob70H 0,92 kg,
  eRob90H 1,98 kg, eRob110H 3,18 kg. Mesh trong CAD mang hậu tố `T` (trục rỗng); bản T nặng hơn
  (eRob110H-T được bán lẻ ghi 4,52–4,64 kg). MVP dùng số bản H vì nguồn nhất quán; ghi rõ giới
  hạn này trong ADR-005.
- Ống nối và chi tiết kết cấu là nhôm 2,70 g/cm³. Thể tích ống = thể tích link trừ thể tích
  actuator cùng loại đo ở link chỉ có actuator (Link_23 = link_3 − link_4 = 813,6 cm³ → 2,197 kg;
  Link_45 = link_5 − link_6 = 573,7 cm³ → 1,549 kg).
- Mật độ hiệu dụng của actuator ra 2,80–2,88 g/cm³, rất gần nhôm, nên coi mỗi link là **đặc
  đồng chất**: giữ nguyên tâm khối, nhân tensor quán tính cùng hệ số với khối lượng. Sai số do
  bỏ qua chênh mật độ trong link ghép ước chừng vài phần trăm, nhỏ hơn nhiều sai số khối lượng
  cũ (gấp 2,8 lần).
- Khối lượng của mỗi actuator được gán trọn cho link chứa mesh của nó; không tách phần rotor và
  stator sang hai link kề nhau.

Thực hiện ở Giai đoạn 0 bằng script `tools/fix_urdf_inertia.py` đọc URDF gốc, bảng khối lượng
trên (lưu thành `sim/urdf/mass_budget.yaml` có nguồn của từng số), ghi URDF đã sửa. URDF gốc giữ
nguyên trong lịch sử git; URDF đã sửa là nguồn duy nhất (D8). Test pytest kiểm tra tổng khối
lượng và tỷ lệ quán tính. Giai đoạn 2 thêm một kiểm tra: mô-men trọng lực G(q) lớn nhất trên lưới
cấu hình không vượt mô-men định mức của actuator ở khớp đó; nếu vượt, ghi vào ADR-005 và báo chủ
dự án trước khi chỉnh gains.

## 6. Tiêu chuẩn mã nguồn và quy trình (kế thừa graph_slam, điều chỉnh)

- C++17, `-Wall -Wextra -Wpedantic`, CI thêm `-Werror`. clang-format 18, clang-tidy 18.
- Tên: kiểu `PascalCase`, hàm/biến `snake_case`, thành viên `_` cuối, hằng `kPascalCase`,
  namespace `cobot::<module>`. `#pragma once`. Không `auto` cho biểu thức Eigen. Eigen truyền
  `const&`, kiểu cố định trong hot path.
- Lỗi: khởi tạo/config được ném; `step()` trả `Status`; không `abort` vì dữ liệu xấu.
- Python 3.12, `ruff`, `pytest`, một `pyproject.toml` ở gốc.
- **Lịch sử và remote chỉ mang dấu của chủ dự án**: commit chỉ có tác giả là chủ dự án, không
  có dòng đồng tác giả tự động; mô tả PR, issue, comment không có chữ ký công cụ. File cấu hình
  và ghi chú riêng của công cụ hỗ trợ trên máy dev được bỏ qua bằng `.git/info/exclude` (không
  liệt kê trong `.gitignore`). Quy tắc phát triển nằm trong `CONTRIBUTING.md`. Thao tác remote
  (push, mở PR, comment) chỉ làm khi chủ dự án yêu cầu.
- Git: trunk-based, `main` bảo vệ, squash merge, Conventional Commits, PR ≤ 400 dòng, PR nêu test
  và số đo. Mọi finding của review phải sửa trước khi merge, không để "parked".
- Định nghĩa hoàn thành một tính năng: dẫn xuất trong `docs/`, ADR nếu có lựa chọn, test đơn vị +
  test thành phần biết trước đáp án, cờ YAML cho hành vi mới, số bench nếu đụng hot path, README/
  docs cập nhật.
- CI (GitHub Actions, `ubuntu-24.04`): job `lint` (clang-format, ruff); job `core` (apt Eigen,
  GTest, yaml-cpp, tinyxml2; cmake + ctest, `-Werror`); job `python` (pytest). **Không** có job
  ROS/Gazebo trong CI lần này (sinh viên, runner mỏng); demo Gazebo chạy tay trong WSL và kết quả
  CSV + đồ thị đính kèm PR. Có thể thêm job `ros2` bằng `ros-tooling/setup-ros` sau, không cần ADR
  mới.

## 7. Các ADR sẽ viết (tiếng Việt, theo mẫu graph_slam) và vị trí đề xuất

| ADR | Chủ đề | Vị trí đề xuất | Cần bàn? |
| --- | --- | --- | --- |
| 001 | Ngôn ngữ và chuẩn | C++17 cho lõi (GCC 13 desktop, GCC 12+ trên Pi OS), Python 3.12 cho eval/tools | thấp |
| 002 | Bố cục repo và build | như 5.1; CMake superbuild gốc + gói ament `ros2_node`; target theo module | thấp |
| 003 | Lõi không ROS, adapter mỏng | như D1 + 5.5 | đã chốt |
| 004 | Tự viết động lực học | D2; RNEA 3D + CRBA; kiểm chứng 6 bước ở 5.3; fixture KDL chỉ trong test | **có**: spatial 6D hay 3D; có cho KDL vào test không |
| 005 | Mô hình từ URDF, parser | URDF nguồn duy nhất; parser tinyxml2 tự viết cho tập con (link, joint, inertial, origin, axis, limit, dynamics); gộp khớp cố định | **có**: tinyxml2 tự viết hay `liburdfdom` |
| 006 | Số bậc tự do | template `N`, alias `Arm7` | đã chốt (D6) |
| 007 | Luật điều khiển | hai tầng: CLIK bậc hai trên tham chiếu (DLS, null-space giới hạn khớp) → computed torque khớp; chế độ giữ an toàn; phương án đã loại: resolved acceleration trực tiếp trên `q` đo, CLIK bậc một + sai phân như bản cũ | **có** |
| 008 | Luồng và thời gian thực | 1 luồng điều khiển + 1 luồng log; deterministic mode; kế hoạch Pi5 | thấp |
| 009 | Cấu hình YAML | như 5.6 | thấp |
| 010 | Kiểm thử và CI | như 5.3, 6; ngưỡng G3 là gate thủ công | thấp |
| 011 | Mô phỏng | Gazebo Harmonic qua `gz_ros2_control`, effort interface, bước 1 ms | **có**: collision mesh hay hình khối |
| 012 | Nối Gazebo/Pi5 qua `ros2_control` hay HAL riêng | cả hai điểm cắm, `ros2_control` cho sim, HAL cho bench; Pi5 để mở | **có** |
| 013 | Định dạng log và đánh giá | CSV + Python; không rosbag | thấp |
| 014 | Interface Pi5 (chờ) | chỉ ghi yêu cầu: EtherCAT CiA402 chế độ CST, 1 kHz, PREEMPT_RT | chờ phần cứng |

## 8. Các giai đoạn

**Giai đoạn 0, setup (kế hoạch đầu tiên, là thứ chủ dự án yêu cầu "trước tiên")**

Kết quả: repo build xanh trong WSL và CI, chưa có thuật toán.

1. Lịch sử git (chủ dự án chốt: **giữ lịch sử**): `git init` trong thư mục dự án, fetch repo
   GitHub, tag `v0-python` trên commit cuối của bản cũ, làm việc trên nhánh, xoá cây Python khỏi
   cây hiện tại (vẫn còn trong lịch sử và tag), chuyển `urdf/`, `meshes/` vào `sim/`. Thiết kế
   này được commit vào nhánh đó. Không push đè `main`; merge qua PR.
1b. Sửa khối lượng URDF theo 5.8.
2. Khung repo theo 5.1: CMake gốc, `core/CMakeLists.txt`, mỗi module một target (INTERFACE khi
   chưa có source), `cmake/cobot_warnings.cmake`, `.clang-format`, `.clang-tidy`, `.editorconfig`,
   `.pre-commit-config.yaml`, `pyproject.toml`, `.gitignore`, `.gitattributes` (STL binary).
3. `CONTRIBUTING.md` (tiếng Anh, nội dung quy tắc như graph_slam), `docs/adr/000-template.md`, `docs/adr/README.md`, ADR
   001–003, 006, 009, 010 ở trạng thái "đề xuất"; các ADR "cần bàn" (004, 005, 007, 011, 012)
   viết với phương án và để trạng thái "đề xuất, chờ bàn"; `docs/conventions.md`.
4. `core/common`: `types.hpp` (`VecN<N>`, `MatN<N>`, `Vec3`, `Mat3`, `Vec6`, `Pose`, `Timestamp`),
   `result.hpp`, `logger.hpp`; một unit test chạy được.
5. **Header interface** (hợp đồng) của `model`, `kinematics`, `dynamics`, `planning`, `control`,
   `hal`, `pipeline`: khai báo lớp và chữ ký hàm với chú thích khung/đơn vị, chưa có thân.
6. `config/default.yaml` với mọi khoá ở 5.6 và giá trị khởi điểm (gains lấy từ bản cũ, sẽ tinh
   chỉnh); `home_q_deg` hợp lệ với giới hạn `joint_4`.
7. `sim/urdf/cobot30_5.urdf.xacro` + world + cấu hình `controller_manager`; kiểm tra
   `xacro` sinh được URDF và `gz sdf -p` đọc được (script `tools/ci/check_sim_assets.sh`, không
   cần Gazebo chạy).
8. `tools/wsl/setup.sh` (apt list cho người dùng gõ), `tools/wsl/build.sh`, CI ba job, README
   mới (mục đích, kiến trúc, cách build, trạng thái).
9. `eval/` và `tests/python/` tối thiểu với một test pytest.

**Giai đoạn 1**: `model` (parser URDF, gộp khớp cố định) + `kinematics` (FK, J, J̇q̇) + test
Jacobian sai phân.
**Giai đoạn 2**: `dynamics` (RNEA, CRBA, SimPlant) + 6 bước kiểm chứng + fixture KDL + bench
thời gian mỗi hàm.
**Giai đoạn 3**: `planning`, `control`, `pipeline::ControlLoop`, bench vòng kín với SimPlant; test
thành phần: đi tới pose trong SimPlant đạt ngưỡng G3 (trừ p95 đo trên desktop).
**Giai đoạn 4**: `ros2_node` + Gazebo demo + `eval` báo cáo; chốt G3; chuyển ADR sang "chấp nhận".
**Giai đoạn 5** (ngoài phạm vi lần này): Pi5, EtherCAT, PREEMPT_RT.

Mỗi giai đoạn là một kế hoạch riêng trong `.local/plans/` (cục bộ, không commit, xem 6), mỗi PR ≤ 400 dòng.

## 9. Để lại cho người xây dựng

- Cách viết parser URDF (tập con, báo lỗi), miễn là không thêm phụ thuộc ngoài tinyxml2.
- Chi tiết RNEA/CRBA (thứ tự lan truyền, cách gộp khớp cố định), miễn là qua 6 bước kiểm chứng.
- Chọn hàm phạt giới hạn khớp `h(q)` và cách tính `λ` của DLS; ghi vào `docs/derivations/control.md`.
- Có thay collision mesh bằng hình khối hay không.
- Cấu trúc bộ đệm vòng cho logger.
- Giá trị gains khởi điểm; chỉ cần ghi lại cách tinh chỉnh và số đo cuối.

## 10. Quyết định sau vòng review

| Câu hỏi | Trả lời | Nguồn |
| --- | --- | --- |
| Lịch sử git | Giữ lịch sử, tag `v0-python` | chủ dự án |
| Khối lượng URDF | Tính lại từ vật liệu và thể tích (5.8) | chủ dự án yêu cầu, người thiết kế tính |
| Kịch bản MVP | Bám một quỹ đạo cố định (G3) | chủ dự án; đích, thời điểm đo do người thiết kế chọn |
| Cấu trúc điều khiển | Hai tầng CLIK tham chiếu → computed torque khớp (5.4) | người thiết kế |
| Luồng | Lõi không tạo luồng; host sở hữu luồng ghi (5.7) | người thiết kế |
| Giấy phép | MIT, tác giả là chủ dự án | mặc định |
| Ngôn ngữ | ADR/derivations tiếng Việt; CONTRIBUTING.md, code tiếng Anh; README tiếng Anh có tóm tắt tiếng Việt | mặc định |
| `damping`/`mu_viscous` trong URDF | Dùng `damping` chuẩn (0,003); bỏ thuộc tính không chuẩn; ghi trong ADR-005 | người thiết kế |

## 11. Mở rộng 2026-10-29: kiểm thử đầu-cuối, mô hình không hoàn hảo, sim-to-real, CI/CD

Yêu cầu của chủ dự án: định nghĩa bộ test và quy trình phát triển đầu-cuối; thêm chức năng sửa
đổi nhiễu đầu vào; báo cáo sai số; đánh giá ảnh hưởng của độ chính xác cảm biến và động cơ tới
kết quả đầu ra để phục vụ chuyển từ mô phỏng sang thật; phát triển song song nhiều nhánh,
người viết test tách khỏi người hiện thực, không sửa test sau khi đã hiện thực; chỉ push khi
test pass; CI/CD kiểm duyệt chất lượng; container Ubuntu sát môi trường Pi5; chỉ merge PR khi
pass; mỗi PR một tính năng, không dài; tự động làm đến hết MVP. Mọi chi tiết dưới đây là lựa chọn
của người thiết kế.

### 11.1 Module `imperfect`: mô hình không hoàn hảo có thể cấu hình

Module mới `core/imperfect` (namespace `cobot::imperfect`, phụ thuộc `hal`, `dynamics`, `common`).
Mọi nguồn ngẫu nhiên dùng `std::mt19937_64` với seed từ YAML; không cấp phát trong `step`.

| Nhóm | Tham số (mỗi khớp, trừ khi ghi khác) | Áp vào |
| --- | --- | --- |
| Cảm biến vị trí | `encoder_bits` (lượng tử hoá trên 2π), `q_noise_std_rad`, `q_offset_std_rad` (lệch hiệu chuẩn, rút một lần mỗi lần chạy) | đọc `q` |
| Cảm biến vận tốc | `qd_source`: `true` hoặc `diff` (sai phân `q` đã lượng tử + lọc thông thấp `qd_filter_hz`), `qd_noise_std_rad_s` | đọc `q̇` |
| Trễ đo | `sensor_delay_cycles` | đọc |
| Động cơ | `torque_scale_std` (sai số hằng số mô-men, rút một lần), `torque_ripple_frac` + `ripple_periods_per_rev`, `torque_cmd_resolution_nm`, `motor_time_constant_s` (bậc một), `command_delay_cycles` | ghi `τ` |
| Ma sát chưa mô hình | `coulomb_nm`, `viscous_nm_s_rad` | plant |
| Sai lệch mô hình | `link_mass_scale_std`, `com_shift_std_m`, `payload_kg` ở đầu cuối | plant (bộ điều khiển dùng mô hình danh định) |
| Thời gian | `period_jitter_std_us` | bước của plant trong bench |

- `ImperfectJointInterface<N>`: decorator bọc bất kỳ `hal::JointInterface<N>` (cảm biến khi `read`,
  động cơ khi `write`). Dùng chung cho bench (`SimJointInterface`) và Gazebo (plugin `ros2_control`
  bọc state/command bằng cùng lớp), nên một profile cho cùng nhiễu ở hai nơi.
- `perturb_model(model, mismatch, rng)`: plant dùng mô hình bị nhiễu, bộ điều khiển dùng mô hình
  danh định. Trong Gazebo, `tools/fix_urdf_inertia.py` có tuỳ chọn sinh URDF nhiễu từ cùng profile.
- Ma sát và độ trễ động cơ nằm trong `SimPlant` (bench) và trong decorator (Gazebo, xấp xỉ theo
  `q̇` đo).
- Profile: `config/imperfections/{ideal,realistic,pessimistic}.yaml`. `ideal` = tất cả bằng 0.
  Mỗi số có trường `source`; số chưa có nguồn ghi `assumed`.

### 11.2 Báo cáo sai số và phân tích độ nhạy

- `bench` có ba chế độ: `run` (một lần chạy, một CSV), `montecarlo` (N seed trên một profile),
  `sweep` (một yếu tố mỗi lần: mỗi tham số ở 5–7 mức, các tham số khác giữ ở profile nền, mô tả
  trong `config/sweeps/<tên>.yaml`). Mỗi lần chạy ghi một dòng vào `summary.csv` (tham số, seed,
  sai số vị trí cuối, sai số hướng cuối, RMS bám khớp, mô-men đỉnh, thời gian bão hoà, có vào chế
  độ giữ không).
- `eval/cobot_eval/report.py`: báo cáo Markdown + PNG cho một lần chạy (bảng so ngưỡng, đồ thị `q`,
  `q_d − q`, `τ`, sai số đầu cuối theo thời gian), thoát mã khác 0 khi trượt ngưỡng.
- `eval/cobot_eval/sensitivity.py`: từ `summary.csv` vẽ đường "mức tham số → chỉ số", biểu đồ
  tornado, và tính **bảng yêu cầu phần cứng**: với mỗi tham số, mức xấu nhất mà chỉ số vẫn đạt
  ngưỡng G3 (ví dụ "encoder ≥ 16 bit", "trễ lệnh ≤ 2 chu kỳ"). Bảng này là sản phẩm sim-to-real của
  MVP, nằm trong `docs/results/`.
- Monte Carlo: tỷ lệ đạt, phân vị 50/95 của từng chỉ số.

### 11.3 Bộ test đầu-cuối

| Mã | Nội dung | Ngưỡng | Chạy khi |
| --- | --- | --- | --- |
| E2E-01 | bench + SimPlant, profile `ideal`, quỹ đạo MVP | G3: 2 mm, 1°, 0,01 rad | mỗi PR |
| E2E-02 | như E2E-01, profile `realistic`, seed cố định | 5 mm, 2°, 0,02 rad | mỗi PR |
| E2E-03 | tất định: hai lần chạy cùng seed cho CSV giống từng byte (chế độ `deterministic` ghi `step_us = 0`, thời gian đo riêng bằng PF-01) | bằng nhau | mỗi PR |
| E2E-04 | an toàn: đích kéo khớp vào giới hạn, NaN từ HAL, HAL quá hạn | vào chế độ giữ, không NaN ra, mô-men trong giới hạn | mỗi PR |
| E2E-05 | Monte Carlo 50 seed, `realistic` | ≥ 95 % đạt ngưỡng E2E-02 | hằng đêm, chạy tay |
| E2E-06 | sweep độ nhạy, báo cáo + bảng yêu cầu phần cứng | báo cáo sinh được | hằng đêm, chạy tay |
| E2E-07 | Gazebo headless, controller thật qua `gz_ros2_control`, quỹ đạo MVP, profile `ideal` | G3 | mỗi PR |
| E2E-08 | container arm64 (Pi5): build, unit, E2E-01..04 dưới QEMU | pass | mỗi PR |
| PF-01 | thời gian một bước `ControlLoop` | báo cáo p50/p95; gate p95 < 1 ms trên runner CI; 200 µs đo tay trên desktop cho G3 | mỗi PR |

Kịch bản E2E là file YAML trong `tests/e2e/scenarios/`; runner `tools/run_e2e.py` gọi bench (hoặc
launch Gazebo), rồi `report.py`, và trả mã lỗi. Ngưỡng nằm trong kịch bản, không trong code.

### 11.4 Quy trình phát triển: test trước, test bị khoá

- Mỗi PR một tính năng, nhánh `feat/<tính-năng>`; ≤ 400 dòng code không phải test và ≤ 400 dòng
  test.
- **Người viết test** viết test từ thiết kế và header interface, commit `test(<phạm vi>): ...`.
  Test phải biên dịch được và thất bại vì chưa có hiện thực.
- **Người hiện thực** chỉ sửa ngoài `tests/`. Build, ctest, pytest, lint cục bộ pass mới được push.
- **Khoá test bằng CI** (`tools/ci/check_frozen_tests.sh`, job `frozen-tests`): (1) file dưới `tests/`
  đã có trên `main` không được sửa hay xoá trong PR, trừ khi PR có nhãn `test-change` kèm lý do;
  (2) trong PR, sau commit `test(` cuối cùng không commit nào được đụng `tests/`.
- Người hiện thực thấy test sai so với thiết kế thì dừng và báo; người điều phối quyết định một PR
  `test-change` riêng. Không bao giờ nới test để cho pass.
- Merge: squash, chỉ khi mọi check bắt buộc xanh. Nhánh tự xoá sau merge.

### 11.5 CI/CD

`main` được bảo vệ: bắt buộc PR, bắt buộc các check dưới đây xanh, nhánh phải cập nhật với `main`,
áp dụng cả cho admin, cấm push thẳng và force-push.

| Job | Nội dung |
| --- | --- |
| `lint` | clang-format 18, ruff check + format, clang-tidy 18 trên file C++ thay đổi |
| `frozen-tests` | 11.4 |
| `core` | build Release `-Werror`, ctest |
| `sanitizers` | build Debug ASan + UBSan, ctest |
| `coverage` | gcovr trên `core/`, gate ≥ 80 % dòng, báo cáo artifact |
| `python` | pytest |
| `e2e` | E2E-01..04, PF-01, báo cáo artifact |
| `gazebo` | image ROS 2 Jazzy + Gazebo (amd64): colcon build, E2E-07 |
| `arm64` | `docker/pi5.Dockerfile` qua buildx + QEMU: E2E-08 |
| `nightly` (lịch, không bắt buộc) | E2E-05, E2E-06, báo cáo artifact |
| `deploy` (push vào `main`) | build image arm64 runtime, đẩy `ghcr.io/nguyenvu1010/cobot-pi5:{main,<sha>}` |

Job nào chưa có gì để chạy (ví dụ `e2e` trước khi có bench) vẫn tồn tại và pass rỗng, để danh sách
check bắt buộc cố định từ đầu.

**Container sát Pi5**: `docker/pi5.Dockerfile`, `FROM ubuntu:24.04` nền `linux/arm64` (Ubuntu 24.04
có bản chính thức cho Pi5), chỉ cài từ apt các gói lõi dùng, biên dịch `-mcpu=cortex-a76`, đa tầng:
`build` (core, test, bench) → `runtime` (bench, config, URDF, chạy không root). Image ROS cho Pi5
không thuộc MVP. Thời gian đo dưới QEMU không có ý nghĩa, chỉ kiểm tra đúng/sai.

### 11.6 Định nghĩa hoàn thành MVP

1. E2E-01..04, E2E-07, E2E-08, PF-01 xanh trên `main`.
2. E2E-05 đạt, E2E-06 sinh báo cáo; bảng yêu cầu phần cứng và báo cáo sai số commit vào
   `docs/results/mvp/`.
3. Image Pi5 được đẩy lên GHCR từ `main`.
4. ADR cập nhật theo những gì đã làm; các ADR "chờ bàn" vẫn chờ chủ dự án.
5. README có số liệu cuối và cách tái lập.

### 11.7 Thay đổi so với các mục trước

- 5.1: thêm `core/imperfect`, `tests/e2e/`, `docker/`, `config/imperfections/`, `config/sweeps/`,
  `tools/run_e2e.py`. `so3_exp`/`so3_log` chuyển từ `kinematics` vào `common` để `planning` không
  phụ thuộc thân hàm của `kinematics`. `tests/unit/` tự thêm mọi thư mục con để các nhánh song song
  không xung đột ở `CMakeLists.txt`.
- 6: CI thêm các job ở 11.5 (thay câu "không có job ROS/Gazebo trong CI").
- 7: thêm ADR-015 (mô hình không hoàn hảo và sim-to-real), ADR-016 (quy trình test khoá và CI/CD),
  ADR-017 (container Pi5).
