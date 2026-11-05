# ADR-022: Cầu nối bộ nhớ chia sẻ giữa controller và studio, quyền điều khiển và lệnh tay

- Trạng thái: đề xuất; hai mục "Chờ bàn" đã chốt theo mặc định ở PR manual-commands (2026-11-05)
- Ngày: 2026-11-03
- Liên quan: thiết kế `docs/design/2026-11-02-cobot-studio-design.md` mục 2 (G6–G9), 4 (quyết
  định 12, 13, 14, 17), 5.4, 6.4, 6.5, 6.8, 7, 8 (PR 7–11); ADR-008, ADR-012, ADR-018, ADR-020, ADR-021

## Bối cảnh

Giai đoạn B của `cobot_studio`: điều khiển robot từ giao diện như robot thật, trên Gazebo với cùng
plugin controller sẽ chạy trên Pi 5 (quyết định 12). Ba chế độ: từng khớp, TCP, theo điểm (quyết
định 13); khớp và TCP có cả giữ nút lẫn nhập giá trị đích (quyết định 14). Studio là tiến trình riêng,
giao tiếp với luồng điều khiển qua vòng bộ nhớ chia sẻ (quyết định 17). Gazebo và studio chạy cùng
máy (WSL, sau này Pi 5 có màn hình).

ADR này viết cùng ADR-020 và ADR-021 để ba ADR mới của thiết kế vào cùng lúc; hiện thực ở giai đoạn
B. Ràng buộc: luồng điều khiển không cấp phát, không khoá, không chờ, không I/O (ADR-008). Phản mục
tiêu: studio treo hay chết mà robot vẫn chạy theo lệnh tay; hai nguồn lệnh (studio và panel RViz
của ADR-018) cùng lúc điều khiển robot.

## Quyết định

**Vùng nhớ cầu nối** (`src/core/bridge`, mới, không ROS, không Qt; dùng chung cho controller và
studio): `shm_open("/cobot_<tên controller>")` + `mmap`, controller tạo khi activate, xoá khi cleanup.
Vùng này **khác** vùng log hộp đen `/cobot_log_<tên controller>` của ADR-020. Đầu vùng: magic, phiên
bản bố cục, `N`, kích thước bản ghi, dung lượng các vòng, PID studio đang giữ quyền, nhịp sống.
Studio kiểm bốn giá trị đầu trước khi dùng; khác thì báo lỗi phiên bản, không đọc. Bố cục byte cụ
thể do người xây dựng chọn.

**Bản ghi trên dây là POD**: mọi kiểu đặt trên vùng nhớ là trivially copyable, mọi
`std::atomic<uint64_t>` có `is_always_lock_free` (`static_assert`). Bản ghi trạng thái **không phải
`LogRecord<N>`** (kiểu đó không trivially copyable, ADR-020) mà là một struct phẳng chép từ nó.

**Các vòng:**

- **Vòng trạng thái** (controller → studio): 4096 ô (khoảng 4 s ở 1 kHz), luồng điều khiển **ghi đè ô
  cũ, không bao giờ chờ**, mỗi ô có số thứ tự kiểu seqlock. Studio phát hiện ô bị ghi đè trong lúc đọc
  thì bỏ ô đó và đếm "mất mẫu hiển thị". Log đầy đủ vẫn nằm trong `Recorder` (ADR-020); studio chậm
  chỉ làm hình thưa đi.
- **Vòng lệnh** (studio → controller): SPSC 64 ô lệnh kích thước cố định. **Luồng điều khiển không
  đọc vòng này**: một luồng thường của controller (luồng cầu nối) lấy lệnh, kiểm quyền, chế độ, giới
  hạn, làm phần nặng (`validate_path`, nạp cấu hình) ngoài thời gian thực, rồi chuyển vào `update()`
  qua cơ chế sẵn có của ADR-018 (`RealtimeBuffer`, cờ atomic).
- **Ô jog**: một ô seqlock (chiều, trục, thời điểm làm mới) mà `update()` đọc không chờ.
- **Vòng phản hồi** (controller → studio, luồng cầu nối ghi): chấp nhận/từ chối kèm lý do và kết quả
  xem trước.
- Dữ liệu của một lần chạy Gazebo cho tab Tracking lấy từ `Recorder` (đầy đủ), không từ vòng trạng
  thái: hết lần chạy, studio gửi lệnh "xuất đoạn [t0, t1]", luồng cầu nối ghi CSV vào thư mục tạm và
  trả đường dẫn. Lần chạy dài hơn `log.capacity_s` bị đánh dấu "cắt đầu".
- `log.mode: recorder_live` (ADR-020) là chế độ có vòng trạng thái; `recorder` chỉ ghi log.

**Quyền điều khiển**: một nguồn lệnh tại một thời điểm. Studio giành quyền qua cầu nối; khi studio
giữ quyền, action/service ADR-018 của panel RViz bị từ chối với lý do "studio đang điều khiển" (xem
vẫn được). Client thứ hai giành quyền bị từ chối. Quyền tự trả khi studio đóng hoặc mất nhịp sống:
studio ghi nhịp mỗi 50 ms; mất nhịp quá **200 ms** thì lệnh tay hay quỹ đạo đang chạy dừng có giảm
tốc (như `request_stop` của ADR-018), về standby, rồi trả quyền.

**Lệnh tay** trong `ControlLoop` (chế độ `command_source: external`, chỉ nhận ở `kStandby`, trừ làm
mới jog và dừng; thiết kế 6.4):

| Lệnh | Tham chiếu | Kiểm tra trước khi nhận |
| --- | --- | --- |
| `jog_joint(j, dir)` | vận tốc khớp j hình thang tới `v_max·speed`, bỏ qua CLIK, các khớp khác giữ | khớp hợp lệ, chưa ở giới hạn mềm theo chiều đó |
| `move_joint(q_goal)` | các khớp tới đích cùng lúc trong `v_max·speed`, `a_max·speed`, bỏ qua CLIK | trong giới hạn mềm; mô-men dọc đường kiểm như `validate_path` |
| `jog_tcp(axis, dir, frame)` | twist trên một trục, hệ gốc hoặc hệ dụng cụ, hình thang tới giới hạn TCP·speed, qua CLIK | không gần kỳ dị, không ở giới hạn mềm |
| `move_tcp(T_goal)` | `WaypointTrajectory` một điểm `stop`, thời lượng theo giới hạn TCP·speed | `validate_path` của ADR-018 (cùng hàm với Xem trước) |
| `load(waypoints)` | như ADR-018 | như ADR-018 |
| `stop()`, `estop()`, `reset()` | xem dưới | — |

- **Nút người chết**: lệnh jog làm mới mỗi ≤ 50 ms; không làm mới trong **100 ms** thì giảm tốc về 0
  rồi standby; thả nút gửi lệnh dừng ngay.
- **Tốc độ chung**: thanh trượt mặc định **10 %**, nhân giới hạn vận tốc và gia tốc khớp và giới hạn
  TCP; giới hạn TCP ở 100 %: **0,25 m/s** và **1,0 rad/s**. Quỹ đạo theo điểm dùng thời lượng đã kiểm
  của ADR-018, không theo thanh tốc độ.
- **Giới hạn mềm**: jog khớp và jog TCP dừng cách giới hạn URDF **2°**. Jog dừng ở giới hạn mềm hay
  gần kỳ dị thì giảm tốc về 0 và báo lý do, **không** vào `kHolding`.
- Đổi chế độ và áp dụng cấu hình **chỉ khi standby** (đứng yên).
- **Dừng**: giảm tốc dọc đường đang đi (jog về vận tốc 0; quỹ đạo co giãn thời gian như ADR-018), rồi
  standby. **Dừng khẩn**: vào `kHolding` ngay, cần `reset` như hiện nay.
- Cột `cmd` của Run CSV (ADR-020) ghi `jog_joint`, `jog_tcp`, `move_joint`, `move_tcp`. Mọi lệnh
  không cấp phát khi thành công; các kiểm tra an toàn hiện có (ADR-008) vẫn áp dụng.

**Áp dụng cấu hình từ studio trên Gazebo**: studio gửi đường dẫn file YAML đã lưu (ADR-021); chỉ nhận
khi standby. Luồng cầu nối dựng **toàn bộ** phần `on_configure` đang dựng (mô hình bị nhiễu, decorator,
`ControlLoop`) ngoài thời gian thực; `update()` đổi con trỏ ở bước kế tiếp; bộ mới bắt đầu ở standby
tại tư thế đo hiện tại. `Recorder` và các vùng nhớ chia sẻ thuộc controller, sống qua lần đổi.
`control_rate_hz` không đổi được.

**Đã chốt ở PR manual-commands** (trước đây là "Chờ bàn"; người xây dựng dùng mặc định tới khi chốt
ở PR `manual-commands`):

- Hồ sơ vận tốc của `move_joint`: mặc định **hình thang đồng bộ** (phương án khác: bậc 5).
- Ngưỡng gần kỳ dị: thiết kế nói "chỉ số manipulability dưới ngưỡng"; mặc định dùng **số điều kiện
  của Jacobian CLIK vượt `clik.cond_threshold`** (hiện 100, cùng ngưỡng CLIK bắt đầu giảm chấn), để
  không thêm khoá mới.

Chốt cả hai mặc định trên:

- `move_joint` dùng **hình thang đồng bộ**: một hồ sơ chuẩn hoá s(t) ∈ [0, 1] chung cho mọi khớp,
  q(t) = q_start + s(t) (q_goal − q_start), đường thẳng trong không gian khớp, mọi khớp tới đích cùng
  lúc. Với d_j = |q_goal_j − q_start_j| (bỏ khớp có d_j = 0): s_v_max = min_j speed·qd_max_j / d_j,
  s_a_max = min_j speed·a_max_j / d_j; hình thang khi s_v_max² / s_a_max < 1, thời lượng
  1 / s_v_max + s_v_max / s_a_max; ngược lại hình tam giác, thời lượng 2·sqrt(1 / s_a_max), đỉnh
  s' = sqrt(s_a_max) (`pipeline::JointMove`, `plan_joint_move`).
- Gần kỳ dị = **số điều kiện của Jacobian CLIK > `clik.cond_threshold`** (100). Jog TCP vượt ngưỡng
  thì giảm tốc về 0 với lý do `singularity`; jog TCP bắt đầu khi đã trên ngưỡng thì bị từ chối.

**Bổ sung 2026-11-05** (PR manual-commands):

- Dải jog = `safety.soft_margin_deg` + `manual.jog_margin_deg` (mặc định 3° + 2° = 5° tính từ giới
  hạn URDF), vì 2° tính từ giới hạn URDF nằm trong dải giữ 3° của ADR-008: jog và `move_joint` dừng
  ở mép dải jog, không bao giờ chạm dải giữ.
- Mục cấu hình mới, tuỳ chọn, `cobot.manual`: `joint_a_max_rad_s2`, `tcp_v_max_m_s`,
  `tcp_w_max_rad_s`, `tcp_a_max_m_s2`, `tcp_alpha_max_rad_s2` (ở 100 % tốc độ), `deadman_ms`,
  `jog_margin_deg` (`docs/conventions.md`).
- (PR jog-headroom) `manual.joint_speed_fraction` (mặc định 0.9, trong (0, 1]): jog khớp và
  `move_joint` ở 100 % = 0.9 qd_max, vì qd bám vượt nhẹ mục tiêu qd_max và gây giữ `overspeed`.
- `HoldReason::kEstop` (chuỗi `estop`): `estop()` đưa bước kế tiếp vào `kHolding`, cần `reset`;
  phiên bản bố cục log hộp đen `kShmLogLayoutVersion` lên 3.
- Nút người chết đếm theo bước điều khiển: `deadman_s · control_rate_hz` bước không làm mới thì jog
  giảm tốc về 0 với lý do `deadman`.
- `move_tcp` dùng dải jog làm `joint_margin_rad` của `validate_path`.

**Bổ sung 2026-11-05** (PR controller-bridge):

- Phần nối chung không có ROS (`bridge::LoopPort` trên luồng điều khiển, `bridge::CommandService`
  trên luồng cầu nối) trao lệnh qua `pipeline::SpscRing` thay cho `realtime_tools::RealtimeBuffer`,
  để `cobot_controller` và `cobot_bench serve` dùng cùng một mã.
- Quyền điều khiển = token ngẫu nhiên của chủ + pid; mất nhịp 200 ms hoặc pid không còn thì dừng có
  giảm tốc một lần, trả quyền khi vòng đã ra khỏi `tracking`. Chủ đổi hoặc trả quyền khi vòng đang
  chạy lệnh cũng dừng một lần.
- Cầu nối chỉ có khi `log.mode: recorder_live`; lệnh chuyển động (`move_joint`, `move_tcp`,
  `run_waypoints`, `apply_config`) cần thêm `command_source: external`.
- Xuất đoạn = xuất Recorder vào Run CSV như thường lệ, rồi chép các dòng `[t0, t1]` của Run CSV;
  Run CSV vẫn đúng một lần.
- Panel RViz: khi studio giữ quyền, goal bị từ chối (log `goal rejected: studio is in control`),
  `~/reset` trả `studio is in control`; `~/preview_waypoints` không đổi.
- Hai host: `cobot_controller` (Gazebo, sau này Pi 5) và `cobot_bench serve` (CI, test, demo không
  Gazebo); client không đồ hoạ `cobot_bridge_cli` chạy được với cả hai (câu hỏi B3).
- Còn một tình huống tranh chấp: lệnh của chủ cũ còn trong vòng lệnh SPSC khi chủ mới giành quyền bị
  trả lời "not the owner" cho chủ mới (câu hỏi B4); không lệnh nào chạy hai lần.

**PF-02**: `cobot_bench timing --paced` (chạy theo nhịp 1 kHz bằng `clock_nanosleep`) đo thời gian
bước (p50, p99, max) với `log.mode` = `off`, `recorder`, `recorder_live` có người đọc ở lõi khác. Trên
CI chỉ chạy như báo cáo trong job `core`, gate lỏng p99 < 1 ms và không cấp phát; số thật đo tay trên
desktop/WSL và Pi 5, ghi vào `docs/results/`.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Topic/service ROS cho studio | Có sẵn, chạy được qua mạng | Studio cần ROS trong bản build mặc định; trễ của executor; để cho đường ROS 2 sau này |
| Socket hoặc gRPC | Tách tiến trình rõ ràng | Chép và tuần tự hoá mỗi mẫu, trong khi hai tiến trình vẫn cùng máy |
| Dùng chung vùng log hộp đen cho xem trực tiếp | Một vùng nhớ | Vòng đời khác (hộp đen sống sau khi thoát) và cách đọc khác (xuất đúng một lần với xem có thể mất mẫu) |
| Struct dùng chung có mutex | Dễ viết đúng | Đảo ưu tiên trên luồng thời gian thực |
| Luồng điều khiển đọc vòng lệnh | Ít luồng hơn | Kiểm tra lệnh (`validate_path`, nạp cấu hình) không giới hạn thời gian trong `update()` |

## Hệ quả

- Chi phí trên luồng điều khiển: chép một bản ghi khoảng 1 KB và vài thao tác atomic mỗi bước; ước
  tính 0,05–1 µs so với bước p95 4,9 µs (PF-01) và chu kỳ 1000 µs. PF-02 đo thật.
- Controller có thêm một luồng thường (luồng cầu nối) và một vùng nhớ chia sẻ thứ hai; container cần
  `/dev/shm` đủ cho cả hai (ADR-020).
- Panel RViz có thêm lý do bị từ chối; E2E Gazebo mới chạy bằng client không đồ hoạ: giành quyền, jog
  một khớp, `move_tcp`, chạy hai điểm, trả quyền, panel bị từ chối khi studio giữ quyền.
- Test hai tiến trình (fork): thứ tự và nội dung bản ghi, người đọc chậm thấy mất mẫu nhưng người ghi
  không chờ, sai phiên bản bị từ chối, mất nhịp 200 ms thì dừng và trả quyền.
- Studio ở máy khác qua mạng không làm được bằng cầu nối này; cần đường ROS 2 riêng sau này.
