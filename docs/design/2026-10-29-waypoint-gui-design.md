# Giao diện nhập điểm và quỹ đạo nhiều điểm (thiết kế)

- Ngày: 2026-10-29
- Trạng thái: chờ duyệt
- Phụ thuộc: thiết kế gốc `docs/design/2026-10-28-cpp-rework-design.md`, ADR-003 (lõi không
  ROS), ADR-007 (điều khiển hai tầng), ADR-008 (luồng và thời gian thực), ADR-011 (mô phỏng)

## 1. Ý định và lý do

MVP bám đúng một quỹ đạo cố định đọc từ `config/default.yaml`: khởi động là chạy, không nhận
lệnh. Chủ dự án muốn tự đưa vào một điểm cuối hoặc một danh sách điểm, để hệ thống nội suy ra
một quỹ đạo chung rồi cánh tay bám theo trong Gazebo.

Lý do: biến bộ điều khiển từ một bài kiểm tra cố định thành một hệ có thể "cầm lên dùng" khi
trình bày phỏng vấn (đặt điểm, xem trước đường đi, bấm chạy, xem sai số), và đặt nền cho bước
lên phần cứng Pi5 sau này, nơi lệnh cũng đến từ bên ngoài.

## 2. Mục tiêu, không mục tiêu, phản mục tiêu

**Mục tiêu**

1. Giao diện trong RViz2: marker 3D kéo thả để đặt vị trí và hướng của một điểm; panel bảng để
   thêm, sửa, xoá, sắp xếp điểm, lưu và nạp file YAML; các nút Xem trước, Chạy, Dừng.
2. Một điểm cuối là trường hợp riêng của danh sách một điểm; không có đường đi riêng cho nó.
3. Quỹ đạo nhiều điểm trong lõi C++ (không ROS): mặc định lướt mượt qua các điểm giữa, dừng hẳn
   ở điểm có cờ `stop` và ở điểm cuối.
4. Xem trước trước khi chạy: đường đi vẽ trong RViz2; điểm không với tới, chạm giới hạn khớp,
   vượt vận tốc khớp hoặc vượt mô-men được tô đỏ kèm lý do; còn điểm đỏ thì không chạy được.
5. Trong lúc chạy: tiến độ (đoạn hiện tại) và sai số bám; khi xong: sai số cuối cùng loại số
   liệu với báo cáo E2E.
6. Dừng bất kỳ lúc nào: giảm tốc êm rồi giữ tư thế.
7. Bench và CI vẫn chạy được không cần GUI: cùng file YAML danh sách điểm dùng được cho
   `cobot_bench` và cho kịch bản E2E mới.

**Không mục tiêu (lần này)**

- Tránh va chạm, vật cản, môi trường; quy hoạch kiểu MoveIt.
- Điểm cho theo góc khớp (chỉ nhận tư thế Cartesian của đầu công cụ trong `base_link`).
- Đổi danh sách khi đang chạy (phải Dừng trước), ghép nối tiếp nhiều lệnh.
- Điều khiển tay (jog), tay cầm, teleop.
- Phần cứng Pi5.

**Phản mục tiêu** (có chạy cũng coi là hỏng)

- Quy hoạch hoặc cấp phát bộ nhớ trong `update()` thời gian thực của controller.
- Lõi `core/` phụ thuộc ROS hoặc Qt.
- Cánh tay chuyển động khi xem trước đã báo đỏ, hoặc bắt đầu từ một tư thế khác tư thế hiện tại
  (giật).
- Phá các kịch bản đang có: E2E-01..09 và CI giữ nguyên ngưỡng và kết quả.

## 3. Ràng buộc

- ROS 2 Jazzy, Gazebo Harmonic, Qt5 có sẵn trong Jazzy; không cài thêm gói ngoài apt của ROS.
- C++17, chuẩn mã và quy trình như phần còn lại của repo (test viết trước, test đóng băng sau
  khi implement, mỗi PR một tính năng, CI xanh mới merge).
- Giới hạn khớp lấy từ URDF của từng khớp (`velocity`: 2.5 rad/s cho khớp 1–4, 3 rad/s cho khớp
  5–7; `effort`, `lower`/`upper`, `acceleration`), không hard-code.
- Vòng 1 kHz; mọi thứ chạy trong `update()` phải không cấp phát, không ném ngoại lệ (ADR-008).

## 4. Đã chốt với chủ dự án

- Nền tảng GUI: RViz2 với interactive marker và một panel plugin C++/Qt (người thiết kế đề xuất,
  chủ dự án đồng ý).
- Nhập được một điểm cuối hoặc danh sách điểm; hệ thống nội suy thành một quỹ đạo chung.
- Trước mắt chỉ làm mô phỏng.

## 5. Quyết định của người thiết kế (chủ dự án có thể đổi)

- Qua điểm giữa: lướt mượt mặc định, cờ `stop` theo từng điểm để dừng hẳn.
- Thời gian mỗi đoạn: tuỳ chọn theo điểm; để trống thì tự tính từ giới hạn vận tốc và gia tốc
  Cartesian trong cấu hình, rồi kéo dài nếu xem trước thấy vượt giới hạn khớp.
- Khi khởi động ở chế độ giao diện, cánh tay giữ tư thế home và chờ lệnh.
- Không nhận danh sách mới khi đang chạy (từ chối lệnh, giữ lệnh cũ).
- Thời lượng do người dùng nhập là ràng buộc: nếu vượt giới hạn thì báo đỏ, không tự kéo dài.
  Chỉ đoạn để trống thời lượng mới được tự tính và tự kéo dài.
- Dừng là giảm tốc **dọc theo đường đã kiểm tra** (co giãn thời gian), không phanh theo twist.
- Điểm là tư thế của mặt bích `link_7`. Cấu hình có thêm `tcp_offset` (mặc định đơn vị, tức
  không có dụng cụ) để sau này gắn dụng cụ chỉ cần đổi cấu hình.
- Lỗi an toàn (chốt giữ khớp hiện có) không tự hết: GUI có nút Đặt lại, chỉ có tác dụng khi
  nguyên nhân đã hết; khi đó vòng về chế độ chờ tại tư thế đo được.

## 6. Kiến trúc

```
RViz2                          controller cobot_controller (ros2_control)          core/ (không ROS)
┌──────────────────────┐       ┌────────────────────────────────────────┐         ┌────────────────────┐
│ panel: bảng điểm,    │ action│ luồng không thời gian thực:            │         │ planning::         │
│ Lưu/Nạp, Xem trước,  ├──────►│  ~/follow_waypoints (action server)    ├────────►│  WaypointTrajectory│
│ Chạy, Dừng, tiến độ  │service│  ~/preview_waypoints (service)         │         │ planning::         │
│ marker 3D kéo thả    ├──────►│  → dựng + kiểm tra quỹ đạo             │         │  validate_path     │
│ hiển thị đường đi    │◄──────┤ RealtimeBuffer (đổi con trỏ, không cấp │         │ pipeline::         │
└──────────────────────┘ topic │ phát trong update)                     │         │  ControlLoop       │
                               │ update() 1 kHz: ControlLoop::step      │         └────────────────────┘
                               └────────────────────────────────────────┘
```

Theo mẫu của `joint_trajectory_controller` trong ros2_controllers: action server sống trong
controller, callback của action chạy ngoài luồng thời gian thực, quỹ đạo dựng xong được đưa vào
vòng 1 kHz qua `realtime_tools::RealtimeBuffer`. Không thêm node riêng.

### 6.1 Lõi: `planning::WaypointTrajectory`

- Đầu vào: tư thế bắt đầu (tư thế tham chiếu hiện tại của vòng điều khiển, không phải tư thế
  trong file), danh sách điểm `{Pose T_b_goal; double duration_s (≤ 0 = tự tính); bool stop}`,
  giới hạn Cartesian `{v_max, a_max, w_max, alpha_max}`.
- Đầu ra: `sample(t)` trả `CartesianSample` như `CartesianTrajectory` hiện có (tư thế, twist,
  đạo hàm twist), để CLIK và computed torque không phải đổi.
- Yêu cầu hình học và thời gian:
  - Đi qua đúng từng điểm (vị trí và hướng) tại thời điểm nút của nó.
  - Vị trí và hướng liên tục bậc 1 (C1) tại điểm lướt qua; vận tốc và gia tốc bằng 0 tại điểm
    `stop`, điểm đầu và điểm cuối.
  - Hướng nội suy trên SO(3) (không dùng Euler), cùng quy ước log/exp với `CartesianTrajectory`.
  - Thời lượng tự tính bảo đảm vận tốc và gia tốc Cartesian không vượt giới hạn trên mỗi đoạn.
- Một điểm, `stop` = true: trùng với `CartesianTrajectory` hiện có (quintic điểm-điểm) trong
  sai số làm tròn; kiểm bằng test đơn vị so sánh mẫu của hai lớp.
- Lời giải cụ thể (Hermite bậc 5 từng đoạn với tiếp tuyến tại nút tính từ các điểm lân cận, hoặc
  spline khác) để lại cho người xây dựng; đổi lại phải chứng minh các yêu cầu trên bằng test.
- Giới hạn số điểm: hằng số biên dịch (đề xuất 32) để không cấp phát khi lấy mẫu.

### 6.2 Lõi: kiểm tra trước khi chạy `planning::validate_path`

Chạy CLIK ngoại tuyến (cùng gain, cùng bộ hạn chế khớp) từ góc khớp hiện tại dọc quỹ đạo ở
bước 1 ms. Trả về theo từng điểm và từng đoạn:

| Lỗi | Điều kiện |
| --- | --- |
| `unreachable` | sai số tư thế của CLIK tại điểm > 1 mm hoặc > 0.5° |
| `joint_limit` | góc khớp vào vùng đệm 2° sát giới hạn URDF |
| `joint_velocity` | \|q̇\| > giới hạn URDF |
| `torque` | RNEA trên tham chiếu (có armature, có bù ma sát danh định) > 90 % `τ_max` |

Mô hình dùng để kiểm tra là đúng mô hình của controller (trên Gazebo là mô hình đã
`perturb_model`).

Kèm theo: đường đi lấy mẫu (để vẽ), thời lượng mỗi đoạn sau khi kéo dài. Với đoạn để trống
thời lượng, nếu chỉ vượt vận tốc khớp hoặc mô-men, hàm thử kéo dài đoạn đó (tối đa ×3) trước khi
báo lỗi; đoạn có thời lượng do người dùng nhập thì báo lỗi ngay. Action chạy đúng kết quả của
lần kiểm tra này (dựng lại cùng hàm, cùng đầu vào), nên Xem trước và Chạy không thể khác nhau.
Ngưỡng nằm trong cấu hình, không hard-code.

### 6.3 `ControlLoop`

- Thêm chế độ nhận lệnh ngoài: `command_source: config | external` trong `cobot` của
  `config/default.yaml`; mặc định `config`. Chế độ `config` giữ nguyên đường code hiện tại
  (`CartesianTrajectory`), nên E2E-01..09 và các test đã đóng băng không đổi.
- Chế độ mới `kStandby` (khác `kHolding`): tiếp tục CLIK + computed torque bám một tư thế
  Cartesian cố định, nên luôn có tư thế tham chiếu xác định. `kHolding` vẫn là chốt an toàn như
  cũ (PD khớp + trọng lực, không tự thoát).
- `external`: sau `start()` vòng vào `kStandby` tại tư thế đo được.
- `load(trajectory)`: chỉ chấp nhận ở `kStandby`; tư thế đầu của quỹ đạo phải trùng tư thế tham
  chiếu của standby (sai lệch > 0.1 mm hoặc 0.01° là từ chối). Quỹ đạo dựng ngoài luồng thời
  gian thực rồi đổi con trỏ; standby giữ tham chiếu cố định nên không có chạy đua.
- `stop()`: co giãn thời gian dọc quỹ đạo đang chạy, `ṡ` giảm về 0 theo một luật đóng (không
  cấp phát, tính trong `update()`), trong thời gian cấu hình (mặc định 0.5 s); xong thì vào
  `kStandby` tại điểm dừng. Vì vẫn trên đường đã kiểm tra nên không vào vùng chưa xét.
- `reset()`: từ `kHolding`, chỉ khi điều kiện gây chốt đã hết (khớp ra khỏi vùng đệm, HAL hợp
  lệ), vào `kStandby` tại tư thế đo được.
- Ghi CSV thêm giá trị `LoopMode` mới; công cụ đánh giá phải đọc được (cập nhật
  `docs/conventions.md`).
- Trạng thái công khai cho feedback: chỉ số đoạn, thời gian, sai số vị trí và hướng tức thời,
  chế độ.

### 6.4 Giao tiếp ROS (gói mới `cobot_interfaces`)

- Action `FollowWaypoints`:
  - Goal: `geometry_msgs/Pose[] poses` (trong `base_link`), `float64[] duration_s`,
    `bool[] stop`.
  - Feedback: `segment`, `t_s`, `pos_err_m`, `rot_err_deg`.
  - Result: `success`, `message`, `final_pos_err_m`, `final_rot_err_deg`,
    `rms_joint_err_rad`, `peak_tau_ratio`.
  - Hủy goal = Dừng.
- Service `PreviewWaypoints`: cùng đầu vào, trả `nav_msgs/Path`, danh sách lỗi theo điểm,
  thời lượng các đoạn.
- Topic `~/reference_path` (`nav_msgs/Path`, latched) để RViz2 vẽ đường đã nạp.

### 6.5 Giao diện (gói mới `cobot_gui`)

- Panel RViz2 (C++/Qt5): bảng điểm (x, y, z, roll, pitch, yaw độ, thời gian, Dừng), Thêm từ
  marker, Xoá, Lên/Xuống, Lưu/Nạp YAML, Xem trước, Chạy, Dừng, Đặt lại; vùng trạng thái hiện tiến độ,
  sai số, lỗi xem trước.
- Interactive marker 6 bậc tự do cho điểm đang chọn; marker cho mọi điểm (số thứ tự, đỏ nếu lỗi).
- Phần "mô hình dữ liệu" của panel (danh sách điểm, chuyển YAML ↔ goal, đổi rpy ↔ quaternion)
  tách khỏi Qt để test không cần màn hình.
- Launch: thêm `rviz:=true` vào `sim.launch.py`; khi bật, controller chạy `command_source
  external` và RViz2 mở với cấu hình có sẵn panel.

### 6.6 File danh sách điểm (dùng chung GUI, bench, E2E)

```yaml
frame: base_link
waypoints:
  - {position_m: [-0.5, -0.6, 0.9], rpy_deg: [0, 0, 90], duration_s: 0, stop: false}
  - {position_m: [-0.3, -0.6, 0.7], rpy_deg: [0, 0, 90], duration_s: 4.0, stop: true}
```

`cobot_bench run --waypoints FILE` dùng file này thay quỹ đạo trong cấu hình.

## 7. Kiểm thử

- Đơn vị (lõi): đi qua đúng điểm; liên tục C1 tại điểm lướt (so sánh trái/phải với dung sai);
  vận tốc, gia tốc bằng 0 tại điểm `stop`; giới hạn Cartesian được tôn trọng; một điểm trùng
  `CartesianTrajectory`; danh sách rỗng, quá 32 điểm, quaternion không chuẩn, thời lượng âm đều bị
  từ chối có thông báo; `validate_path` bắt được từng loại lỗi trong bảng 6.2.
- `ControlLoop`: `load` bị từ chối khi đang chạy hoặc tư thế đầu lệch; `stop` giảm tốc về 0
  dọc đường đi, không vượt `τ_max`; `reset` bị từ chối khi nguyên nhân chốt còn; chế độ `config` không đổi hành vi (E2E-01..04 như cũ).
- Mô hình dữ liệu panel: YAML ↔ goal khứ hồi, rpy ↔ quaternion.
- E2E-10 (bench, realistic): 4 điểm, có một điểm `stop`; ngưỡng sai số cuối như E2E-02, cộng
  sai số vị trí tại mỗi điểm `stop` ≤ 5 mm.
- E2E-11 (Gazebo, CI): client Python gửi action với danh sách của E2E-10; kiểm tra result và
  feedback; thêm một lần Hủy giữa chừng kiểm tra cánh tay dừng và giữ.
- GUI: build được trên CI (job gazebo); không test giao diện bằng màn hình.

## 8. Chia PR (dự kiến)

1. `waypoint-trajectory`: lõi `WaypointTrajectory` + test.
2. `path-validation`: `validate_path` + test.
3. `loop-external-command`: `ControlLoop` `command_source`, `load`, `stop` + bench `--waypoints`
   + E2E-10.
4. `waypoint-interfaces`: gói `cobot_interfaces`, action và service trong controller,
   RealtimeBuffer, E2E-11.
5. `waypoint-gui`: gói `cobot_gui`, panel, marker, launch `rviz:=true`, tài liệu.
6. Cập nhật README, ADR mới (ADR-018: lệnh ngoài và quỹ đạo nhiều điểm) và kết quả.

## 9. Để lại cho người xây dựng

- Công thức nội suy cụ thể và cách chọn tiếp tuyến tại điểm lướt, miễn thoả mục 6.1.
- Bố cục chi tiết của panel, biểu tượng, màu (đỏ cho điểm lỗi là bắt buộc).
- Giá trị mặc định của giới hạn Cartesian (ghi rõ nguồn trong cấu hình).
- Cách vẽ đường đi (Path hay MarkerArray), miễn RViz2 hiển thị được trước khi chạy.
