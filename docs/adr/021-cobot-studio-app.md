# ADR-021: Ứng dụng `cobot_studio`: Qt5 Widgets, `cobot_studio_core` không Qt, chạy bench trong tiến trình

- Trạng thái: đề xuất
- Ngày: 2026-11-03
- Liên quan: thiết kế `docs/design/2026-11-02-cobot-studio-design.md` mục 2 (G3–G5), 3, 4 (quyết
  định 2, 4–11, 15, 16), 5.1, 5.2, 5.3, 6, 6.6, 6.7, 6.9, 7, 9; ADR-003, ADR-009, ADR-013, ADR-015,
  ADR-019, ADR-020; ADR-023 (giai đoạn C)

## Bối cảnh

Số liệu độ nhạy hiện nằm trong báo cáo Markdown và Summary CSV của `cobot_bench sweep`. Không có
chỗ để người xem chọn thông số, bấm chạy và nhìn nhiều lần chạy chồng lên nhau. Chủ dự án chốt:
giao diện viết bằng Qt C++ (quyết định 7), đồ thị QtCharts và chấp nhận file chạy theo GPLv3
(quyết định 8), hình 3D tự vẽ bằng Qt, không nhúng RViz (quyết định 11), quét tự động giữ lại thành
một nút (quyết định 16).

Đây là lần đầu repo có Qt, nên cần ADR trước khi thêm phụ thuộc (quy tắc của `docs/adr/README.md`).
Ràng buộc: lõi `src/core` không ROS, không Qt (ADR-003); chỉ `pipeline` đọc YAML của bộ điều khiển và
của lớp nhiễu (ADR-009); bản build mặc định và ảnh Pi 5 không đổi. Phản mục tiêu quan trọng nhất:
**cùng đầu vào mà studio cho số khác `cobot_bench run|sweep`**.

Lập kế hoạch tìm ra ba điểm thiết kế chưa khớp với code: lưu cấu hình bằng cách tuần tự hoá
`Config<N>` không chính xác (khoá `_deg` đổi độ sang rad khi nạp; rad → độ → rad không giữ nguyên
bit); ω mặc định nay khác theo profile (#51: 50 / 40 / 20, `default.yaml` 80) chứ không phải 80 chung
như thiết kế 6.7; giai đoạn A chỉ có hai tab, không phải ba như thiết kế 5.3 và 7.

## Quyết định

**Hai phần, hai target:**

- `src/studio/core` → thư viện tĩnh `cobot_studio_core` (namespace `cobot::studio`), **không Qt**, test
  được không cần màn hình. Phụ thuộc `cobot_bench_run`, `cobot::pipeline`, `yaml-cpp` (đã có trong
  repo). Build cùng bench, không thêm phụ thuộc mới. Giai đoạn C (`cobot_bench verify`, ADR-023) dùng
  lại nó.
- `src/studio/app` → file chạy `cobot_studio` (Qt5 Widgets), cùng thư viện tĩnh `cobot_studio_ui`
  chứa mọi thứ trừ `main.cpp` để test dùng được widget. Chỉ build khi bật `COBOT_BUILD_STUDIO`
  (mặc định OFF, khai báo ở `CMakeLists.txt` gốc). `tests/` được thêm sau các ứng dụng để thư mục test
  của studio kiểm `if(TARGET ...)`. Ảnh Pi 5 không build cả hai (không chép `src/studio`).
- Phụ thuộc đi một chiều: `studio/app -> studio/core -> {bench, pipeline} -> core`.

**Thư viện giao diện**: Qt 5.15 cài bằng `apt` trên Ubuntu 24.04 (cùng bản với RViz2 của Jazzy),
hiển thị trong WSL qua WSLg. Đồ thị QtCharts (`libqt5charts5-dev`) với `setUseOpenGL(true)` và rút
gọn điểm min/max theo pixel (`decimate_minmax`, luôn bật). Chủ dự án chấp nhận file chạy
`cobot_studio` theo GPLv3 vì QtCharts; mã nguồn vẫn MIT; phương án thay là Qwt (LGPL). 3D bằng Qt3D
(`Qt3DExtras`), mesh STL của `src/sim/meshes`; người xây dựng được đổi sang `QOpenGLWidget` trên cùng
`RobotScene` nếu Qt3D không đủ, và khi đó thêm mục "Bổ sung" vào ADR này. `AUTOMOC` bật theo target,
không include file `.moc`.

**Chạy bench trong tiến trình**: studio gọi `bench::run_in_memory` và `bench::summarize`, cùng code
với `cobot_bench run|sweep`, nên cùng số. Mọi lần chạy được **vật chất hoá** thành
`<work_dir>/<run-id>/{config.yaml, imperfections.yaml, studio.yaml}` chạy lại được bằng `cobot_bench`
(cùng hạt giống). Run CSV chỉ ghi khi người dùng xuất. Studio không ghi gì vào cây nguồn (thư mục làm
việc là tham số, mặc định thư mục cache của người dùng).

**Mô hình cấu hình**:

- **Cặp nền** `BasePair {name, config_yaml, imperfections_yaml}`: profile nhiễu
  `config/imperfections/<name>.yaml` cùng cấu hình bộ điều khiển `config/<name>.yaml` nếu có, không
  thì `config/default.yaml`. Thứ tự: ideal, realistic, pessimistic, premium_fr3, mid_kr1205,
  budget_xarm7, rồi các file khác theo chữ cái.
- **Ghi đè theo khoá có kiểu** (`pipeline::KeyOverride`: `double`, danh sách theo khớp, `bool`,
  `string`), tên khoá như `config/sweeps/*.yaml` (`config.<đường dưới cobot:>` hoặc đường dưới
  `imperfections:`). `pipeline` áp ghi đè lên **chính text YAML** của file nền
  (`write_config_with`, `write_imperfections_with`): giá trị không đổi giữ nguyên text gốc, nên chạy
  lại cho Run CSV giống từng byte và tránh vòng độ ↔ rad. Khoá sai báo đúng như loader chặt.
- **Khoá ảo** `config.computed_torque.omega` (ω > 0): đặt `kp = ω²`, `kd = 2ω` mọi khớp; lỗi nếu cùng
  lúc ghi đè `kp`/`kd`. `cobot_bench sweep` cũng quét được khoá này, để kiểm chéo studio với bench phủ
  cả ω. ω ban đầu của giao diện lấy từ cặp nền (`sqrt(kp_1)`): 50 / 40 / 20 cho ba profile realism-2,
  80 cho `default.yaml`.
- **File studio** (gốc `studio:`): `version`, `base` (tên và hai đường dẫn tương đối với file),
  `overrides`, `seed`, `tolerance` (mm và độ), `speed_pct`, `waypoints`, `duration_s`. `yaml-cpp`
  trong `cobot_studio_core` chỉ đọc/ghi file này: **ngoại lệ có phạm vi của ADR-009**; YAML bộ điều
  khiển và lớp nhiễu vẫn chỉ do `pipeline` đọc.
- **Giới hạn nhập**: ω trong `[10, 200]` rad/s, `safety.hal_timeout_ms` trong `[1, 50]` ms, tốc độ
  1–100 %; ngoài khoảng thì không cho chạy, ô báo lỗi nêu khoá. Thanh tốc độ hiện nhưng tắt ở giai
  đoạn A (không có nghĩa trên bench), được lưu trong file studio, dùng từ giai đoạn B.

**Chỉ số** (thiết kế 5.2) chỉ nằm trong `cobot_studio_core`, tính từ dữ liệu đã ghi; `RunSummary`,
Summary CSV và `passed` không đổi, và chỉ số nào `RunSummary` đã có thì chép, không tính lại:

- Sai số dọc đường: `|p_d − p|` và góc của `R_d Rᵀ` trên các dòng `tracking`/`finished`.
- Thời gian xác lập tại đích cuối: từ lúc tham chiếu tới đích tới lần **cuối** vào vùng cho phép mà
  sau đó ở lại tới hết cửa sổ (`goal_time_s + hold_s`); không vào hoặc lần chạy bị giữ thì "không đạt".
- Điểm `stop` giữa: tham chiếu rời đi ngay (ADR-018), nên đọc đúng chữ thiết kế ("lần đầu vào vùng
  trước khi tham chiếu rời đi") gần như luôn cho "chưa vào". Chỉ số dùng cửa sổ 0,2 s sau thời điểm
  nút như `cobot_eval` (`STOP_SETTLE_S`): sai số nhỏ nhất tới điểm trong `[t_nút, t_nút + 0,2 s]` và
  thời gian tới lần đầu vào vùng cho phép trong cửa sổ đó, không vào thì "chưa vào". Studio và
  `cobot_eval` nhờ vậy cho cùng kết luận.
- Sai số cho phép mặc định: tại đích 1 mm / 0,5°, dọc đường 2 mm / 1°. Mỗi lần chạy giữ một
  `ErrorTrace` (khoảng 40 B mỗi dòng, cả với hạt giống chỉ giữ tóm tắt), nên sửa ngưỡng chỉ tính lại,
  không chạy lại.
- Đường "nhiễu" theo nguồn (ADR-020): cảm biến `q − q_true`; động cơ `taum[k+1] − tau[k]` (dịch một
  dòng); ma sát `tauf`; sai mô hình, jitter, truyền thông không có đường nhiễu, nút xám kèm lý do.

**Quét**: một tham số (khoá quét được trong danh mục, gồm khoá bộ điều khiển), danh sách mức, hạt
giống mặc định `[1, 2, 3]` (chủ dự án). Kiểm mọi mức trước khi chạy lần đầu. Bảng ghi trường hợp xấu
nhất qua các hạt giống; đồ thị vẽ hạt giống đầu, chỉ hạt giống đó giữ bản ghi trong bộ nhớ. Huỷ được
giữa hai lần chạy. `SweepRunner` có tham số `max_parallel` (mặc định 1).

**Giao diện**: giai đoạn A có hai tab, **Configuration** và **Tracking**; tab Control đến ở giai đoạn
B (`studio-control`, ADR-022). Chuỗi hiển thị bằng tiếng Anh, bọc trong `tr()`; bản dịch tiếng Việt
(`.ts`) làm sau (chủ dự án). Tab Tracking có danh sách lần chạy chồng nhau (mỗi lần một màu, chú
thích ghi khác biệt so với cặp nền), đồ thị theo khớp hoặc task, bảng sai số (ô vượt ngưỡng tô đỏ),
nút Run và Sweep, khung 3D phát lại. Khung 3D vẽ "q thật" (`qt`) khi hữu hạn, không thì `q`; lần
chạy đang chọn vẽ đặc, tối đa ba lần khác vẽ mờ; quả cầu sai số cho phép vẽ đúng kích thước kèm hệ
số phóng đại sai lệch (×1..×1000, mặc định ×1) vì 1 mm không nhìn thấy ở tỉ lệ cánh tay.

**Luồng**: luồng GUI sở hữu `RunStore` (không thread-safe, chỉ luồng GUI chạm). Chạy và quét trên một
`QThread` (`StudioWorker`), mỗi kết quả sang luồng GUI bằng tín hiệu queued; huỷ bằng
`std::atomic<bool>`; nạp mesh cũng ngoài luồng GUI. Đóng cửa sổ khi đang quét thì huỷ và chờ.

**CI**: job `core` và `lint` cài `qtbase5-dev libqt5charts5-dev qt3d5-dev xvfb libgl1-mesa-dri` và
cấu hình với `-DCOBOT_BUILD_STUDIO=ON` (`lint` cần vậy để clang-tidy thấy include Qt). Test ứng dụng
chạy `QT_QPA_PLATFORM=offscreen`; test 3D chạy dưới `xvfb-run`, bỏ qua sạch khi không tạo được
context OpenGL. Các job khác không đổi; test của `cobot_studio_core` chạy ở mọi job có bench.
`tools/wsl/build.sh --studio` bật tuỳ chọn một cách tường minh.

**Móc cho giai đoạn C** (ADR-023, không làm bây giờ, không được chặn): cặp nền đến từ một bộ dò, nên
`config/hardware/<model>.yaml` chỉ thêm mục; ghi đè có kiểu, nên `control.algorithm` là một mục danh
mục nữa; `Tolerance` là struct các yêu cầu có tên; `SweepRunner` nằm trên bộ chạy danh sách lần chạy
có `max_parallel`, Monte Carlo 100 lần là một người gọi nữa; mỗi lần chạy tái tạo được từ
`config.yaml` + `imperfections.yaml` + hạt giống (hash đầu vào cho báo cáo).

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Giao diện Python/PyQt đọc CSV | Viết nhanh, gần `cobot_eval` | Cài lại lần chạy và chỉ số lần thứ hai; đúng phản mục tiêu "studio cho số khác `cobot_bench`" |
| Giao diện web | Chạy mọi nơi | Thêm máy chủ và cả bộ JS cho một công cụ chạy cục bộ |
| Nhúng RViz | 3D sẵn, quen | Cần ROS trong bản build mặc định; để cho `COBOT_STUDIO_ROS` sau này |
| Gọi `cobot_bench` thành tiến trình con mỗi lần chạy | Tách biệt hoàn toàn | Ghi rồi đọc CSV mỗi lần, không có bản ghi trong bộ nhớ, huỷ và tiến độ khó hơn |
| Dear ImGui | Nhẹ, nhanh | Không có widget đồ thị và bảng, phải tự làm |
| Qwt (LGPL) | File chạy không bị GPLv3 | QtCharts đủ và chủ dự án chấp nhận GPLv3; Qwt là phương án dự phòng |
| Lưu cấu hình bằng cách tuần tự hoá `Config<N>` | Một hàm | Vòng độ ↔ rad không giữ bit, chạy lại không giống từng byte |

## Hệ quả

- File chạy `cobot_studio` theo GPLv3 (QtCharts); mã nguồn và các target khác vẫn MIT. README ghi rõ.
- CI tốn thêm phút cho cài Qt và build ứng dụng ở `core` và `lint`; bản build mặc định và ảnh Pi 5
  không đổi.
- Bộ nhớ mỗi lần chạy lưu trong studio khoảng 12 MB ở 12 s với các kênh mới; hạt giống thứ hai trở
  đi của quét chỉ giữ tóm tắt và `ErrorTrace`. Thanh trạng thái hiện tổng bộ nhớ.
- Phải đo: quét trên studio và `cobot_bench sweep` cùng đầu vào cho mọi trường `RunSummary` bằng
  nhau từng bit (gồm `omega`); luồng GUI không đứng quá 250 ms khi chạy, quét, nạp mesh; vẽ lại đồ
  thị 10 lần chạy × 7 đường × 12 000 dòng dưới 100 ms (đo tay).
- Giai đoạn C dùng lại `cobot_studio_core` cho `cobot_bench verify` và tab Kiểm nghiệm (ADR-023).

## Bổ sung (2026-11-06): tab Điều khiển, thanh tốc độ, lần chạy trực tiếp

- **Ba tab theo thứ tự thiết kế 5.3**: Cấu hình, Điều khiển (`controlTab`), Theo dõi. Tab Điều khiển
  dùng `BridgeClient` của `cobot_studio_core` (không Qt, ADR-022) qua vùng `/cobot_<name>`, tên chọn
  bằng `--bridge` (mặc định `cobot_controller`).
- **Thanh tốc độ được bật**: `speedSlider` của tab Cấu hình và `controlSpeedSlider` là một giá trị,
  `StudioConfig::speed_pct`, dùng cho jog và các lệnh di chuyển.
- **Lần chạy trực tiếp lấy từ đoạn Recorder**: mỗi chuyển động (lệnh di chuyển, waypoint, jog) kết
  thúc ở mẫu standby/holding đầu tiên; khi đã có mẫu muộn hơn `settle_window_s` = 1 s, studio yêu cầu
  host xuất đoạn `[t_start, t_end + 1 s]` của Run CSV. Luồng worker đọc đoạn đó (`read_run_csv`) và
  tính tóm tắt bằng chính `bench::summarize`, nên số trong tab Theo dõi giống cách tính của bench; luồng
  GUI không đọc file.
- **Nhịp tim chạy trên luồng GUI, cố ý**: `bridgeTimer` 50 ms gọi `tick`; GUI đứng thì nhịp tim dừng,
  controller dừng robot sau 200 ms và trả quyền điều khiển. Khi GUI chạy lại sau một khoảng lặng dài
  hơn 200 ms, client tự nhả quyền thay vì gia hạn. Đóng cửa sổ cũng nhả quyền ngay.
- **Khoá dòng không hỗ trợ trên Gazebo**: khi đã kết nối, các khoá controller từ chối hoặc bỏ qua
  (payload, friction, transmission, `sensor.position_source`, timing, comm) bị làm mờ trong cây tham
  số.
