# Thiết kế: ứng dụng cobot_studio (cấu hình, điều khiển, bám quỹ đạo) và bộ ghi log trong bộ nhớ

- Trạng thái: đã duyệt (2026-11-02)
- Ngày: 2026-11-02
- Làm sau: `docs/design/2026-11-02-realism-2-design.md` (realism-2 làm trước, dùng ADR-019)
- Liên quan: ADR-003, ADR-007, ADR-008, ADR-012, ADR-013, ADR-015, ADR-018. ADR mới viết trong PR
  đầu tiên của loạt này: **ADR-020** (bộ ghi log trong bộ nhớ, chế độ log), **ADR-021** (ứng dụng
  `cobot_studio`), **ADR-022** (cầu nối bộ nhớ chia sẻ và các chế độ điều khiển tay). Giai đoạn C
  (kiểm nghiệm kịch bản phần cứng, mục 10, thêm 2026-11-03): **ADR-023**

## 1. Ý định và lý do

Chủ dự án cần **trình bày cho người xem** (phỏng vấn, demo) hai điều:

1. Bộ điều khiển chịu được bao nhiêu nhiễu từ **từng nguồn** (cảm biến, động cơ, hộp số, truyền
   thông, ma sát, sai mô hình, jitter; gồm các nguồn của realism-2) mà sai số vẫn nằm trong vùng
   chính xác, và phản hồi nhanh tới đâu.
2. Robot **điều khiển được như robot thật** từ một giao diện: từng khớp, đầu công cụ (TCP), và theo
   danh sách điểm; chạy trên Gazebo, để sát robot thật, cùng plugin controller sẽ chạy trên Pi 5.

Hiện tại các con số độ nhạy nằm trong báo cáo Markdown và Summary CSV của `cobot_bench sweep`; điều
khiển chỉ có panel RViz chạy danh sách điểm (ADR-018). Không có chỗ để người xem chọn thông số, chạy
và nhìn các đường chồng lên nhau, cũng không điều khiển tay được.

Phần ghi log hiện tại (ring SPSC + luồng ghi CSV chạy suốt lần chạy, ADR-008, ADR-013) định dạng
text và ghi đĩa **trong lúc** vòng điều khiển chạy; bench còn bắt vòng điều khiển chờ khi đĩa chậm.
Chủ dự án muốn hàm log **chỉ chép biến vào bộ nhớ**; chạy xong hoặc có tín hiệu "lấy log" mới xuất.

`cobot_studio` là **giao diện điều khiển** của dự án, viết bằng Qt C++.

## 2. Mục tiêu, không mục tiêu, phản mục tiêu

Mục tiêu, giai đoạn A (bench, không cần Gazebo):

- **G1. Bộ ghi log trong bộ nhớ** (`pipeline::Recorder<N>`): cấp phát trước, mỗi bước chỉ chép một
  bản ghi; xuất CSV khi lần chạy kết thúc hoặc khi có yêu cầu. Thay ring + luồng ghi hiện tại ở cả
  bench và node ROS. Chế độ log chọn lúc chạy (`log.mode`), không bằng cờ build.
- **G2. Đủ dữ liệu để vẽ**: q'' tham chiếu, trạng thái thật trước lớp nhiễu (q, q', q''), mô-men
  động cơ sau lớp nhiễu, mô-men ma sát; từ đó tính được các đường "nhiễu".
- **G3. Tab Cấu hình**: mọi thông số ở một chỗ (nhiễu từng nguồn, bộ điều khiển, sai số cho phép,
  tốc độ), lưu/mở YAML.
- **G4. Tab Bám quỹ đạo**: kết quả của các lần chạy: đồ thị theo thời gian (bật/tắt từng đường,
  chồng nhiều lần chạy), bảng sai số, nút **Chạy** (bench, một lần) và nút **Quét** (bench, một nguồn
  nhiều mức, tự chạy hết rồi chồng lên).
- **G5. Khung 3D robot** trong tab Bám quỹ đạo: phát lại đồng bộ với con trỏ thời gian, chồng các lần
  chạy, vết đầu công cụ và vùng sai số cho phép.

Mục tiêu, giai đoạn B (Gazebo):

- **G6. Cầu nối bộ nhớ chia sẻ** giữa `cobot_controller` (đang chạy trong Gazebo, sau này trên Pi 5)
  và `cobot_studio` (tiến trình riêng): vòng trạng thái và vòng lệnh, luồng điều khiển không chờ.
- **G7. Tab Điều khiển** với ba chế độ: từng khớp, TCP, chạy theo điểm (soạn, dạy điểm, xem trước,
  chạy); mỗi chế độ khớp/TCP có cả **giữ nút** và **nhập giá trị đích**. Khung 3D trực tiếp, nút
  Dừng và Dừng khẩn luôn hiện.
- **G8. Lần chạy trên Gazebo vào tab Bám quỹ đạo** như lần chạy bench (cùng đồ thị, bảng, 3D).
- **G9. Chứng minh không ảnh hưởng vòng điều khiển**: test PF-02 đo thời gian bước có và không có
  studio đọc dữ liệu.

Không mục tiêu (làm sau):

- Quét tự động trên Gazebo (quét chỉ chạy bench; Gazebo chạy tay từng lần rồi so sánh).
- Studio ở máy khác qua mạng (đường ROS 2 cho cùng giao diện "nguồn chạy"); tab RViz nhúng
  (`COBOT_STUDIO_ROS`).
- Tránh va chạm, lập quỹ đạo khớp nhiều điểm (chế độ điểm vẫn là quỹ đạo Cartesian của ADR-018).
- Thay báo cáo Python (`cobot_eval`): giữ nguyên, vẫn đọc Run CSV.

Phản mục tiêu (dù chạy được vẫn coi là hỏng):

- Bước điều khiển cấp phát heap, khoá mutex, chờ, định dạng text hoặc chạm đĩa vì log hay vì studio.
- Studio treo, chậm hay chết làm robot tiếp tục chạy theo lệnh tay (phải dừng trong thời gian cố định).
- Giao diện đứng hình trong lúc chạy hay quét.
- Cùng đầu vào mà studio cho chỉ số khác `cobot_bench run`/`sweep`.
- Hai nguồn lệnh (studio và panel RViz) cùng lúc điều khiển robot.

## 3. Ràng buộc

- C++17, quy ước `CONTRIBUTING.md`; đường nóng không cấp phát, không exception (ADR-008).
- Lõi `src/core` không phụ thuộc ROS hay Qt (ADR-003). Qt chỉ ở `src/studio`; studio không phụ thuộc
  ROS ở bản build mặc định.
- Qt5 (cùng bản với RViz2 của Jazzy), phụ thuộc cài bằng `apt` trên Ubuntu 24.04. Hiển thị trong WSL
  qua WSLg. Gazebo và studio chạy **cùng máy** (WSL, sau này Pi 5 có màn hình) nên dùng được bộ nhớ
  chia sẻ POSIX.
- Kết quả bench tất định: E2E-03 (so CSV bằng byte) vẫn phải qua.
- CI giữ đúng 9 check bắt buộc; mỗi PR một tính năng; test đóng băng chỉ đổi với nhãn `test-change`.
- License MIT.

## 4. Đã chốt với chủ dự án

1. Mục đích: chứng minh mức nhiễu chịu được theo từng nguồn trong vùng chính xác, và tốc độ phản hồi;
   kèm điều khiển robot từ giao diện.
2. So sánh **nhiều mức nhiễu / nhiều lần chạy trên cùng một đồ thị**; mỗi đường bật/tắt được.
3. Log chỉ ghi biến vào bộ nhớ; xuất khi chạy xong hoặc khi có tín hiệu.
4. Trong ứng dụng: chọn thông số, bấm chạy, so sánh ngay.
5. Chọn **từng nguồn nhiễu**, không nhân đều.
6. **Sai số cho phép nhập từ giao diện.**
7. Qt C++; ADR mới cho bộ ghi log và cho giao diện.
8. Đồ thị dùng QtCharts (chấp nhận file chạy `cobot_studio` theo GPLv3).
9. Quét: mỗi mức chạy 3 seed (mặc định `[1, 2, 3]`), bảng ghi trường hợp xấu nhất, đồ thị vẽ seed đầu.
10. **Tham số bộ điều khiển chỉnh từ giao diện**: hệ số computed-torque (ω hoặc kp/kd từng khớp) và
    thời gian chờ ngắt `safety.hal_timeout_ms`; cả hai quét được.
11. **Hình 3D robot tự vẽ bằng Qt**, không nhúng RViz; nhúng RViz chỉ khi có chế độ ROS 2 sau này.
12. **Giao diện điều khiển có ngay**, điều khiển robot trên **Gazebo** (sát robot thật).
13. Ba chế độ điều khiển: **từng khớp**, **TCP**, **theo điểm** (tạo quỹ đạo).
14. Chế độ từng khớp và TCP: **giữ nút** (chạy liên tục, thả ra dừng) **và nhập giá trị đích**.
15. Bố cục theo tab: **Cấu hình** (cài đặt thông số), **Bám quỹ đạo** (đồ thị và bảng sai số của
    quỹ đạo vừa chạy). Điều khiển ở tab riêng (đề xuất của người thiết kế, chủ dự án không phản đối).
16. **Quét tự động giữ lại**, thành một nút trong tab Bám quỹ đạo.
17. Studio là tiến trình song song, giao tiếp với luồng điều khiển qua **vòng bộ nhớ chia sẻ**.
18. **Log hộp đen sống qua sự cố**: bộ ghi log của controller nằm trong vùng nhớ chia sẻ, xuất được
    sau khi controller hoặc Gazebo chết (mục 6.1).

## 5. Quyết định của người thiết kế (chủ dự án có thể đổi)

### 5.1 Log

- **Chế độ log lúc chạy, không cờ build**: khoá `log.mode: off | recorder | recorder_live`
  (`config/default.yaml`, mặc định `recorder`; `recorder_live` khi có cầu nối studio). Lý do: kiểm
  thử đúng bản sẽ chạy thật, CI không nhân đôi job, log hộp đen dùng được ngoài hiện trường mà không
  build lại; chi phí khi tắt là một nhánh đoán trước được. Cờ build chỉ chọn **ứng dụng nào** được
  build (`COBOT_BUILD_STUDIO`, sau này `COBOT_STUDIO_ROS`).
- **Đường "nhiễu"** là sai lệch do lớp nhiễu tạo ra, chỉ có trong mô phỏng. "Thật" là trạng thái
  **trước** decorator `ImperfectJointInterface`.
  - Nguồn cảm biến (`sensor.*`, `transmission` khi encoder ở phía động cơ): `q đo − q thật`.
  - Nguồn động cơ (`actuator.*`): `τ động cơ − τ lệnh`, τ động cơ là mô-men **sau** scale, ripple,
    độ phân giải, trễ lệnh, hằng số thời gian motor và các giới hạn của realism-2. Trong bench, trễ
    motor và ma sát nằm trong `SimPlant` (decorator tạo với `apply_plant_effects=false`), nên
    `SimPlant` trả ra mô-men động cơ sau trễ và mô-men ma sát của bước vừa tính.
  - Ma sát (`friction.*`): đường nhiễu là mô-men ma sát. Trên Gazebo ma sát nằm trong vật lý, cột để
    `nan`.
  - Sai mô hình (`model.*`), jitter (`timing.*`), truyền thông (`comm.*`): không có đường nhiễu (nút
    "nhiễu" xám, có chú thích); xem qua đường sai số. Vẫn quét được.
- **q''** là gia tốc của plant (bench: `SimPlant`; Gazebo: `nan`, vì interface khớp không có gia
  tốc), và `q''_ref` của bộ sinh tham chiếu.

### 5.2 Chỉ số

- **Sai số cho phép**: bốn ô, mặc định tại điểm đích 1 mm / 0,5° (như `path_validation.reach_*`),
  trên cả quỹ đạo 2 mm / 1°.
- **Sai số trên quỹ đạo**: giữa `T_b_e` và `T_b_d` ở mọi dòng mode `tracking` hoặc `finished`; dòng
  `holding`/`fault` không tính; lần chạy có `holding` ghi thời điểm và lý do ngắt (cột `hold_time_s`,
  `hold_reason` của realism-2).
- **Thời gian xác lập**: tính ở **điểm cuối** của mỗi lần chạy: từ lúc tham chiếu tới đích đến lúc sai
  số task vào vùng cho phép tại đích **và ở lại** tới hết cửa sổ quan sát (bench: `hold_s`; chế độ
  external/Gazebo: lần chạy ghi thêm `settle_window_s` = 1 s sau khi tham chiếu dừng); không vào được
  thì "không đạt". Ở **điểm `stop` giữa** (tham chiếu rời đi ngay, ADR-018 không có thời gian dừng),
  bảng ghi sai số lúc tham chiếu tới điểm và thời gian tới lần **đầu** vào vùng cho phép trước khi tham
  chiếu rời đi, hoặc "chưa vào".
- **Chỉ số mới chỉ nằm trong `cobot_studio_core`**, tính từ dữ liệu đã ghi. `RunSummary`, Summary CSV
  và `passed` của `cobot_bench sweep` không đổi ngoài các cột realism-2 thêm; kiểm tra khớp với bench
  trên các chỉ số `RunSummary` sẵn có.

### 5.3 Giao diện

- **Ba tab**: Cấu hình, Điều khiển, Bám quỹ đạo (mục 6.7–6.9).
- **Thư viện đồ thị**: QtCharts (Qt5, `libqt5charts5-dev`), OpenGL và rút gọn điểm theo pixel. File
  chạy `cobot_studio` theo GPLv3 (mã nguồn vẫn MIT). Phương án thay nếu đổi ý: Qwt (LGPL).
- **3D**: Qt3D (`Qt3DExtras`), mesh STL của `src/sim/meshes`; người xây dựng được đổi sang
  `QOpenGLWidget` nếu Qt3D không đủ.
- **Giới hạn nhập**: ω trong `[10, 200]` rad/s, thời gian chờ ngắt trong `[1, 50]` ms, tốc độ
  1–100 %; ngoài khoảng không cho chạy/áp dụng.
- Build bằng CMake gốc (không phải gói colcon), tuỳ chọn `COBOT_BUILD_STUDIO` (mặc định OFF); job
  `core` của CI cài Qt và bật nó. Ảnh Pi 5 không build studio (headless).

### 5.4 Điều khiển tay và an toàn

- **Một nguồn lệnh tại một thời điểm**: studio "giành quyền" qua cầu nối; khi studio giữ quyền,
  action/service ADR-018 của panel RViz bị từ chối với lý do "studio đang điều khiển" (xem vẫn được).
  Quyền tự trả khi studio đóng hoặc mất nhịp sống.
- **Nhịp sống**: studio ghi nhịp mỗi 50 ms. Mất nhịp quá **200 ms**: đang chạy lệnh tay hay quỹ đạo
  thì dừng có giảm tốc (như `request_stop` của ADR-018) rồi về standby, và trả quyền.
- **Giữ nút (jog)**: lệnh jog phải được làm mới mỗi ≤ 50 ms; không nhận lệnh mới trong **100 ms**
  (nút người chết) thì giảm tốc về 0 rồi standby. Thả nút gửi lệnh dừng ngay.
- **Tốc độ chung** (thanh trượt, mặc định **10 %**): nhân giới hạn vận tốc và gia tốc khớp (jog khớp,
  đi tới khớp) và giới hạn TCP (jog TCP, đi thẳng). Giới hạn TCP 100 %: 0,25 m/s, 1,0 rad/s. Quỹ đạo
  theo điểm dùng thời lượng đã kiểm tra của ADR-018, không theo thanh tốc độ.
- **Chỉ đổi chế độ và áp dụng cấu hình khi robot ở standby** (đứng yên).
- **Dừng**: giảm tốc dọc đường đang đi (jog: về vận tốc 0; quỹ đạo: co giãn thời gian ADR-018), rồi
  standby. **Dừng khẩn**: vào `kHolding` ngay (giữ tại chỗ), cần `reset` như hiện nay.
- **Giới hạn mềm khớp**: jog khớp và jog TCP dừng cách giới hạn URDF **2°**; jog TCP dừng khi gần kỳ
  dị (chỉ số manipulability dưới ngưỡng; người xây dựng chọn ngưỡng và ghi vào ADR-022) và báo lý do.

## 6. Kiến trúc

```
src/core/pipeline   Recorder<N>, LogRecord<N> (thêm cột), ControlLoop: ghi Recorder, các lệnh tay
src/core/bridge     (mới, không ROS, không Qt) bố cục bộ nhớ chia sẻ, vòng trạng thái, vòng lệnh,
                    nhịp sống; dùng chung cho controller và studio
src/bench           run engine dùng Recorder; API chạy trả dữ liệu trong bộ nhớ
src/ros2_node       dùng Recorder; ~/dump_log; máy chủ cầu nối (giai đoạn B)
src/studio/
  core/             cobot_studio_core (không Qt): cấu hình, quét, chỉ số, mô hình 3D, client cầu nối
  app/              cobot_studio: Qt5 Widgets, ba tab, đồ thị, bảng, khung 3D
```

Phụ thuộc: `studio/app -> studio/core -> {bench, core/bridge} -> core`. `ros2_node -> core/bridge`.

### 6.1 `pipeline::Recorder<N>` (ADR-020)

- Mảng bản ghi cấp phát **một lần** khi tạo, dung lượng tính bằng số bước (thời lượng × tần số, hoặc
  `log.capacity_s`, mặc định 60 s cho node ROS).
- `record(const LogRecord<N>&)` trên luồng điều khiển: chép vào ô kế tiếp, tăng chỉ số. Không cấp
  phát, không khoá, không I/O. Khi đầy: `kStopWhenFull` (bench: ngừng ghi, đếm số bỏ) hoặc
  `kOverwriteOldest` (node ROS: giữ cửa sổ gần nhất).
- `Recorder` thuộc về **host** (bench, controller), `ControlLoop` nhận tham chiếu tới nó; dựng lại
  `ControlLoop` (áp dụng cấu hình) không làm mất log.
- Đọc: bench đọc sau khi vòng dừng. Node ROS: yêu cầu "lấy log" là cờ atomic; bước kế tiếp luồng điều
  khiển **đổi** bộ đệm đang ghi với bộ đệm dự phòng cấp phát sẵn (O(1)), luồng khác xuất bộ đệm cũ.
- `export_csv(path)` dùng `write_csv_header`/`write_csv_row` hiện có, ngoài luồng điều khiển.
- `ControlLoop` ghi vào `Recorder` thay cho `SpscRing log_`; `last_record()` giữ nguyên.
- **Hộp đen sống qua sự cố** (chủ dự án chốt): bộ nhớ của `Recorder` có hai loại:
  - `HeapStorage` (bench, test): như trên.
  - `ShmStorage` (controller trên Gazebo và Pi 5): cả hai bộ đệm nằm trong vùng nhớ chia sẻ riêng
    `/cobot_log_<tên controller>` (khác vùng cầu nối). Đầu vùng: magic, phiên bản bố cục, `N`, kích
    thước bản ghi, dung lượng, chỉ số bộ đệm đang ghi, chỉ số ghi của mỗi bộ đệm (atomic), cờ "đã
    xuất". Bản ghi được chép xong **rồi** mới tăng chỉ số (release), nên bản ghi đang chép dở lúc sập
    không bao giờ được tính.
  - Controller hoặc Gazebo chết: vùng nhớ vẫn còn tới khi bị xoá hoặc khởi động lại máy. Lệnh mới
    `cobot_logdump <tên controller> [--out file]` (trong `src/bench`, không ROS) xuất CSV từ vùng nhớ;
    studio có nút tương ứng.
  - Controller khởi động mà thấy vùng nhớ cũ chưa "đã xuất": tự xuất ra
    `<log_path>.recovered-<thời điểm>.csv` (luồng thường, trước khi activate) rồi mới dùng lại vùng.
  - Kích thước: 60 s × 1 kHz × khoảng 1 KB × 2 bộ đệm ≈ 120 MB. `log.capacity_s` chỉnh được; vùng nhớ
    được `mlock` khi khoá mới `log.lock_memory: true` (mặc định false; bật trên Pi 5) để không bị
    đẩy ra swap; `mlock` thất bại thì cảnh báo và chạy tiếp.

### 6.2 Dữ liệu thêm (`LogRecord<N>`, hợp đồng Run CSV)

Thêm vào cuối dòng (cột cũ giữ thứ tự):

| Trường | Cột CSV | Ai điền |
| --- | --- | --- |
| `qdd_ref` | `qddref1..N` | `ControlLoop` |
| `q_true`, `qd_true` | `qt1..N`, `qdt1..N` | host, từ interface bên trong decorator |
| `qdd_true` | `qddt1..N` | host (bench: `SimPlant`; Gazebo: `nan`) |
| `tau_motor` | `taum1..N` | host: mô-men động cơ sau lớp nhiễu và trễ motor |
| `tau_friction` | `tauf1..N` | host (bench: `SimPlant`; Gazebo: `nan`) |
| `cmd_mode` | `cmd` | `ControlLoop`: `config`, `waypoints`, `jog_joint`, `jog_tcp`, `move_joint`, `move_tcp` |

Robot thật ghi `nan` cho các giá trị "thật". Đổi hợp đồng: sửa `docs/conventions.md`, writer C++,
reader `cobot_eval` và test hai phía trong cùng PR, nhãn `test-change` (ADR-013).

### 6.3 Bench

- Run engine bỏ luồng ghi CSV song song: ghi vào `Recorder`, xuất CSV **sau** vòng lặp. Thêm API chạy
  trả về dữ liệu đã ghi và `RunSummary` không cần file, cho `studio/core`.
- `cobot_bench run|montecarlo|sweep` giữ dòng lệnh và đầu ra.

### 6.4 Lệnh tay trong `ControlLoop` (lõi, ADR-022)

Thêm vào chế độ `command_source: external` (chỉ nhận ở `kStandby`, trừ jog làm mới và dừng):

| Lệnh | Tham chiếu sinh ra | Kiểm tra trước khi nhận |
| --- | --- | --- |
| `jog_joint(j, dir)` | vận tốc khớp j tăng/giảm theo hình thang tới `v_max·speed`; `q_ref`, `qd_ref`, `qdd_ref` đưa thẳng vào computed torque, **bỏ qua CLIK**; các khớp khác giữ | khớp hợp lệ, chưa ở giới hạn mềm theo chiều đó |
| `move_joint(q_goal)` | các khớp đồng bộ tới đích cùng lúc, hồ sơ vận tốc hình thang (hoặc bậc 5) trong `v_max·speed`, `a_max·speed`; bỏ qua CLIK | trong giới hạn mềm; mô-men dọc đường kiểm bằng mô hình như `validate_path` |
| `jog_tcp(axis, dir, frame)` | twist trên một trục (x, y, z, rx, ry, rz) theo hệ gốc hoặc hệ dụng cụ, tăng/giảm hình thang tới giới hạn TCP·speed; tư thế mong muốn tích phân, đi qua CLIK như hiện nay | không gần kỳ dị, không ở giới hạn mềm |
| `move_tcp(T_goal)` | đường thẳng: một `WaypointTrajectory` một điểm `stop`, thời lượng theo giới hạn TCP·speed | `validate_path` của ADR-018 (cùng hàm với Xem trước) |
| `load(waypoints)` | như ADR-018 | như ADR-018 |
| `stop()`, `estop()`, `reset()` | mục 5.4 | — |

Jog dừng ở giới hạn mềm hay gần kỳ dị: giảm tốc về 0, ghi lý do vào bản ghi (`hold_reason`/thông
báo trạng thái), **không** vào `kHolding`. Mọi lệnh không cấp phát khi thành công. Các kiểm tra an
toàn hiện có (ADR-008) áp dụng như mọi chế độ.

### 6.5 Cầu nối bộ nhớ chia sẻ `core/bridge` (ADR-022)

- **Vùng nhớ**: `shm_open("/cobot_<tên controller>")` + `mmap`, do controller tạo khi activate, xoá
  khi cleanup. Đầu vùng: magic, phiên bản bố cục, `N`, kích thước bản ghi, dung lượng các vòng, PID
  studio đang giữ quyền, nhịp sống. Studio kiểm tra cả bốn giá trị đầu trước khi dùng; khác thì báo
  lỗi phiên bản, không đọc.
- Chỉ dùng kiểu trivially copyable và `std::atomic<uint64_t>` có `is_always_lock_free`
  (`static_assert`).
- **Vòng trạng thái** (controller → studio): 4096 ô `LogRecord<N>` (khoảng 4 s ở 1 kHz). Luồng điều
  khiển ghi **ghi đè ô cũ, không bao giờ chờ**, mỗi ô có số thứ tự kiểu seqlock; studio đọc, phát hiện
  ô bị ghi đè trong lúc đọc thì bỏ ô đó và đếm "mất mẫu hiển thị". Log đầy đủ vẫn nằm trong
  `Recorder`, nên studio chậm chỉ làm hình thưa đi.
- **Vòng lệnh** (studio → controller): SPSC 64 ô lệnh kích thước cố định. **Luồng điều khiển không
  đọc vòng này**: một luồng thường của controller (luồng cầu nối) lấy lệnh, kiểm tra (quyền, chế độ,
  giới hạn), chạy phần nặng (`validate_path`, nạp cấu hình) ngoài thời gian thực, rồi chuyển vào
  `update()` qua cơ chế sẵn có của ADR-018 (`RealtimeBuffer`, cờ atomic). Lệnh jog đi qua một ô
  seqlock (chiều, trục, thời điểm làm mới) mà `update()` đọc không chờ.
- **Dữ liệu của một lần chạy Gazebo** lấy từ `Recorder` (đầy đủ), không từ vòng trạng thái (chỉ để
  hiển thị trực tiếp, có thể mất mẫu): khi lần chạy kết thúc (về standby sau `settle_window_s`), studio
  gửi lệnh "xuất đoạn [t0, t1]"; luồng cầu nối ghi CSV vào thư mục tạm và trả đường dẫn; studio nạp vào
  `RunStore`. Lần chạy dài hơn cửa sổ `log.capacity_s` bị đánh dấu "cắt đầu".
- **Phản hồi lệnh**: vòng nhỏ thứ ba (controller → studio, luồng cầu nối ghi): chấp nhận/từ chối kèm
  lý do và kết quả xem trước.
- **Áp dụng cấu hình từ studio trên Gazebo**: studio gửi đường dẫn file YAML (cấu hình + nhiễu) đã lưu;
  chỉ nhận khi standby. Luồng cầu nối nạp và dựng **toàn bộ** phần như `on_configure` hiện làm (mô hình
  bị nhiễu theo `model.*` và seed, `ImperfectJointInterface`, `ControlLoop`) ngoài thời gian thực;
  `update()` đổi con trỏ ở bước kế tiếp. Bộ mới bắt đầu ở standby tại tư thế đo hiện tại (CLIK và các
  hàng trễ khởi tạo lại), nên không có bước nhảy tham chiếu. `Recorder` và vùng nhớ chia sẻ thuộc
  controller, sống qua lần đổi. `control_rate_hz` không đổi được (phải bằng tần số update). Các khoá Gazebo không hỗ trợ (payload, jitter, ma sát của lớp nhiễu, như
  `cobot_controller` hiện cảnh báo) hiện xám trong tab Cấu hình khi đích là Gazebo.
- Chi phí luồng điều khiển: chép một bản ghi (khoảng 1 KB) và vài thao tác atomic mỗi bước. Ước tính
  0,05–1 µs so với bước p95 4,9 µs (PF-01) và chu kỳ 1000 µs; PF-02 đo thật.

### 6.6 `cobot_studio_core` (không Qt, test được)

- `StudioConfig`: cấu hình bộ điều khiển + nhiễu + sai số cho phép + tốc độ; nạp/lưu YAML cùng tên
  khoá với `config/default.yaml` và `config/imperfections/*.yaml`.
- `SweepSpec`, `SweepRunner`: một tham số (tên khoá như `config/sweeps/*.yaml`, gồm khoá bộ điều
  khiển), danh sách mức, seed; chạy từng mức bằng API bench 6.3; tiến độ; huỷ giữa hai lần chạy.
- `RunStore`: các lần chạy (bench hay Gazebo) kèm cấu hình đã dùng; giữ tới khi xoá.
- `metrics`: chỉ số mục 5.2, dùng chung định nghĩa với `RunSummary`.
- `RobotScene`: đọc URDF (khâu, mesh, gốc visual), tư thế các khâu bằng `forward_kinematics`.
- `BridgeClient`: mở vùng nhớ, đọc vòng trạng thái, gửi lệnh, nhịp sống; một lần chạy trên Gazebo
  (từ lúc nhận lệnh tới khi về standby) thành một mục trong `RunStore`.

### 6.7 Tab Cấu hình

- Profile nền (ideal, realistic, pessimistic, ba profile realism-2), các nguồn nhiễu theo nhóm, sửa
  từng ô; giá trị khác profile nền hiện màu khác.
- Bộ điều khiển: ω (mặc định từ `config/default.yaml`, hiện 80 rad/s, kp = ω², kd = 2ω), chế độ nâng
  cao kp/kd từng khớp, `hal_timeout_ms`.
- Sai số cho phép (bốn ô), tốc độ chung, file quỹ đạo cho bench.
- Lưu/mở YAML, "Về mặc định", "Áp dụng lên Gazebo" (chỉ khi standby).

### 6.8 Tab Điều khiển

- Trạng thái nối (Gazebo đang chạy hay không, ai giữ quyền), nút Giành quyền / Trả quyền.
- **Từng khớp**: mỗi khớp một hàng: góc hiện tại, nút giữ −/+, ô góc đích; nút "Đi" cho cả bảy ô.
- **TCP**: tư thế hiện tại (x, y, z, rx, ry, rz), chọn hệ gốc/dụng cụ, nút giữ −/+ mỗi trục, ô tư thế
  đích, nút "Đi".
- **Theo điểm**: bảng điểm sửa được, "Thêm tư thế hiện tại" (dạy điểm), Xem trước (đường đi và lỗi
  từng điểm), Chạy; lưu/mở file điểm YAML của ADR-018.
- Khung 3D trực tiếp (vết đầu công cụ, đường xem trước), thanh tốc độ, nút **Dừng** và **Dừng khẩn**
  luôn hiện, nút Reset khi đang giữ.

### 6.9 Tab Bám quỹ đạo

- Danh sách lần chạy (bench, quét, Gazebo), chọn để chồng; mỗi lần một màu, chú thích ghi khác biệt
  cấu hình so với profile nền.
- Đồ thị: trục thời gian, chọn khớp 1–7 hoặc "task"; bật/tắt q_ref, q, q', q'', nhiễu, sai số; phóng
  to/kéo; con trỏ thời gian.
- Bảng sai số: mỗi lần chạy một dòng (quét: worst-case qua seed), cột ở mục 5.2, cột cấu hình khác
  biệt, ngắt (thời điểm, lý do), tỉ lệ ngắt; ô vượt ngưỡng tô đỏ; sửa ngưỡng thì tính lại, không chạy
  lại.
- Nút **Chạy** (bench với cấu hình hiện tại) và **Quét** (chọn một tham số, nhập các mức); worker trên
  `QThread`, tiến độ, Huỷ.
- Khung 3D phát lại: con trỏ thời gian điều khiển thời điểm; phát/tạm dừng, 0,25× / 1× / 4×; lần chạy
  đang chọn vẽ đặc, các lần khác bóng mờ cùng màu; vết đầu công cụ, đường mong muốn, điểm đích, quả
  cầu sai số cho phép, dấu tại thời điểm ngắt. Dùng "q thật" khi có.
- Xuất CSV: Run CSV từng lần và Summary CSV.

## 7. Kiểm thử

- `Recorder`: không cấp phát trong `record()` (đếm cấp phát như các test no-alloc hiện có), hai chính
  sách khi đầy, đổi bộ đệm, xuất CSV đúng hợp đồng.
- Bench: E2E-03 vẫn qua; `cobot_bench sweep` cho Summary CSV như trước; PF-01 p95 không tăng.
- Cột mới: q_true khớp q đo khi profile ideal; `tau_motor = tau` và `tau_friction = 0` khi không nhiễu
  động cơ và ma sát; `qdd_ref` khớp đạo hàm số của `qd_ref`.
- Lệnh tay (lõi, bench): jog khớp đạt đúng vận tốc `v_max·speed` và dừng trong quãng giảm tốc khi
  thả; nút người chết dừng sau 100 ms không làm mới; jog dừng cách giới hạn 2°; `move_joint` các khớp
  tới đích cùng lúc trong giới hạn; jog TCP giữ hướng khi đi theo x (sai số hướng dưới ngưỡng) và dừng
  khi gần kỳ dị; `move_tcp` từ chối đích ngoài tầm với bằng cùng lỗi với Xem trước; lệnh khi không ở
  standby bị từ chối; không cấp phát.
- Cầu nối: hai tiến trình (fork trong test): thứ tự và nội dung bản ghi đúng; người đọc chậm thấy "mất
  mẫu" nhưng người ghi không bao giờ chờ; phiên bản bố cục sai bị từ chối; mất nhịp sống 200 ms →
  dừng và trả quyền; hai client giành quyền → client thứ hai bị từ chối.
- **PF-02**: `cobot_bench timing --paced` (thêm chế độ chạy theo nhịp 1 kHz bằng `clock_nanosleep`)
  đo thời gian bước (p50, p99, max) với `log.mode` = `off`, `recorder`, `recorder_live` có người đọc
  ở lõi khác. Trên CI chỉ chạy như báo cáo trong job `core` (gate lỏng như PF-01: p99 < 1 ms, không
  cấp phát); con số thật đo tay trên desktop/WSL và Pi 5, ghi vào `docs/results/`.
- Hộp đen: tiến trình con chạy bench với `ShmStorage` bị `kill -9` giữa chừng; `cobot_logdump` xuất
  đủ mọi bản ghi tới bước cuối đã hoàn tất, không có dòng hỏng; lần khởi động sau tự xuất file
  `.recovered`; phiên bản bố cục sai bị từ chối.
- Gazebo: test hiện có vẫn qua; harness E2E Gazebo gọi `~/dump_log` trước khi tắt launch, và dùng
  `cobot_logdump` nếu file chưa có (sửa harness đóng băng, nhãn `test-change`); `~/dump_log` sinh CSV; E2E mới: studio client không đồ hoạ giành
  quyền, jog một khớp, `move_tcp`, chạy hai điểm, trả quyền; panel RViz bị từ chối khi studio giữ quyền.
- `cobot_studio_core`: chỉ số trên dữ liệu biết trước; quét một tham số cho cùng `RunSummary` như
  `cobot_bench sweep`; tư thế khâu cuối của `RobotScene` trùng `forward_kinematics` (< 1e-9 m); mọi
  mesh nạp được.
- `cobot_studio`: smoke test `QT_QPA_PLATFORM=offscreen`: mở ba tab, chạy bench ngắn, quét 2 mức,
  bảng đủ dòng, khung 3D đặt con trỏ thời gian, xuất CSV.

## 8. Chia PR (dự kiến)

Giai đoạn A:

1. `docs`: ADR-020, ADR-021, ADR-022, bản thiết kế này.
2. `pipeline-recorder`: `Recorder<N>` với `HeapStorage`, `log.mode`, `ControlLoop` dùng nó, bench
   xuất sau khi chạy.
2b. `blackbox-log`: `ShmStorage`, `cobot_logdump`, tự khôi phục khi khởi động; node ROS dùng nó;
   `~/dump_log`; harness Gazebo.
3. `log-channels`: cột mới trong `LogRecord`/Run CSV, host điền, `cobot_eval` đọc được.
4. `studio-core`: `StudioConfig`, `SweepSpec`/`SweepRunner`, `RunStore`, metrics, API chạy trong bộ
   nhớ của bench.
5. `studio-app`: Qt, tab Cấu hình và Bám quỹ đạo (Chạy, Quét, đồ thị, bảng); CI bật
   `COBOT_BUILD_STUDIO`.
6. `studio-3d`: `RobotScene`, khung 3D phát lại.

Giai đoạn B:

7. `manual-commands`: lệnh tay trong `ControlLoop` (mục 6.4), test bench.
8. `shm-bridge`: `core/bridge`, test hai tiến trình, PF-02.
9. `controller-bridge`: máy chủ cầu nối trong `cobot_controller`, quyền điều khiển, nhịp sống, áp dụng
   cấu hình; E2E Gazebo bằng client không đồ hoạ.
10. `studio-control`: tab Điều khiển, `BridgeClient`, lần chạy Gazebo vào tab Bám quỹ đạo.
11. `docs`: README (lệnh chạy, ảnh chụp), kết quả mẫu và số đo PF-02 trong `docs/results/`.

Sau loạt này: đường ROS 2 cho studio ở máy khác; tab RViz nhúng (`COBOT_STUDIO_ROS`); chạy trên Pi 5.

## 9. Để lại cho người xây dựng

- Cấu trúc bên trong `Recorder` (struct of arrays hay mảng `LogRecord`), miễn giữ ràng buộc 6.1.
- Bố cục byte cụ thể của vùng nhớ chia sẻ, miễn có các trường đầu vùng ở 6.5 và kiểm phiên bản.
- Hồ sơ vận tốc của `move_joint` (hình thang hay bậc 5) và ngưỡng gần kỳ dị, ghi vào ADR-022.
- Cách rút gọn điểm khi vẽ, bảng màu, chi tiết tương tác đồ thị và 3D.
- Danh sách tham số trong tab Cấu hình lấy từ schema hay liệt kê tay, miễn khớp tên khoá YAML.

## 10. Giai đoạn C: kiểm nghiệm kịch bản phần cứng (thêm 2026-11-03)

Chủ dự án chốt hướng ngày 2026-11-03, quyết định ở **ADR-023**. Mục này mô tả phạm vi; chi tiết
giao diện và tên kiểu là quyết định của người xây dựng, trừ những gì ADR-023 đã cố định.

### 10.1 Ý định

Người dùng hỏi: "với phần cứng này, thuật toán này, tải này, robot có làm được nhiệm vụ của tôi
không?". Studio trả lời bằng một kết luận ba mức (ĐẠT, KHÔNG ĐẠT, SÁT NGƯỠNG, CẦN ĐO THẬT) kèm
giả định, biên an toàn và độ tin cậy. Đây là công cụ sàng lọc và so sánh trước khi mua hoặc chế
tạo, không phải chứng nhận an toàn.

Lý do (từ #51): dự án đã đưa ra được loại kết luận này một cách thủ công (bản giá rẻ cần vòng điều
khiển trong driver; bản cao cấp đạt khi ω = 50 nhưng trượt khi ω = 80). Giai đoạn C làm nó lặp
lại được, có thống kê, và không phụ thuộc người chỉnh hệ số tay.

### 10.2 Mục tiêu

- **G10. Model phần cứng** `config/hardware/<model>.yaml`: profile nhiễu, giới hạn khớp, hệ số mặc
  định; ba model mốc từ ADR-019. Model thiếu `source` thì không nạp được.
- **G11. Chọn thuật toán** `control.algorithm` từ danh sách đăng ký lúc build: computed torque (hiện
  tại, kết quả bit-identical), PD cộng bù trọng lực, vòng vị trí trong driver.
- **G12. Nhiệm vụ và yêu cầu** `config/tasks/<task>.yaml`: quỹ đạo hoặc danh sách điểm, tải, ngưỡng
  sai số, thời gian chu kỳ, tỉ lệ dừng an toàn chấp nhận được, tỉ lệ đạt mục tiêu.
- **G13. Tự chỉnh hệ số** cho mỗi tổ hợp model và thuật toán (ADR-023), tắt được.
- **G14. Kết luận ba mức** từ Monte Carlo (mặc định 100 lần chạy, Wilson 95 %, biên 20 %), lệnh
  `cobot_bench verify` không giao diện và tab **Kiểm nghiệm** trong studio cho cùng kết quả.
- **G15. Báo cáo** Markdown và JSON: kết luận, khoảng tin cậy, chỉ số xấu nhất, danh sách giả định
  và độ nhạy của kết luận theo từng giả định, hệ số đã chọn, nguyên nhân và gợi ý khi không đạt,
  phiên bản và hash đầu vào.
- **G16. So sánh**: chạy một nhiệm vụ trên nhiều model hoặc nhiều thuật toán, một bảng kết luận cạnh
  nhau.

Không mục tiêu (làm sau): kiểm chứng mô hình từ log robot thật (nhận dạng ma sát, độ cứng, trễ);
tối ưu chọn phần cứng tự động; đánh giá an toàn va chạm theo ISO/TS 15066.

Phản mục tiêu (dù chạy được vẫn coi là hỏng):

- Kết luận ĐẠT không kèm danh sách giả định và khoảng tin cậy.
- Kết luận ĐẠT từ chế độ xem nhanh hoặc từ ít hơn số lần chạy cần thiết cho mục tiêu.
- Kết luận KHÔNG ĐẠT trên hệ số chưa chỉnh mà báo cáo không nói rõ.
- Studio và `cobot_bench verify` cho kết luận khác nhau trên cùng đầu vào.
- Chọn thuật toán qua cấu hình làm chậm bước điều khiển (PF-01) hoặc đổi kết quả computed torque.

### 10.3 Điều kiện tiên quyết

- Giai đoạn A (bộ ghi log, studio-core, tab Cấu hình và Bám quỹ đạo).
- Thuật toán **vòng vị trí trong driver**: driver mô phỏng chạy vòng vị trí và dòng điện ở vài kHz,
  máy chủ gửi điểm đặt vị trí ở tần số servo của profile. Làm trước kết luận đầu tiên (ADR-023).

### 10.4 Chia PR (dự kiến)

1. `docs`: ADR-023 và mục này (PR này).
2. `controller-registry`: giao diện chọn thuật toán, computed torque và PD cộng bù trọng lực; E2E-03
   và PF-01 không đổi.
3. `drive-loop`: thuật toán vòng vị trí trong driver; profile giá rẻ được kiểm lại.
4. `hardware-catalog`: `config/hardware/`, `config/tasks/`, loader và test khoá.
5. `auto-tune`: quét hệ số, tiêu chí chọn, bảng quét trong kết quả.
6. `verify`: `cobot_bench verify`, Monte Carlo song song, Wilson, kết luận ba mức, báo cáo
   Markdown và JSON, E2E trên ba model mốc.
7. `studio-verify`: tab Kiểm nghiệm và bảng so sánh (G16).
8. `docs`: kết quả mẫu trên ba model mốc trong `docs/results/`, README.

Sau giai đoạn C: kiểm chứng mô hình từ log robot thật, khi có phần cứng.
