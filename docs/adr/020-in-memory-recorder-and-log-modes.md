# ADR-020: Bộ ghi log trong bộ nhớ `Recorder<N>`, chế độ log lúc chạy, log hộp đen và kênh dữ liệu mới

- Trạng thái: đề xuất
- Ngày: 2026-11-03
- Liên quan: thiết kế `docs/design/2026-11-02-cobot-studio-design.md` mục 2 (G1, G2), 4 (quyết
  định 3, 18), 5.1, 6.1, 6.2, 6.3, 7, 8 (PR 2, 2b, 3); ADR-003, ADR-008, ADR-012, ADR-013

## Bối cảnh

Từ MVP, `ControlLoop` đẩy mỗi bước một `LogRecord<N>` vào `SpscRing<LogRecord<N>, 8192>`; host có
một luồng ghi rút ring, định dạng text và ghi CSV **trong lúc** vòng điều khiển chạy (ADR-008,
ADR-013). Bench còn bắt vòng điều khiển chờ luồng ghi khi đĩa chậm (`kMaxInFlight`). Node ROS ghi
`log_path` và flush theo từng lô; harness E2E Gazebo (`tools/run_e2e.py`) đọc đuôi `run.csv` trong
lúc chạy để biết khi nào lần chạy kết thúc.

Chủ dự án muốn hàm log **chỉ chép biến vào bộ nhớ**, xuất khi chạy xong hoặc khi có tín hiệu
(quyết định 3), và log của controller **sống qua sự cố** của controller hoặc Gazebo (quyết định
18). `cobot_studio` (ADR-021) cần thêm dữ liệu để vẽ: gia tốc tham chiếu, trạng thái thật của
plant trước lớp nhiễu, mô-men plant thật sự nhận, mô-men ma sát.

Lập kế hoạch tìm ra bốn điểm thiết kế chưa khớp với code, ghi lại ở đây:

1. `LogRecord<N>` **không trivially copyable** (kiểu cố định của Eigen 3.4 có copy constructor do
   người dùng định nghĩa), nên `static_assert(trivially copyable)` của thiết kế 6.5 không áp dụng được
   cho nó.
2. "Host điền `tau_motor`" (thiết kế 6.2) không khớp dòng: bản ghi được viết trong `step()`, trước
   khi plant áp dụng lệnh của bước đó.
3. `/dev/shm` mặc định của Docker là 64 MB; hộp đen 60 s cần khoảng 66 MB trước khi thêm kênh và
   khoảng 120 MB sau.
4. Bỏ `ControlLoop::log()` làm hỏng node ROS ngay ở PR đầu, nên node phải đổi cùng lúc, giữ việc ghi
   `run.csv` liên tục mà harness Gazebo cần.

## Quyết định

**`pipeline::Recorder<N>`** (`src/core/pipeline/include/cobot/pipeline/recorder.hpp`):

- Bộ nhớ cấp phát **một lần** khi tạo, dung lượng tính bằng số bước. Bộ nhớ thuộc về **host**
  (bench, controller) qua một storage (`HeapStorage<N>` hoặc `ShmStorage<N>`) trao cho `Recorder` một
  khung nhìn `RecorderMemory<N>` (con trỏ khối điều khiển `RecorderControl`, một hoặc hai bộ đệm, số
  thứ tự từng ô, dung lượng).
- `ControlLoop(model, config, Recorder<N>* recorder = nullptr)`: con trỏ null nghĩa là không ghi log.
  `step()` gọi `recorder_->record(record)`; `last_record()` giữ nguyên. Dựng lại `ControlLoop` (áp
  dụng cấu hình) với cùng `Recorder` thì log nối tiếp, không mất. `log()`, `log_`, `kLogCapacity` bị
  bỏ; `spsc_ring.hpp` ở lại cho vòng lệnh của giai đoạn B.
- `record()` chạy trên luồng điều khiển: chép một bản ghi vào ô kế tiếp, O(1), `noexcept`. Không cấp
  phát, không khoá, không I/O, không định dạng text, không gọi hàm ảo.
- Hai chính sách khi đầy (`FullPolicy`):
  - `kStopWhenFull` (bench): dung lượng bằng số chu kỳ của lần chạy, nên không bao giờ bỏ bản ghi;
    `dropped() > 0` vẫn là lỗi như hiện nay.
  - `kOverwriteOldest` (controller): giữ cửa sổ `log.capacity_s` gần nhất; `overwritten()` đếm số
    bản ghi bị đè.
- **Hai bộ đệm và đổi O(1)**: luồng xuất gọi `request_swap()` (cờ atomic); lần `record()` kế tiếp
  đổi bộ đệm đang ghi và báo `swap_done()` (acquire/release). Luồng xuất đọc `retired()`, ghi CSV,
  rồi `release_retired()`. Yêu cầu đổi khi bộ đệm cũ chưa được trả, hoặc khi đang có yêu cầu chờ, bị
  từ chối: một bộ đệm không bao giờ có hai chủ.
- CSV ghi **ngoài luồng điều khiển** bằng `write_csv_header`/`write_csv_row` sẵn có (`write_csv`,
  `export_csv`; header chỉ khi file mới hoặc rỗng). Cấu trúc bên trong (mảng `LogRecord` hay struct
  of arrays) do người xây dựng chọn (thiết kế 9).

**Chế độ log lúc chạy, không cờ build** (`pipeline::Config<N>`, khoá dưới `cobot.log`):

| Khoá | Kiểu, mặc định | Ý nghĩa |
| --- | --- | --- |
| `cobot.log.mode` | `off \| recorder \| recorder_live`, `recorder` | `LogMode::kOff/kRecorder/kRecorderLive`. `recorder_live` được nhận và chạy như `recorder` cho tới khi có cầu nối `shm-bridge` (ADR-022) |
| `cobot.log.capacity_s` | số > 0, `60.0` | cửa sổ của controller (`kOverwriteOldest`) |
| `cobot.log.lock_memory` | bool, `false` | `mlock` vùng log; bật trên Pi 5; `mlock` thất bại thì cảnh báo, chạy tiếp |

`cobot.log.output_dir` giữ nguyên. Bench luôn ghi log bất kể `log.mode` (đầu ra của bench chính là
log); `cobot_bench timing` theo `log.mode` (`off`: không recorder, để PF-01 đo được cả hai). Bench
từ chối trước khi cấp phát nếu log của lần chạy cần quá 2 GiB, báo `duration_s`. Bốn file cấu hình
bộ điều khiển (`config/default.yaml` và ba file realism-2) có cùng ba khoá.

**Nơi chứa bộ nhớ**: heap (`HeapStorage`, bench và test) hoặc vùng nhớ chia sẻ POSIX
`/cobot_log_<tên controller>` (`ShmStorage`, `recorder_shm.hpp`, chỉ Linux) cho controller:

- Đầu vùng: magic, phiên bản bố cục (`kShmLogLayoutVersion`, tăng khi đổi `LogRecord` hay đầu vùng),
  `N`, kích thước bản ghi, dung lượng, **dấu vân tay bố cục** của `LogRecord<N>` (hash của offset và
  kích thước từng trường, `N`, `sizeof(Pose)`), bộ đếm đã ghi và đã xuất tăng đơn điệu, số thứ tự
  từng ô. Bố cục byte cụ thể do người xây dựng chọn. Mở vùng có sẵn kiểm theo thứ tự magic, phiên
  bản, `N`, kích thước bản ghi, dấu vân tay, dung lượng; sai thì lỗi nêu tên trường và hai giá trị.
- Bản ghi được chép xong **rồi** mới công bố bộ đếm (release), người đọc dùng acquire: tiến trình chết
  giữa chừng không bao giờ để lộ dòng chép dở. Khi bộ đệm `kOverwriteOldest` đã quay vòng, ô mà lần
  `record()` kế tiếp sẽ đè không bao giờ được xuất. Qua mọi xen kẽ của đổi bộ đệm, `~/dump_log`,
  deactivate, `cobot_logdump` và khôi phục khi khởi động, mỗi bản ghi đã công bố được xuất đúng một
  lần, đúng thứ tự.
- **Khác thiết kế 6.5**: `LogRecord<N>` là standard layout (`static_assert`) nhưng không trivially
  copyable, nên vùng log **chỉ được đọc bởi cùng bản build** (dấu vân tay khớp). Cầu nối của giai đoạn
  B không đặt `LogRecord<N>` lên vùng nhớ chung mà dùng bản ghi POD riêng (ADR-022).
- Kích thước vùng được giữ trước bằng `posix_fallocate`: `/dev/shm` nhỏ là lỗi rõ lúc activate
  ("`/dev/shm too small: need <n> MB`"), không bao giờ là SIGBUS giữa lần chạy.
- Cỡ: khoảng 0,55 KB mỗi bản ghi hiện nay, khoảng 1 KB sau khi thêm kênh. 60 s × 1 kHz × 2 bộ đệm ≈
  66 MB hiện nay, ≈ 120 MB sau. Container `gazebo` trên CI chạy với `--shm-size=512m`.

**Controller** (`cobot_controller`, tên vùng `cobot_log_` + tên node):

- `on_activate`, trước khi chạy: nếu vùng cũ còn dòng chưa xuất thì xuất ra
  `<log_path>.recovered-<YYYYmmdd-HHMMSS>.csv` (cảnh báo kèm đường dẫn), xoá vùng cũ, rồi tạo vùng
  mới; tạo thất bại thì activate lỗi, nêu lý do. `log.mode: off`: không vùng nhớ, không Run CSV.
- Dịch vụ `~/dump_log` (`std_srvs/srv/Trigger`): đổi bộ đệm (chờ `swap_done()` tối đa 200 ms), nối
  các dòng từ lần xuất trước vào `log_path`; trả `<csv> <rows> rows`, kèm số dòng bị đè nếu có. Chạy
  ở luồng thường, một luồng xuất tại một thời điểm.
- Tham số ROS `log_stream_ms` (mặc định `0` = chỉ xuất theo yêu cầu): > 0 thì luồng xuất làm như
  `~/dump_log` mỗi `log_stream_ms`. Harness Gazebo truyền `200` (vẫn đọc đuôi `run.csv`) và gọi
  `~/dump_log` trước khi tắt launch.
- Xuất lần cuối khi deactivate và cleanup; vùng nhớ **không** bị xoá khi thoát, để `cobot_logdump`
  còn dùng được; lần activate sau thấy không còn gì để khôi phục thì xoá nó.
- `cobot_logdump <controller> [--out file] [--keep] [--remove]` (`src/bench/logdump_main.cpp`, không
  ROS), build bởi CMake gốc và bởi gói `cobot_ros2` (`ros2 run cobot_ros2 cobot_logdump`). Mã thoát:
  0 xong, 1 không có vùng hoặc không có gì để xuất, 2 sai tham số hoặc sai bố cục.
- Bước chuyển tiếp (PR `pipeline-recorder`, trước `blackbox-log`): node dùng `HeapStorage` hai bộ đệm
  và đổi bộ đệm, nối `run.csv` mỗi 50 ms, để harness Gazebo không đổi.

**Kênh mới trong Run CSV**, nối sau cột `hold` (cột cũ giữ thứ tự):

| Trường `LogRecord<N>` | Cột | Điền bởi | Giá trị |
| --- | --- | --- | --- |
| `qdd_ref` | `qddref1..N` | `ControlLoop` | `qdd` tham chiếu của CLIK; 0 ở dòng holding/fault/idle |
| `q_raw` | `qraw1..N` | `ControlLoop` | q đo **trước** bù độ xoắn (bằng `q` khi không bù) |
| `q_true`, `qd_true` | `qt1..N`, `qdt1..N` | host (`PlantTruth<N>`) | trạng thái khâu của plant tại thời điểm của dòng, trước decorator |
| `qdd_true` | `qddt1..N` | host | gia tốc khâu trung bình trên khoảng plant kết thúc tại `t` |
| `q_motor_true` | `qmt1..N` | host | vị trí phía động cơ của plant, đơn vị khâu |
| `tau_motor` | `taum1..N` | host | mô-men plant đã áp trên khoảng kết thúc tại `t` (sau decorator, trễ motor, trần) |
| `tau_friction` | `tauf1..N` | host | mô-men ma sát trên khoảng đó (Coulomb + nhớt của profile), ngược chiều chuyển động |
| `cmd_mode` | `cmd` | `ControlLoop` | `config`, `waypoints`; `jog_joint`, `jog_tcp`, `move_joint`, `move_tcp` dành cho giai đoạn B |

- Host truyền `PlantTruth<N>` qua `ControlLoop::step(state, out, truth)`; bench lấy từ
  `SimTruthProbe<N>` đọc `SimPlant` (thêm `SimPlant::friction_torque()`). `ControlLoop` chỉ chép, không
  hành vi nào dựa vào các trường này.
- **Căn dòng**: `taum`, `tauf`, `qddt` của dòng k mô tả khoảng plant (t_{k−1}, t_k]; `taum` của dòng
  k+1 là phản hồi của plant với `tau` của dòng k. Studio dịch đường nhiễu động cơ đi một dòng.
- Không có giá trị thật thì ghi `nan` (không bao giờ `-nan`): robot thật ghi `nan` mọi cột thật;
  Gazebo có `qt`, `qdt` (đọc thô trước decorator) và `taum` (lệnh cuối gửi Gazebo), còn `qddt`, `qmt`,
  `tauf` là `nan` (ma sát nằm trong vật lý).
- Với N = 7: 57 + 8·7 + 1 = 114 cột. `cobot_eval` (`run_columns`, `legacy_run_columns`) đọc cả dạng
  57 cột cũ lẫn 114 cột mới, từ chối dạng khác và nêu cột sai đầu tiên. `qraw` và `qmt` vượt danh sách
  của thiết kế 6.2, thêm vào để thấy phần bù độ xoắn bỏ đi và độ xoắn thật của lò xo.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Bật/tắt log bằng cờ build | Khi tắt không tốn gì | CI phải chạy gấp đôi cấu hình; log hiện trường cần build lại; test không chạy đúng bản sẽ chạy thật |
| Giữ ring + luồng ghi (ADR-008) | Đã có, đã test | I/O đĩa và định dạng text trong lúc chạy; bench phải chờ khi đĩa chậm |
| File ánh xạ bộ nhớ trên đĩa | Sống qua cả khởi động lại máy | Page cache flush ra đĩa trong lúc chạy; mòn thẻ SD của Pi 5 |
| Log nhị phân + công cụ chuyển đổi | File nhỏ, xuất nhanh | Thêm một trình đọc; xuất CSV đã nằm ngoài đường nóng nên không cần |
| Một bộ đệm, chép toàn bộ khi lấy log | Đơn giản | Chép 60 MB trong một chu kỳ 1 ms, hoặc phải dừng vòng điều khiển |
| Hoãn mỗi bản ghi một bước để `taum` cùng dòng với `tau` | Dòng dễ đọc hơn | Cần ô chờ; dòng cuối mất khi sập; mọi host phải gọi thêm hàm flush |
| Đặt thẳng `LogRecord<N>` lên vùng nhớ chung cho mọi tiến trình đọc | Một kiểu dữ liệu | Không trivially copyable; chỉ an toàn trong cùng bản build |

## Hệ quả

- Bộ nhớ: bench khoảng 12 MB cho lần chạy 12 s sau khi thêm kênh (khoảng 7 MB trước đó); controller
  khoảng 120 MB vùng nhớ chia sẻ ở `log.capacity_s: 60`, cấp phát và chạm trang trước khi chạy.
- PF-01 (p95 của `ControlLoop::step`) đo trước/sau ở PR `pipeline-recorder` và `log-channels` (bản
  ghi lớn từ khoảng 0,55 KB lên khoảng 1 KB), ghi trong mô tả PR; gate CI giữ p95 < 1 ms.
- `/dev/shm` phải đủ chỗ: Docker cần `--shm-size` (mặc định 64 MB không đủ); WSL và Pi 5 dùng tmpfs
  mặc định nên đủ, nhưng `lock_memory: true` trên Pi 5 cần giới hạn `memlock` đủ cho vùng nhớ.
- Byte Run CSV không đổi ở `pipeline-recorder` và `blackbox-log` (so sha256 với `main`); `log-channels`
  đổi hợp đồng một lần, có chủ ý, lên 114 cột (sửa `docs/conventions.md`, writer, `cobot_eval`).
- Test đóng băng đổi trong ba PR có nhãn `test-change`: `pipeline-recorder` (năm test dùng
  `ControlLoop::log()`), `blackbox-log` (giả lập harness Gazebo), `log-channels` (hợp đồng Run CSV).
- Harness Gazebo gọi `~/dump_log` trước khi tắt và dùng `cobot_logdump` khi `run.csv` thiếu dòng.
- Chạy song song `blackbox-log` và `log-channels`: PR nào vào sau thì tăng `kShmLogLayoutVersion`.
