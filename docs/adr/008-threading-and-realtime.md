# ADR-008: Luồng và thời gian thực: lõi không tạo luồng, chế độ tất định

- Trạng thái: chấp nhận (MVP, 2026-10-29)
- Ngày: 2026-10-29
- Liên quan: thiết kế mục 5.4 (bước 5), 5.7; ADR-003, ADR-012, ADR-014

## Bối cảnh

Vòng điều khiển chạy ở 1 kHz dưới hai host khác nhau: `controller_manager` của `ros2_control`
(Gazebo, sau này có thể Pi5 PREEMPT_RT) và `cobot_bench`. `controller_manager` đã sở hữu luồng
thời gian thực của nó; lõi tự tạo luồng sẽ tranh với host. Ghi CSV là I/O chặn, không được nằm
trong vòng. Test và E2E-03 cần chạy lại cho kết quả giống từng byte.

## Quyết định

- **Lõi không tạo luồng.** Host sở hữu một luồng điều khiển gọi `ControlLoop::step()` và một
  luồng ghi log.
- Hai luồng trao đổi qua `pipeline::SpscRing<LogRecord<N>, kLogCapacity>` (một ghi, một đọc,
  dung lượng cố định 4 096, `push`/`pop` `noexcept`, không khoá, không cấp phát). Ring đầy thì bỏ
  bản ghi mới và tăng một bộ đếm; không bao giờ chặn vòng điều khiển. Cấu trúc bên trong ring do
  người xây dựng chọn.
- `pipeline::CsvSink` / `write_csv_row` chạy ở luồng log của host.
- **Chế độ tất định** `deterministic: true` (mặc định trong `config/default.yaml`): một luồng,
  không timer thật, bước theo `dt` cố định, seed cố định; host rút ring sau mỗi bước. Dùng trong
  test, bench và E2E.
- **Kế hoạch Pi5** (ghi trước, không làm lần này): kernel PREEMPT_RT, `isolcpus` cho lõi điều
  khiển, `SCHED_FIFO`, `mlockall`, cấp phát trước khi vào vòng. Xem ADR-014.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Lõi tự tạo luồng điều khiển và luồng log | Host đơn giản | Tranh với luồng của `controller_manager`; không tất định trong test |
| Ghi CSV trực tiếp trong `step()` | Ít code | I/O chặn trong vòng 1 kHz; trái quy tắc hot path |
| Hàng đợi có mutex | Dễ viết đúng | Khoá trong vòng điều khiển; đảo ưu tiên trên RT |
| Nhiều node ROS như bản cũ | Tách biệt | Jitter topic, không tất định; xem ADR-003 |

## Hệ quả

Cùng một `ControlLoop` chạy dưới mọi host; cách tổ chức luồng trên Pi5 do host quyết. Đổi lại,
mỗi host phải tự viết vòng rút ring, và log có thể mất bản ghi khi luồng ghi quá chậm (bộ đếm cho
biết bao nhiêu).
