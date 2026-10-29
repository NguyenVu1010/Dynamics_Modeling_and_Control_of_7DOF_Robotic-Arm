# ADR-013: Log và đánh giá: CSV + Python, không rosbag

- Trạng thái: chấp nhận (MVP, 2026-10-29)
- Ngày: 2026-10-29
- Liên quan: thiết kế mục 2 (G5), 5.6, 11.2; ADR-008, ADR-015; `docs/conventions.md` (hợp đồng dữ liệu)

## Bối cảnh

Tiêu chí G3 phải tính được giống nhau cho chạy bench và chạy Gazebo. Bench không có ROS. Đánh giá
cần đồ thị và bảng, viết nhanh nhất bằng Python. Dữ liệu một lần chạy nhỏ (12 s × 1 kHz).

## Quyết định

- Vòng điều khiển ghi **Run CSV** (một file mỗi lần chạy) theo hợp đồng cột trong
  `docs/conventions.md`: `t_ns, mode, q, qd, qref, qdref, tau, p, r, pd, rd, e, step_us`;
  double in `%.17g`.
- `bench` `montecarlo`/`sweep` ghi thêm **Summary CSV** một dòng mỗi lần chạy.
- Kết quả nằm trong `results/<run-id>/`, không bao giờ trong cây nguồn (`results/` trong
  `.gitignore`). Kết quả chọn lọc cho luận văn commit vào `docs/results/`.
- `eval/cobot_eval/` (Python, không import lõi): `metrics.py` (sai số cuối, RMS bám khớp, p50/p95
  thời gian bước), `plot.py`, `report.py` (Markdown + PNG, bảng so ngưỡng, thoát mã khác 0 khi
  trượt), `sensitivity.py` (ADR-015).
- Plugin `ros2_control` ghi cùng định dạng CSV qua luồng log của host; RViz chỉ để xem.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| rosbag2 (MCAP) | Công cụ ROS sẵn, đầy đủ | Bench không có ROS; Python cần `rosbags` để đọc; nặng cho dữ liệu nhỏ |
| Định dạng nhị phân riêng | Nhỏ, nhanh | Không đọc bằng mắt được; phải viết reader; không cần ở 12 000 dòng |
| HDF5 / Parquet | Có kiểu, nén | Thêm phụ thuộc cho lõi (trái ADR-002) |
| Tính chỉ số trong C++ | Không cần Python | Lặp code vẽ và báo cáo; đánh giá nên độc lập với code được đánh giá |

## Hệ quả

Một trình đọc cho mọi nguồn chạy; E2E-03 so sánh hai CSV bằng byte. Hợp đồng cột là giao diện
giữa C++ và Python: đổi cột cần sửa `docs/conventions.md`, writer và test cả hai phía trong cùng
PR có nhãn `test-change`.
