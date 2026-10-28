# ADR-014: Interface Pi5: EtherCAT CiA402 CST, 1 kHz, PREEMPT_RT (chỉ yêu cầu)

- Trạng thái: chờ phần cứng
- Ngày: 2026-10-29
- Liên quan: thiết kế mục 2 (G4, không mục tiêu), 3, 5.7; D3 của chủ dự án; ADR-008, ADR-012, ADR-017

## Bối cảnh

Phạm vi lần này là mô phỏng (D3). Đích chạy thật sau này là Raspberry Pi 5 (Cortex-A76, aarch64).
Actuator theo tên mesh là ZeroErr eRob110H/eRob90H/eRob70H, giao tiếp EtherCAT CiA402, có chế độ
mô-men. Chưa có phần cứng, nên ADR này chỉ ghi yêu cầu để thiết kế hiện tại không đóng đường đi.

## Quyết định

Yêu cầu cho hiện thực sau (không làm trong MVP):

- Bus: EtherCAT, profile CiA402, chế độ **Cyclic Synchronous Torque (CST)**, chu kỳ 1 ms, đồng
  bộ Distributed Clock.
- Mỗi khớp: đọc vị trí, vận tốc, mô-men thực; ghi mô-men đích; ánh xạ đơn vị encoder → rad và
  mô-men định mức → N·m nằm trong một file YAML của phần cứng, không trong lõi.
- Hệ điều hành: kernel PREEMPT_RT, `isolcpus` cho lõi điều khiển, `SCHED_FIFO`, `mlockall`
  (ADR-008).
- Ngân sách thời gian: một `ControlLoop::step()` đầy đủ < 100 µs trên một lõi Pi5.
- Lõi chỉ dùng phụ thuộc cài được bằng apt trên Pi OS / Ubuntu 24.04 arm64 (kiểm tra bằng
  container của ADR-017).
- Hiện thực sẽ là một `hal::JointInterface<7>` mới hoặc hardware interface `ros2_control`, theo
  câu hỏi mở của ADR-012; an toàn phần cứng (STO, phanh, giới hạn dòng) bổ sung khi có datasheet.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Chế độ CSP (vị trí) | Driver tự ổn định, an toàn hơn | Bỏ qua computed torque, mục tiêu của đồ án |
| CAN / CANopen | Rẻ, đơn giản | Actuator đã chọn dùng EtherCAT; 7 khớp × 2 khung × 1 kHz khó vừa CAN 1 Mbit/s |
| Không RT (kernel thường) | Không phải dựng kernel | Jitter vài ms phá vòng 1 kHz |
| Làm ngay trong MVP | Có demo phần cứng | Chưa có phần cứng; ngoài phạm vi D3 |

## Hệ quả

Thiết kế hiện tại giữ được đường lên phần cứng: lõi không ROS, không cấp phát, phụ thuộc apt, ảnh
arm64 kiểm tra mỗi PR. Bảng yêu cầu phần cứng từ ADR-015 (bit encoder, trễ lệnh, sai số mô-men)
dùng để kiểm tra actuator thật đủ tốt hay không trước khi viết driver.
