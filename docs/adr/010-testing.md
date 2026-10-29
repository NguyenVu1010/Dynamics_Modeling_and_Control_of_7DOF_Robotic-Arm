# ADR-010: Kiểm thử: các tầng test, kiểm chứng không cần Pinocchio, ngưỡng trong kịch bản

- Trạng thái: chấp nhận (MVP, 2026-10-29)
- Ngày: 2026-10-29
- Liên quan: thiết kế mục 2 (G3), 5.3, 6, 11.3; ADR-004, ADR-015, ADR-016

## Bối cảnh

Lỗi điển hình của bài toán này là Jacobian sai khung, M sai do gộp khớp cố định, và bộ điều khiển
"chạy" nhưng không đạt ngưỡng. Không có Pinocchio làm tham chiếu (ADR-004). Hội đồng và nhà
tuyển dụng cần tin được con số, nên mỗi tiêu chí G3 phải có test chạy tự động.

## Quyết định

- **Tầng test:**
  1. Unit (GTest, `tests/unit/<module>`): mỗi hàm công khai; mọi đạo hàm giải tích so với sai
     phân trung tâm (tương đối < 1e-6).
  2. Component (`tests/component/`): bài toán biết trước đáp án (con lắc 1–2 khớp, sáu bước kiểm
     chứng của ADR-004, `SimPlant` + computed torque bám tới đích).
  3. End-to-end (`tests/e2e/scenarios/*.yaml`, chạy bằng `tools/run_e2e.py`): E2E-01..08 và PF-01
     của thiết kế 11.3.
  4. Python (`tests/python/`, pytest): script URDF, cấu hình, `eval` trên CSV nhỏ tổng hợp.
- **Ngưỡng nằm trong file kịch bản**, không trong code. Kịch bản: `id, runner, config,
  imperfections, seed, thresholds, expect`.
- Bảng E2E:

| Mã | Nội dung | Ngưỡng | Chạy khi |
| --- | --- | --- | --- |
| E2E-01 | bench + SimPlant, `ideal`, quỹ đạo MVP | 2 mm, 1°, 0,01 rad | mỗi PR |
| E2E-02 | như E2E-01, `realistic`, seed cố định | 5 mm, 2°, 0,02 rad | mỗi PR |
| E2E-03 | hai lần chạy cùng seed cho CSV giống từng byte | bằng nhau | mỗi PR |
| E2E-04 | an toàn: đích vào giới hạn khớp, NaN từ HAL, HAL quá hạn | vào chế độ giữ, không NaN, mô-men trong giới hạn | mỗi PR |
| E2E-05 | Monte Carlo 50 seed, `realistic` | ≥ 95 % đạt ngưỡng E2E-02 | hằng đêm, chạy tay |
| E2E-06 | sweep độ nhạy, báo cáo + bảng yêu cầu phần cứng | báo cáo sinh được | hằng đêm, chạy tay |
| E2E-07 | Gazebo headless, controller thật, `ideal` | G3 | mỗi PR |
| E2E-08 | container arm64 dưới QEMU: build, unit, E2E-01..04 | pass | mỗi PR |
| PF-01 | thời gian một bước `ControlLoop` | p95 < 1 ms trên runner CI; 200 µs đo tay trên desktop | mỗi PR |

- Mọi test ngẫu nhiên dùng seed cố định. Test C++ định nghĩa `EIGEN_RUNTIME_NO_MALLOC` để bắt cấp
  phát trong vòng.
- Ngưỡng G3 về sai số được gate tự động qua E2E-01 và E2E-07; riêng p95 < 200 µs của G3 là số đo
  tay trên desktop, ghi vào PR, vì runner CI dùng chung không đo được thời gian ổn định.
- Quy trình viết test trước và khoá test, cùng danh sách job CI: ADR-016.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Chỉ chạy demo Gazebo rồi nhìn đồ thị | Nhanh | Không có đáp án đúng cho từng module; không tái lập |
| Catch2 thay GTest | Cú pháp gọn | ROS 2 và graph_slam dùng GTest; một khung test cho cả repo |
| Ngưỡng hard-code trong test C++ | Một chỗ | Đổi ngưỡng phải biên dịch lại; khó thấy trong review |
| G3 chỉ là gate thủ công (ý ban đầu của thiết kế mục 7) | Không cần Gazebo trong CI | Thay bằng E2E-01/07 tự động theo thiết kế 11.3 |

## Hệ quả

Mỗi tiêu chí G3 có một test tự động trên mọi PR, trừ p95 desktop. Đổi lại CI dài hơn (job
`gazebo`, `arm64`) và kịch bản phải được bảo trì khi quỹ đạo MVP đổi.
