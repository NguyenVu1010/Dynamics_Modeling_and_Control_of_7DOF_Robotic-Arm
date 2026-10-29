# ADR-001: Ngôn ngữ: C++17 cho lõi, Python 3.12 cho đánh giá và công cụ

- Trạng thái: chấp nhận (MVP, 2026-10-29)
- Ngày: 2026-10-29
- Liên quan: thiết kế mục 1, 2 (G1, G5), 6; quyết định D1 của chủ dự án

## Bối cảnh

Bản cũ là 6 node Python ROS 1, M, C, G là biểu thức đóng sinh từ Maple. Mục tiêu mới là một
vòng điều khiển 1 kHz trên một lõi Raspberry Pi 5, không cấp phát heap trong vòng, và kiểu kích
thước cố định lúc biên dịch. Chủ dự án đã chốt viết lại bằng C++ (D1). Đích chạy sau này là
Pi OS / Ubuntu 24.04 aarch64 với GCC 12 trở lên; máy dev là WSL Ubuntu 24.04 với GCC 13.3.

## Quyết định

- Lõi `core/`, `bench/`, `ros2_node/`: **C++17**, biên dịch được bằng GCC 12+ (Pi OS) và GCC 13
  (desktop). Cờ `-Wall -Wextra -Wpedantic`, `-Werror` qua `COBOT_WERROR=ON` trong CI và
  `tools/wsl/build.sh`.
- Định dạng clang-format 18, kiểm tra tĩnh clang-tidy 18, cấu hình chép từ graph_slam.
- Không dùng tính năng C++20 (`std::span`, concepts) để giữ GCC 12; khi cần span dùng con trỏ +
  kích thước hoặc `Eigen::Ref`.
- **Python 3.12** cho `eval/`, `tools/`, launch file: type hints, `ruff` (dòng 100, py312),
  `pytest`, một `pyproject.toml` ở gốc (gói `cobot-tools`, extras `eval`, `dev`).
- Code, chú thích và `CONTRIBUTING.md` bằng tiếng Anh; ADR và dẫn xuất bằng tiếng Việt; README
  tiếng Anh có mục tóm tắt tiếng Việt.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Giữ Python, tối ưu bằng NumPy | Không viết lại từ đầu | Không đạt 1 kHz ổn định trên Pi5; không kiểm soát được cấp phát; trái D1 |
| C++20 | `std::span`, concepts, designated initializers | GCC 12 trên Pi OS hỗ trợ chưa đủ; lợi ích nhỏ cho lõi số học |
| Rust | An toàn bộ nhớ | Hệ sinh thái `ros2_control` và Eigen là C++; tăng chi phí cho đồ án |
| Python cho đánh giá bằng C++ | Một ngôn ngữ | matplotlib/pandas nhanh hơn nhiều cho báo cáo; tách đánh giá khỏi lõi là chủ ý |

## Hệ quả

Lõi build được bằng apt trên cả desktop và Pi5 không cần toolchain riêng. Hai ngôn ngữ nghĩa là
hai bộ lint và test (ctest, pytest), nên CI có job `core` và `python` riêng. Hợp đồng giữa hai
phía chỉ là file CSV (ADR-013).
