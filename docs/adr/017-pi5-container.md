# ADR-017: Container arm64 sát Pi5: build, test dưới QEMU, image runtime trên GHCR

- Trạng thái: chấp nhận (MVP, 2026-10-29)
- Ngày: 2026-10-29
- Liên quan: thiết kế mục 3, 11.5, 11.6; ADR-001, ADR-014, ADR-016

## Bối cảnh

Đích chạy sau này là Raspberry Pi 5 (Cortex-A76, aarch64). Lỗi chỉ lộ ra trên arm64 (phụ thuộc
không có trên apt, cờ biên dịch, khác biệt số học) nên được bắt sớm, khi chưa có phần cứng và chưa
có runner arm64. Chủ dự án yêu cầu một container Ubuntu sát môi trường Pi5.

## Quyết định

- `docker/pi5.Dockerfile`, `FROM ubuntu:24.04`, nền `linux/arm64` (Ubuntu 24.04 có bản chính thức
  cho Pi5). Chỉ cài từ apt các gói lõi dùng: Eigen 3.4, tinyxml2, yaml-cpp, GTest, CMake, Ninja.
- Biên dịch với `-mcpu=cortex-a76`.
- Đa tầng: `build` (core, test, bench; chạy ctest và E2E-01..04) → `runtime` (bench, config, URDF,
  chạy không root).
- CI job `arm64` (bắt buộc): buildx + QEMU, chạy E2E-08. Job `deploy` khi push vào `main` đẩy
  image runtime lên `ghcr.io/nguyenvu1010/cobot-pi5:{main,<sha>}`.
- Thời gian đo dưới QEMU không có ý nghĩa; job chỉ kiểm tra đúng/sai. Số đo thời gian Pi5 chỉ lấy
  trên phần cứng thật (ADR-014).
- Image ROS 2 cho Pi5 không thuộc MVP. `docker/ros.Dockerfile` (amd64, Jazzy + Gazebo) chỉ phục
  vụ job `gazebo`.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Base Raspberry Pi OS (Debian) | Đúng OS mặc định của Pi | Không có image Docker chính thức; Ubuntu 24.04 cũng được hỗ trợ chính thức trên Pi5 và trùng với desktop |
| Cross-compile từ amd64 | Nhanh hơn QEMU | Phải dựng sysroot; không chạy được test |
| Runner arm64 self-hosted (Pi5 thật) | Đo được thời gian thật | Chưa có phần cứng; runner tự host chạy code PR là rủi ro |
| Không kiểm tra arm64 tới khi có phần cứng | CI nhanh | Lỗi phụ thuộc và biên dịch lộ muộn |

## Hệ quả

Mỗi PR chứng minh lõi build và pass E2E-01..04 trên arm64 chỉ với apt. Job `arm64` chậm (QEMU
mô phỏng toàn bộ). Image trên GHCR là điểm khởi đầu khi có Pi5: kéo về và chạy `cobot_bench`.
