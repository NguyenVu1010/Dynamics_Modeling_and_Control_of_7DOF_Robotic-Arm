# ADR-016: Quy trình test trước, test bị khoá, CI/CD và bảo vệ `main`

- Trạng thái: chấp nhận (MVP, 2026-10-29)
- Ngày: 2026-10-29
- Liên quan: thiết kế mục 6, 11.3, 11.4, 11.5; ADR-010, ADR-017; `CONTRIBUTING.md`

## Bối cảnh

Chủ dự án yêu cầu: phát triển song song nhiều tính năng; người định nghĩa test tách khỏi người
hiện thực; không sửa test sau khi đã hiện thực; chỉ push khi test pass; chỉ merge PR khi CI xanh;
mỗi PR một tính năng, không dài. Nếu không ép bằng công cụ, cách dễ nhất để "pass" là nới test.

## Quyết định

**Quy trình một tính năng** (nhánh `feat/<tính-năng>`):

1. Commit test trước, `test(<phạm vi>): ...`, viết từ thiết kế và header interface. Test phải
   biên dịch được và thất bại vì thiếu hiện thực.
2. Commit hiện thực (`feat(...)`, `fix(...)`) chỉ sửa ngoài `tests/`. Build, ctest, pytest, lint
   cục bộ pass mới được push.
3. Thấy test sai so với thiết kế thì dừng và báo; một PR riêng có nhãn `test-change` sửa test.
   Không bao giờ nới test hoặc ngưỡng để cho pass; ngưỡng G3 và thiết kế 11.3 chỉ đổi khi chủ dự án
   đồng ý.
4. PR ≤ 400 dòng code không phải test và ≤ 400 dòng test (di chuyển thuần, xoá, mesh nhị phân và
   văn bản docs không tính).
5. Merge: squash, chỉ khi mọi check bắt buộc xanh; nhánh tự xoá sau merge.

**Khoá test bằng CI** (`tools/ci/check_frozen_tests.sh`, job `frozen-tests`): (1) file dưới
`tests/` đã có trên `main` không được sửa hay xoá trong PR, trừ khi PR có nhãn `test-change` kèm
lý do; (2) trong PR, sau commit `test(` cuối cùng không commit nào được đụng `tests/`.

**Bảo vệ `main`:** bắt buộc PR, bắt buộc các check dưới đây xanh trên commit đầu của PR, không bắt
nhánh cập nhật với `main` (CI chạy lại trên `main` sau mỗi merge, `main` đỏ thì sửa trước mọi merge
khác), áp dụng cả cho admin, cấm push thẳng và force-push.

| Job | Nội dung | Bắt buộc |
| --- | --- | --- |
| `lint` | clang-format 18, ruff check + format, clang-tidy 18 trên file C++ thay đổi | có |
| `frozen-tests` | hai luật khoá test ở trên | có |
| `core` | build Release `-Werror`, ctest | có |
| `sanitizers` | build Debug ASan + UBSan, ctest | có |
| `coverage` | gcovr trên `core/`, gate ≥ 80 % dòng, báo cáo artifact | có |
| `python` | pytest | có |
| `e2e` | E2E-01..04, PF-01, báo cáo artifact | có |
| `gazebo` | image ROS 2 Jazzy + Gazebo (amd64): colcon build, E2E-07 | có |
| `arm64` | `docker/pi5.Dockerfile` qua buildx + QEMU: E2E-08 (ADR-017) | có |
| `nightly` | lịch: E2E-05, E2E-06, báo cáo artifact | không |
| `deploy` | push vào `main`: image arm64 runtime lên `ghcr.io/nguyenvu1010/cobot-pi5:{main,<sha>}` | không |

Job chưa có gì để chạy (ví dụ `e2e` trước khi có bench) vẫn tồn tại và pass rỗng, để danh sách
check bắt buộc cố định từ đầu. Đây thay mục 6 của thiết kế (ba job, không ROS/Gazebo).

**Lịch sử:** commit chỉ mang tác giả là chủ dự án, không có dòng đồng tác giả tự động; mô tả PR,
issue, comment không có chữ ký công cụ; file riêng của công cụ trên máy dev bỏ qua bằng
`.git/info/exclude`.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Khoá test chỉ bằng quy ước | Không cần script | Không kiểm chứng được; dễ nới test khi gấp |
| CODEOWNERS cho `tests/` | Tính năng có sẵn của GitHub | Một người duy trì; không phân biệt test cũ với test mới của chính PR |
| Merge commit hoặc rebase merge | Giữ từng commit | Lịch sử `main` dài; squash cho một commit mỗi tính năng |
| Thêm job theo nhu cầu | CI ngắn lúc đầu | Danh sách check bắt buộc phải sửa mỗi lần, dễ quên |
| Chạy Gazebo chỉ bằng tay | Runner nhẹ | E2E-07 không được gate; demo hỏng không ai thấy |

## Hệ quả

Mỗi tính năng có bằng chứng test viết trước hiện thực trong lịch sử PR. CI dài hơn (Gazebo và
QEMU mất nhiều phút) và một lỗi trong test cần thêm một PR `test-change` riêng.
