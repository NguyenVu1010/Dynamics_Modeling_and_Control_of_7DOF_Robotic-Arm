# Quy trình và CI

Quy tắc của dự án: test viết trước và đóng băng sau khi implement; sửa một test đã có trên main
cần nhãn `test-change` và lý do trong mô tả PR; không commit nào được chạm `tests/` sau commit
`test(` cuối cùng; CI đủ 9 job xanh mới merge (CONTRIBUTING.md, ADR-016).

## 1. Job frozen-tests đếm nhầm commit của main là sửa test muộn (#5)

- **Nguyên nhân:** CI kiểm tra merge commit của PR, nên các commit vào main sau khi PR mở cũng bị
  tính là commit của PR.
- **Xử lý:** kiểm tra trên SHA đầu nhánh của PR và loại các commit thuộc nhánh gốc.

## 2. Đổi gain làm test cấu hình đã đóng băng trượt (#15)

- **Triệu chứng:** test C++ ghim giá trị `kp`, `kd` của `config/default.yaml` trượt sau khi chỉnh
  gain.
- **Xử lý:** sửa test qua PR có nhãn `test-change`, ghi rõ lý do; cấu hình kịch bản an toàn E2E-04
  được đồng bộ vì một test yêu cầu nó giống cấu hình mặc định trừ đích.
- **Bài học:** test ghim nội dung file cấu hình nên được nhóm lại, để biết ngay một thay đổi cấu
  hình chạm tới những test nào.

## 3. Đã gắn nhãn `test-change` mà job vẫn trượt

- **Nguyên nhân:** job đọc nhãn từ dữ liệu sự kiện của lần chạy; gắn nhãn sau khi CI đã chạy thì
  lần chạy đó không thấy nhãn.
- **Xử lý:** gắn nhãn khi tạo PR (`gh pr create --label test-change`), hoặc gỡ ra gắn lại để chạy
  lại.

## 4. Thêm kịch bản mới làm một test cũ trượt (#26)

- **Triệu chứng:** thêm E2E-09 (Gazebo, tag `gazebo`) làm `test_all_tag_gazebo_runs_e2e_07_without_bench`
  trượt: test giả định tag `gazebo` chỉ chọn đúng một kịch bản.
- **Xử lý:** người implement không tự sửa test mà báo tranh chấp kèm giá trị đúng; test được sửa
  trong commit `test(` (gộp vào commit test có sẵn để giữ quy tắc thứ tự), PR mang nhãn
  `test-change`.
- **Bài học:** test nên kiểm tra tính chất ("mọi kịch bản gazebo đều chạy với controller thật")
  thay vì một số lượng cố định khi số lượng có thể tăng.

## 5. Sửa test sau commit implement làm hỏng thứ tự commit

- **Nguyên nhân:** quy tắc "không commit nào chạm `tests/` sau commit `test(` cuối" bị vi phạm khi
  một file kịch bản trong `tests/` được thêm trong commit `feat(`.
- **Xử lý:** dời file đó vào commit `test(` (sửa lịch sử nhánh trước khi push), chạy
  `tools/ci/check_frozen_tests.sh` trên máy trước khi push.

## 6. Lỗi lint lặp lại trong các lần sửa test nhỏ

- **Triệu chứng:** dòng quá 100 ký tự, thứ tự import, docstring xuống dòng sai, phát hiện trên CI.
- **Xử lý:** luôn chạy `tools/wsl/lint.sh` trước khi commit, kể cả với thay đổi một dòng.

## 7. Các review bổ sung test tìm ra lỗi thật (#19–#22, #27)

- Sau MVP, một lượt review riêng cho từng module chỉ ra các khoảng trống test; viết bổ sung các
  test đó đã tìm ra lỗi NaN (file 03, mục 1). Các khoảng trống phát hiện trong review của một PR
  mà không sửa được trong PR đó (vì test đã đóng băng) được ghi lại và làm thành PR test riêng
  ngay sau (#27).
- **Bài học:** ghi lại mọi phát hiện review bị bỏ qua và xử lý nó, không để trôi.
