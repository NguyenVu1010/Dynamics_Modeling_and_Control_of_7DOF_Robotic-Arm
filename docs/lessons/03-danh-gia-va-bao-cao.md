# Đánh giá và báo cáo

## 1. NaN ở một khớp bị nuốt khi tổng hợp chỉ số (#22)

- **Triệu chứng:** khi review bổ sung test, phát hiện một lần chạy có sai số khớp NaN vẫn có thể
  được báo là đạt.
- **Nguyên nhân:** chỉ số RMS khớp lấy theo cách bỏ qua NaN ở một khớp; phía C++, `maxCoeff()` của
  Eigen mặc định không lan truyền NaN; tỉ số mô-men đỉnh cũng vậy.
- **Xử lý:** Python lấy `max` theo cách giữ NaN; C++ dùng `maxCoeff<Eigen::PropagateNaN>()` và giữ
  NaN "dính" cho `peak_tau_ratio`; `metrics.within` coi giá trị không hữu hạn là trượt. Thêm test
  NaN cho từng chỉ số.
- **Bài học:** mọi phép gộp (max, min, mean) và mọi so sánh ngưỡng phải có test với NaN và vô cực.

## 2. Bảng yêu cầu phần cứng đọc sai hướng (#28)

- **Triệu chứng:** sau đợt tăng độ thực tế, bảng ghi "encoder tệ nhất còn đạt: 20 bit", trong khi
  10 bit cũng đạt.
- **Nguyên nhân:** hướng duyệt các mức được suy từ mẫu đạt/trượt: nếu mức nhỏ nhất trượt và mức
  lớn nhất đạt thì đảo chiều. Khi mọi mức đều đạt, mẫu này không nói gì, và code luôn đi chiều
  tăng.
- **Xử lý:** khi hai đầu cùng kết quả, chọn đầu có sai số lớn hơn (tổng tỉ số chỉ số/ngưỡng) là
  đầu tệ. Lần chạy lại cho thấy hằng số thời gian động cơ chỉ làm sai số đổi khoảng 1e-9, đủ để lật
  chiều do nhiễu; thêm biên 5 %: chênh lệch nhỏ hơn thì giữ chiều tăng.
- **Bài học:** một bảng kết luận cần test cho cả trường hợp "mọi mức đạt" và "mọi mức trượt", và
  một quyết định dựa trên so sánh số phải có ngưỡng chống nhiễu.

## 3. Báo cáo MVP lạc quan về ma sát (#29)

- **Vấn đề:** sau khi tìm ra lỗi ma sát (file 01, mục 2) và armature (file 01, mục 3), các số MVP
  liên quan tới ma sát và độ nhạy không còn đúng.
- **Xử lý:** giữ nguyên `docs/results/mvp` làm mốc so sánh, thêm ghi chú ở đầu trang; kết quả mới
  ở `docs/results/realism` với bảng MVP / hiện tại cạnh nhau. Mọi ADR liên quan có mục "Cập nhật".
- **Bài học:** không ghi đè kết quả cũ khi kết luận thay đổi; để lại cả hai và giải thích vì sao
  khác.
