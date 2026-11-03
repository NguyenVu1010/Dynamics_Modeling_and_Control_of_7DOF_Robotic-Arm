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

## 4. Kết luận đạt hay không từ mô phỏng chỉ có giá trị khi có giả định, biên và độ tin cậy (#51)

- **Bối cảnh:** tính năng `profiles` chạy cùng một quỹ đạo trên ba phần cứng mốc và cho ra kết luận
  kiểu "phần cứng này có làm được không". Ba trường hợp trong cùng lần làm cho thấy kết luận đó dễ
  sai thế nào.
- **Hệ số chưa chỉnh cho phần cứng:** với ω = 80 chung, bản cao cấp trượt (0,6–2,7 mm, rung 60 Hz);
  với ω = 50 nó đạt 0,04–0,12 mm. Kết luận "không đạt" lúc đầu là về hệ số, không phải về phần cứng.
- **Đạt sát ngưỡng:** bản trung bình, hạt giống 3, biên độ dao động chỉ giảm từ 1,457e-4 xuống
  1,438e-4 rad. Test qua, nhưng không nói gì về lần chạy thứ tư. Ba hạt giống đều đạt chỉ cho cận
  dưới tỉ lệ đạt (Wilson 95 %) là 0,44; cần khoảng 100 lần chạy đều đạt mới vượt 0,95.
- **Giả định trong mô hình:** một phần thông số của profile là giả định (`source: assumed`). Kết
  luận chỉ đúng tới mức các giả định đúng, và người đọc kết luận phải thấy được danh sách đó.
- **Kiến trúc thiếu:** bản giá rẻ không ổn định ở mọi hệ số vì mô phỏng gửi mô-men từ máy chủ ở
  250 Hz, trong khi robot thật lớp này chạy vòng vị trí trong driver. Kết luận "không đạt" khi đó
  phản ánh mô phỏng thiếu một tầng, không phải phần cứng kém.
- **Xử lý:** dự án định hướng thành công cụ kiểm nghiệm kịch bản phần cứng (ADR-023, thiết kế
  `cobot_studio` mục 10): tự chỉnh hệ số trước khi kết luận, Monte Carlo 100 lần với khoảng tin cậy,
  kết luận ba mức (ĐẠT, KHÔNG ĐẠT, SÁT NGƯỠNG, CẦN ĐO THẬT) có biên 20 %, báo cáo kèm danh sách giả
  định và độ nhạy theo từng giả định, và làm vòng điều khiển trong driver trước kết luận đầu tiên.
- **Bài học:** một chữ "đạt" từ mô phỏng phải đi cùng bốn thứ: hệ số đã chỉnh cho đúng phần cứng,
  biên so với ngưỡng, độ tin cậy thống kê, và danh sách giả định. Thiếu một trong bốn thì đó là ý
  kiến, không phải kết luận. Và trước khi kết luận phần cứng kém, kiểm tra mô phỏng có đủ các tầng
  điều khiển mà phần cứng thật có không.
