# Điều khiển

## 1. Gain của bản Python cũ không chịu được nhiễu thực tế (#15)

- **Triệu chứng:** E2E-02 (profile realistic) trượt nặng: 42 mm, 44°, 0.489 rad RMS.
- **Tìm nguyên nhân:** sai số dồn vào khớp 5–7. Với computed torque, lực phục hồi là
  `M(q) k_p e`; cổ tay có `M_ii` nhỏ và gain cũ chỉ `ω ≈ 4.5 rad/s`, nên ma sát Coulomb 0.24 N·m và
  độ phân giải lệnh mô-men đẩy khớp lệch xa.
- **Xử lý:** quét ω (30, 60, 80, 100 rad/s), chọn tắt dần tới hạn ở `ω = 80` (`k_p = 6400`,
  `k_d = 160`) cho mọi khớp; kiểm biên pha với trễ khoảng 3 chu kỳ. Không nới ngưỡng nào.
  Bảng so sánh nằm trong `docs/derivations/control.md`.
- **Bài học về sau:** nguyên nhân sâu hơn là mô hình thiếu armature (xem file 01, mục 3). Chỉnh
  gain đúng hướng, nhưng giải thích "cổ tay quá nhạy" phải dẫn tới câu hỏi "vì sao `M_77` nhỏ thế".

## 2. Bù ma sát không cải thiện như tính toán (#23, #25)

- **Triệu chứng:** thêm feed-forward ma sát `c ⊙ tanh(q̇_ref/ε) + b ⊙ q̇_ref` nhưng bật lên thì
  sai số không giảm như tính toán.
- **Nguyên nhân:** plant khi đó không áp ma sát Coulomb ổn định (file 01, mục 2), nên phần bù trở
  thành nhiễu.
- **Xử lý:** ship #23 với bù tắt và ghi rõ lý do trong `config/default.yaml`; sửa plant ở #25 rồi
  mới bật. Bù dùng vận tốc tham chiếu (không đưa nhiễu cảm biến qua `tanh` dốc, không rung quanh
  `q̇ = 0`), giá trị danh định bằng 80 % ma sát thật vì bù thiếu an toàn hơn bù thừa. Kịch bản lý
  tưởng dùng cấu hình tắt bù, vì bù khi plant không có ma sát là tự đưa nhiễu vào.
- **Bài học:** một bộ bù chỉ kiểm chứng được khi plant mô tả đúng thứ nó bù.

## 3. Đồng hồ ROS có thể đứng trong mô phỏng (#17)

- **Triệu chứng (trong lúc làm controller Gazebo):** đồng hồ mô phỏng có thể đứng nhiều chu kỳ,
  nên nếu lấy tem thời gian từ đồng hồ ROS, bộ kiểm tra "HAL quá hạn" có thể hiểu nhầm là mất dữ
  liệu và kích hoạt chế độ giữ.
- **Xử lý:** tem trạng thái = thời điểm kích hoạt + số chu kỳ × chu kỳ điều khiển; vòng an toàn
  vẫn bắt được tem đứng yên thật (đếm số bước không có tem mới).
- **Bài học:** trong mô phỏng, đừng dùng đồng hồ tường hoặc đồng hồ ROS cho logic an toàn tính
  theo chu kỳ.
