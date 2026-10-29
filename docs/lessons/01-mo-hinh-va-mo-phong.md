# Mô hình và mô phỏng

## 1. Tham số robot của bản cũ bị thất lạc, khối lượng URDF quá nhẹ

- **Triệu chứng:** bản Python cũ đọc M, C, G từ biểu thức Maple và một file JSON có đường dẫn
  cứng ra ngoài repo; file đó không còn. Tổng khối lượng chuyển động trong URDF chỉ 6.06 kg, quá
  nhẹ so với một cánh tay 7 khớp dùng actuator eRob.
- **Tìm nguyên nhân:** đối chiếu từng link với datasheet actuator và thể tích ống nhôm.
- **Xử lý:** lập `sim/urdf/mass_budget.yaml` (mỗi số ghi nguồn), viết `tools/fix_urdf_inertia.py`
  sinh lại URDF; khối lượng chuyển động thành 17.05 kg, quán tính co giãn theo, sửa lỗi gõ ở
  `effort`. Động lực học viết lại bằng RNEA/CRBA đệ quy, không dùng biểu thức đóng (#1, #11).
- **Bài học:** mọi số vật lý phải truy được về một nguồn nằm trong repo, và được sinh bằng công
  cụ thay vì sửa tay trong URDF.

## 2. Ma sát Coulomb gần như biến mất trong SimPlant (#25)

- **Triệu chứng:** khi làm bù ma sát (#23), bật bù không cải thiện cổ tay như tính toán, như thể
  plant không có ma sát Coulomb. Test "khớp đứng yên khi mô-men dưới ngưỡng ma sát" vẫn đạt.
- **Tìm nguyên nhân:** đo công ma sát bằng cân bằng năng lượng trên cổ tay đang quay đều: công
  của mô-men trừ động năng tăng thêm chỉ bằng phần ma sát nhớt, phần Coulomb gần như bằng 0.
- **Nguyên nhân:** Coulomb được mô hình bằng `tanh(q̇ / 1e-3)` trong RK4 bước 1 ms. Độ dốc của nó
  là `c / 1e-3`, và với quán tính cổ tay khoảng 1e-4 kg·m² thì `h · dốc / I` khoảng 2400, vượt xa
  giới hạn ổn định của RK4 (khoảng 2.8). Vận tốc dao động quanh 0, mô-men ma sát đổi dấu liên tục
  và trung bình về 0. Test dưới ngưỡng vẫn đạt vì ở đó vận tốc vốn bằng 0.
- **Xử lý:** RK4 không còn tính Coulomb; sau mỗi bước, một phép chiếu dính-trượt áp xung ma sát qua
  các cột của `M⁻¹`. Lần sửa đầu xử lý từng khớp một lượt, reviewer chỉ ra khớp trượt có thể kéo
  khớp đang dính ra; bản cuối giải đồng thời cả 7 khớp (Gauss-Seidel chiếu, rồi LU chính xác trên
  tập khớp dính). Thêm test 7 khớp có ma sát ở mọi khớp (#27).
- **Bài học:** với mô hình cứng (stiff) trong tích phân hiện, test trạng thái tĩnh không đủ; cần
  test động lực học có đo năng lượng. Hàm "làm mượt" có độ dốc lớn không phải là cách xử lý ổn
  định.

## 3. URDF thiếu armature: cổ tay nhẹ hơn thực tế khoảng 1300 lần (#24)

- **Triệu chứng:** sau khi sửa ma sát, E2E-02 vẫn trượt (4.33°, 0.060 rad RMS ở khớp 7) và Monte
  Carlo đạt 0/50. Ngay cả khi bù ma sát đúng tuyệt đối, sai số vẫn 0.022 rad.
- **Tìm nguyên nhân:** sai số xác lập `e ≈ d / (M_77 k_p)`. Tính ra `M_77` khoảng 1.8e-4 kg·m², độ
  cứng chỉ khoảng 1 N·m/rad. Thử nhanh cách cộng một armature xấp xỉ vào cổ tay: sai số giảm khoảng
  100 lần.
- **Nguyên nhân:** quán tính rotor nhân bình phương tỉ số truyền (`J_in N²`, N = 100) chiếm phần
  lớn quán tính khớp của tay máy có hộp số, nhưng URDF không có khái niệm này.
- **Xử lý:** thêm mục `armature` vào `mass_budget.yaml` (eRob110I, eRob70I theo thông số; eRob90
  nội suy; tỉ số 100, đánh dấu là giả định), ghi `<cobot_armature>` vào URDF, mô hình cộng
  `diag(armature)` vào M và `armature ⊙ q̈` vào RNEA.
- **Bài học:** trước khi chỉnh gain vì "khớp quá nhạy", kiểm tra quán tính khớp có hợp lý không.
  Số liệu không lấy được từ nhà sản xuất thì ghi rõ là giả định, nằm ở đâu, và cần thay bằng gì.

## 4. Gazebo không hiểu armature: E2E-07 trượt sau khi thêm armature (#24)

- **Triệu chứng:** sau khi mô hình có armature, kịch bản Gazebo lý tưởng E2E-07 trượt.
- **Nguyên nhân:** controller dùng mô hình có armature, còn plant Gazebo đọc URDF không có nó: hai
  robot khác nhau.
- **Xử lý:** công cụ sinh thêm `cobot30_5_gazebo.urdf`, gộp armature vào quán tính link con như
  một đĩa rotor quay quanh trục khớp: cộng `a` quanh trục, `a/2` quanh hai trục vuông góc. Nếu chỉ
  cộng vào một trục, SDFormat từ chối link vì vi phạm bất đẳng thức tam giác của tensor quán tính
  (Error Code 19). Đây là xấp xỉ (đĩa cũng đi theo các khớp phía trên); giới hạn ghi trong
  docstring của công cụ.
- **Bài học:** mỗi hiệu ứng thêm vào mô hình phải có mặt trong mọi plant, và một tensor quán tính
  phải hợp lệ về vật lý chứ không chỉ về số.

## 5. Gazebo có nhiễu: tránh tính ma sát hai lần (#26)

- **Vấn đề thiết kế:** profile realistic có ma sát, Gazebo cũng tính ma sát khớp trong DART.
- **Xử lý:** ma sát của profile đi vào `<dynamics friction damping>` của URDF qua xacro arg; phần
  giả lập cảm biến/động cơ nhận bản sao profile với ma sát bằng 0. Sai lệch mô hình được tạo bằng
  cách đưa cho controller mô hình đã `perturb_model` (plant Gazebo là URDF gốc), cùng độ lớn, ngược
  dấu với bench. Tải đầu cuối không làm được theo cách này nên bị từ chối với thông báo rõ; jitter
  chu kỳ không có nghĩa trên đồng hồ mô phỏng nên chỉ cảnh báo.
- **Phụ:** damping danh định 0.003 N·m·s/rad của URDF không còn trong plant Gazebo; sai lệch nhỏ
  hơn độ phân giải mô-men, đã ghi trong ADR-011.
- **Bài học:** liệt kê mỗi hiệu ứng do ai áp (plant hay phần giả lập) trước khi nối hai môi trường.
