# ADR-004: Tự viết động lực học: RNEA 3D + CRBA, sáu bước kiểm chứng, fixture KDL chỉ trong test

- Trạng thái: đề xuất, chờ bàn
- Ngày: 2026-10-29
- Liên quan: thiết kế mục 5.3; D2 của chủ dự án; ADR-005, ADR-010; `docs/derivations/rnea.md`, `crba.md`

## Bối cảnh

Chủ dự án đã chốt tự viết động lực học, không Pinocchio, không KDL trong lõi (D2). Bản cũ dùng
biểu thức đóng từ Maple (riêng M là 1 292 dòng), không đọc được và không đổi được khi sửa khối
lượng. Thuật toán đệ quy cho phép đổi mô hình bằng URDF và đủ nhanh: mục tiêu < 20 µs mỗi lần
gọi RNEA hoặc CRBA trên desktop với N = 7. Thiếu thư viện tham chiếu thì cần cách kiểm chứng độc
lập, nếu không "Jacobian hoặc M, C, G không có test đối chiếu độc lập" là phản mục tiêu.

## Quyết định

- **RNEA** (Newton–Euler đệ quy, vector 3D kiểu Craig/Siciliano): `rnea(model, q, qd, qdd, g)`.
  O(N).
- **CRBA** cho `M(q)`, O(N²). `G(q) = rnea(q, 0, 0, g)`; `C(q, q̇) q̇ = rnea(q, q̇, 0, 0)`.
  Ma trận `C` đầy đủ chỉ tính trong test (ký hiệu Christoffel từ sai phân số của M).
- **Động lực học thuận** `q̈ = M⁻¹(τ − C q̇ − G − D q̇)` bằng LDLT 7×7 (`forward_dynamics`), dùng
  trong `SimPlant` (RK4, 1 ms, chặn khớp cứng: dừng và triệt vận tốc).
- **Sáu bước kiểm chứng bắt buộc** trong `tests/`:
  1. Jacobian so với sai phân trung tâm của FK, sai số tương đối < 1e-6.
  2. `M` đối xứng (‖M − Mᵀ‖ < 1e-9) và xác định dương tại 1 000 cấu hình ngẫu nhiên seed cố
     định trong giới hạn khớp.
  3. `rnea(q, q̇, q̈) = M q̈ + C q̇ + G` (< 1e-9 tương đối).
  4. `Ṁ − 2C` phản đối xứng.
  5. `SimPlant` không ma sát, không trọng lực, τ = 0, 10 s: biến thiên động năng tương đối < 1e-4.
  6. Đối chiếu `tests/data/reference_dynamics.csv` (100 bộ `q, q̇, q̈ → τ, M, G`), sinh **một
     lần** bằng KDL qua `tools/gen_reference_dynamics.cpp` (chỉ build với
     `-DCOBOT_WITH_KDL_REF=ON`, tìm `orocos-kdl` trong `/opt/ros/jazzy`). Fixture được commit;
     test mặc định chỉ đọc file.
- Nếu tìm lại được `robot_parameters.json` của bản cũ, thêm fixture thứ hai từ biểu thức Maple,
  sinh một lần bằng Python.
- Mục tiêu thời gian: < 20 µs mỗi lần gọi trên desktop; < 100 µs cho một `ControlLoop::step()`
  đầy đủ trên Pi5.

**Câu hỏi mở cho chủ dự án.** (a) Viết RNEA/CRBA bằng vector 3D hay đại số không gian 6D
(Featherstone)? Mặc định trong lúc chờ: 3D. (b) Có cho KDL vào chuỗi kiểm chứng (bước 6) không?
Mặc định: có, nhưng chỉ trong công cụ sinh fixture, không link vào test hay lõi.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| (a) Vector 3D (mặc định) | Theo được sách Craig/Siciliano, dễ đọc cho hội đồng | Đang chọn; chậm hơn 6D không đáng kể ở N = 7 |
| (a) Đại số không gian 6D (Featherstone) | Gọn, cùng ngôn ngữ với ABA, chuẩn của thư viện hiện đại | Khó đọc với người quen ký hiệu 3D; lợi tốc độ không cần ở N = 7 |
| (b) KDL chỉ sinh fixture một lần (mặc định) | Đối chiếu độc lập, test không cần ROS | Đang chọn; fixture phải sinh lại khi URDF đổi |
| (b) KDL link vào test, so trực tiếp | Không có file fixture phải bảo trì | Job `core` phải có ROS/KDL; trái ADR-003 |
| (b) Không dùng KDL, chỉ năm bước tính chất | Không phụ thuộc gì | Năm bước đều tự nhất quán; một lỗi khung chung cho FK và RNEA vẫn lọt |
| Pinocchio trong lõi | Nhanh, đã kiểm chứng | Trái D2 |
| Giữ biểu thức Maple | Đã có | Không đọc được, không đổi theo URDF, tham số đã thất lạc |

## Hệ quả

Mỗi hàm động lực học có test tính chất và test đối chiếu. Fixture KDL phải sinh lại (PR nhãn
`test-change`) mỗi khi URDF đã sửa thay đổi. Thời gian từng hàm đo trong bench ở giai đoạn
`dynamics`; nếu vượt 20 µs, xem lại cách cài đặt trước khi chuyển sang 6D.

## Kết quả MVP (2026-10-29)

RNEA và CRBA tự viết đã qua đủ sáu bước kiểm chứng trong test: sai phân hữu hạn Jacobian, M đối xứng và xác định dương, Ṁ − 2C phản đối xứng, bảo toàn năng lượng, và so với fixture KDL (KDL chỉ có trong test). Một bước điều khiển đầy đủ (CLIK, computed torque, RNEA/CRBA) mất p95 4.9 µs trên desktop, nên không cần Pinocchio để đạt hiệu năng. Vẫn giữ trạng thái chờ bàn để chủ dự án quyết có giữ fixture KDL lâu dài hay không.
