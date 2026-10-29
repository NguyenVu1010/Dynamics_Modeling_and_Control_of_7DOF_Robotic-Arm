# ADR-005: Mô hình từ URDF: parser tinyxml2 tự viết, gộp khớp cố định, sửa khối lượng

- Trạng thái: đề xuất, chờ bàn
- Ngày: 2026-10-29
- Liên quan: thiết kế mục 3, 5.2, 5.8, 10; D8; ADR-004, ADR-015; `docs/conventions.md`

## Bối cảnh

Tham số của bản cũ nằm trong một JSON đã thất lạc. `cobot30_5.urdf` (xuất từ SolidWorks) có đủ
khối lượng, tâm khối, tensor quán tính 9 link và giới hạn 7 khớp. D8: URDF là nguồn tham số hình
học và quán tính duy nhất cho cả bộ điều khiển và Gazebo. Nhưng **khối lượng trong URDF gốc
sai**: thể tích mesh chia khối lượng ra đúng 1 000 kg/m³ ở mọi link (mật độ mặc định của
SolidWorks), tổng phần chuyển động chỉ 6,06 kg. Chủ dự án quyết định sửa; cách sửa do người
thiết kế đề xuất.

## Quyết định

**Parser.** `model::parse_urdf_chain(urdf, base_link, tip_link)` tự viết trên tinyxml2, đọc tập
con `link`, `joint`, `inertial`, `origin`, `axis`, `limit`, `dynamics`; lỗi nêu tên phần tử.
`make_model<N>` báo lỗi khi số khớp quay khác `N`. Khớp cố định gộp lúc nạp theo
`docs/conventions.md`: cộng khối lượng, dời quán tính bằng Steiner, hợp biến đổi vào `<origin>`
của khớp quay kế tiếp. `fixed` (world → `base_link`) bị bỏ; `joint_6_2_fix` nằm giữa chuỗi nên
`link_6_2` gộp vào `link_6`.

**Sửa khối lượng** bằng `tools/fix_urdf_inertia.py`; bảng lưu ở `sim/urdf/mass_budget.yaml` kèm
nguồn từng số. URDF gốc: `sim/urdf/cobot30_5_original.urdf`; URDF đã sửa (nguồn duy nhất):
`sim/urdf/cobot30_5.urdf`.

| Link | Thành phần | Thể tích (cm³) | Khối lượng cũ (kg) | Khối lượng mới (kg) | Hệ số |
| --- | --- | --- | --- | --- | --- |
| base_link | flange nhôm | 502,3 | 0,503 | 1,356 | 2,70 |
| link_1 | eRob110H | 1104,0 | 1,106 | 3,180 | 2,87 |
| link_2 | eRob110H | 1104,0 | 1,106 | 3,180 | 2,87 |
| link_3 | eRob90H + ống Link_23 | 1519,7 | 1,512 | 4,177 | 2,76 |
| link_4 | eRob90H | 706,1 | 0,708 | 1,980 | 2,80 |
| link_5 | eRob70H + ống Link_45 | 896,0 | 0,894 | 2,469 | 2,76 |
| link_6 | eRob70H | 322,3 | 0,323 | 0,920 | 2,85 |
| link_6_2 | eRob70H | 322,3 | 0,323 | 0,920 | 2,85 |
| link_7 | đầu cuối nhôm | 84,0 | 0,084 | 0,227 | 2,69 |
| **Tổng chuyển động** | | | **6,06** | **17,05** | |

- Actuator lấy khối lượng datasheet ZeroErr eRob v2.0: eRob70H 0,92 kg, eRob90H 1,98 kg,
  eRob110H 3,18 kg. Ống nối và kết cấu là nhôm 2,70 g/cm³; thể tích ống = link trừ actuator cùng
  loại đo ở link chỉ có actuator (Link_23 = 813,6 cm³ → 2,197 kg; Link_45 = 573,7 cm³ → 1,549 kg).
- Mỗi link coi là đặc đồng chất: giữ tâm khối, nhân tensor quán tính cùng hệ số với khối lượng.
  Script đặt khối lượng bằng giá trị trong bảng và nhân quán tính theo `mới / hiện tại`, nên chạy
  lần hai không đổi gì.
- `effort` của `joint_2` và `joint_4` là 870 N·m, các khớp cùng cỡ là 87 N·m: sửa thành 87.
- `<dynamics>` chỉ giữ `damping="0.003"` và `friction="0.0"` chuẩn URDF; bỏ thuộc tính không
  chuẩn `D`, `K`, `mu_coulomb`, `mu_viscous`. Thẻ ROS 1 (`<transmission>`, `<gazebo>`) bị bỏ.

**Giới hạn đã biết:**

- Mesh CAD mang hậu tố `T` (trục rỗng); bản T nặng hơn (eRob110H-T bán lẻ ghi 4,52–4,64 kg).
  MVP dùng số bản **H** vì cùng một nguồn; khối lượng thật có thể cao hơn bảng.
- Bỏ qua chênh mật độ trong link ghép (actuator 2,80–2,88 g/cm³ so với nhôm 2,70): sai số vài
  phần trăm, nhỏ hơn nhiều sai số cũ (gấp 2,8 lần). Khối lượng actuator gán trọn cho link chứa
  mesh của nó, không tách rotor/stator sang hai link kề nhau.
- `effort` 870 → 87 là suy luận từ khớp cùng cỡ; người xây dựng đối chiếu datasheet eRob trước khi
  dùng làm `tau_max`.
- `damping` 0,003 nhỏ hơn rất nhiều `mu_viscous` 16 của bộ xuất cũ. Chọn thuộc tính chuẩn mà
  Gazebo đọc được; ma sát thật mô tả bằng profile không hoàn hảo (ADR-015).
- Giai đoạn `dynamics` thêm kiểm tra: G(q) lớn nhất trên lưới cấu hình không vượt mô-men định
  mức của actuator ở khớp đó; nếu vượt, ghi bổ sung vào ADR này và báo chủ dự án trước khi chỉnh
  gains.

**Câu hỏi mở cho chủ dự án:** parser tinyxml2 tự viết hay `liburdfdom`? Mặc định: tinyxml2.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Parser tinyxml2 tự viết (mặc định) | Một phụ thuộc nhỏ có trên apt Pi OS; kiểm soát thông báo lỗi; tập con đủ dùng | Đang chọn; phải tự test các trường hợp lỗi |
| `liburdfdom` | Parser chuẩn, đầy đủ | Kéo theo `urdfdom_headers`, console_bridge; kiểu dữ liệu riêng phải chuyển; phần lớn tính năng không cần |
| Giữ DH + JSON như bản cũ | Ít code | JSON đã mất; DH tách khỏi mô hình Gazebo, trái D8 |
| Giữ khối lượng URDF gốc | Không phải sửa | Sai gấp 2,8 lần; mọi số mô-men và gains vô nghĩa |
| Dùng khối lượng bản T | Gần CAD hơn | Chỉ có số bán lẻ cho eRob110H-T, không có nguồn nhất quán cho cả ba cỡ |
| Đo khối lượng trên robot thật | Chính xác nhất | Chưa có phần cứng |

## Hệ quả

Bộ điều khiển và Gazebo dùng cùng một file URDF đã sửa. Mọi số khối lượng truy được về một nguồn
trong `mass_budget.yaml`. Khi có phần cứng, cập nhật bảng (bản T, cân thật) là một PR đổi YAML,
sinh lại URDF và fixture KDL (ADR-004).

## Kết quả MVP (2026-10-29)

Khối lượng chuyển động sau khi sửa là 17.05 kg (bản cũ 6.06 kg). Với mô hình này, mô-men trọng trường lớn nhất bằng 0.889 lần mô-men định mức ở khớp 2. Mô-men đỉnh khi bám quỹ đạo MVP là 52 % τ_max ở kịch bản lý tưởng, và tối đa 76 % trong Monte Carlo realistic. Sweep cho thấy sai số khối lượng link tới 20 % vẫn đạt, nhưng tải đầu cuối không khai báo 1 kg thì trượt. Còn chờ bàn: số liệu eRob và vật liệu ống nhôm cần chủ dự án xác nhận theo bản vẽ thật.
