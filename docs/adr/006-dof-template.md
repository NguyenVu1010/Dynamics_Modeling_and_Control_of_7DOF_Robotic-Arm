# ADR-006: Số bậc tự do là tham số template `N`, alias `Arm7`

- Trạng thái: chấp nhận (MVP, 2026-10-29)
- Ngày: 2026-10-29
- Liên quan: thiết kế mục 2 (G1), 4 (D6), 5.2; ADR-002; `docs/conventions.md`

## Bối cảnh

Vòng điều khiển không được cấp phát heap (G1). Ma trận Eigen kích thước động cấp phát; kích thước
cố định thì không, và trình biên dịch tối ưu tốt hơn cho 7×7. Nhưng hard-code 7 khắp nơi làm
không viết được test trên chuỗi nhỏ (1–2 khớp, có đáp án giải tích). Người thiết kế đề xuất
template, chủ dự án đồng ý (D6).

## Quyết định

- Mọi kiểu và thuật toán của lõi phụ thuộc số khớp là template theo `int N`: `Model<N>`,
  `FkState<N>`, `rnea<N>`, `SimPlant<N>`, `JointInterface<N>`, `ClikReference<N>`,
  `ComputedTorque<N>`, `ImperfectJointInterface<N>`, `ControlLoop<N>`, `LogRecord<N>`.
- Kiểu cố định trong `cobot/common/types.hpp`: `VecN<N>`, `MatN<N>`, `Mat6N<N>`; mảng là
  `std::array<T, N>`.
- Dự án dùng `constexpr int kArmDof = 7; using Arm7 = Model<kArmDof>;`.
- Hiện thực template viết trong header, hoặc trong `.cpp` với instantiation tường minh cho `N = 7`
  (và các `N` nhỏ mà test dùng).
- `parse_urdf_chain` trả về chuỗi kích thước động (chỉ lúc khởi tạo); `make_model<N>` chuyển sang
  kích thước cố định và báo lỗi khi số khớp khác `N`.
- Sau khởi tạo, kiểu kích thước động không xuất hiện trong lõi.

## Phương án đã cân nhắc

| Phương án | Điểm cộng | Vì sao không chọn |
| --- | --- | --- |
| Hard-code 7 (`Matrix<double, 7, 7>`) | Đơn giản nhất | Không test được trên chuỗi 1–2 khớp có đáp án giải tích; không dùng lại được |
| Kích thước động + cấp phát trước | Một binary cho mọi robot | Dễ cấp phát nhầm trong vòng; Eigen không tối ưu vòng lặp cố định; chạm phản mục tiêu |
| Kích thước tối đa cố định + số khớp lúc chạy | Không template | Mọi vòng lặp phải mang số khớp thật; phức tạp mà đồ án không cần |

## Hệ quả

Không cấp phát trong vòng điều khiển được đảm bảo bởi kiểu. Test đơn vị dùng được `Model<1>`,
`Model<2>`. Đổi lại, thời gian biên dịch tăng, thông báo lỗi template dài, và nhiều code nằm
trong header.
