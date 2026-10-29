# Bài học

Tổng hợp các vấn đề đã gặp từ lúc bắt đầu viết lại dự án bằng C++ (2026-10-28) và cách đã tìm
nguyên nhân, xử lý. Mỗi mục có: triệu chứng, cách tìm ra nguyên nhân, nguyên nhân, cách xử lý và
bài học rút ra, kèm PR liên quan để đọc code.

| File | Chủ đề |
| --- | --- |
| [01-mo-hinh-va-mo-phong.md](01-mo-hinh-va-mo-phong.md) | URDF, khối lượng, armature, ma sát trong SimPlant, plant Gazebo |
| [02-dieu-khien.md](02-dieu-khien.md) | Gain computed torque, bù ma sát, đồng hồ mô phỏng |
| [03-danh-gia-va-bao-cao.md](03-danh-gia-va-bao-cao.md) | Chỉ số, NaN, bảng yêu cầu phần cứng, số liệu lạc quan |
| [04-moi-truong-va-cong-cu.md](04-moi-truong-va-cong-cu.md) | WSL, worktree, colcon, ROS domain, SSH |
| [05-quy-trinh-va-ci.md](05-quy-trinh-va-ci.md) | Test đóng băng, nhãn `test-change`, CI |

## Những quy tắc rút ra (đọc trước khi sửa)

1. **Số đẹp bất thường cũng là lỗi cần giải thích.** Sai số E2E-02 ở MVP đạt, nhưng một phần vì
   plant làm mất ma sát Coulomb và mô hình thiếu armature. Mỗi lần kết quả tốt lên hoặc xấu đi
   nhiều, tìm cơ chế vật lý trước khi tin.
2. **Kiểm tra cân bằng năng lượng của mô phỏng.** Công của mô-men trừ động năng tăng thêm phải
   bằng công ma sát. Phép đo này bắt được lỗi ma sát mà các test "dính dưới ngưỡng" không bắt.
3. **Độ cứng thực tế của một khớp là `M_ii k_p`.** Sai số xác lập xấp xỉ `d / (M_ii k_p)`; khớp
   có quán tính nhỏ sẽ nhạy nhất. Thiếu một số hạng quán tính (armature) làm sai cả kết luận về
   độ nhạy.
4. **Plant và mô hình phải mô tả cùng một robot,** chỉ khác ở sai lệch cố ý. Thêm một hiệu ứng
   vào mô hình (armature) thì phải thêm vào cả hai plant (SimPlant và Gazebo).
5. **Một hiệu ứng chỉ được áp một lần.** Khi Gazebo đã tính ma sát trong engine vật lý, phần giả
   lập cảm biến/động cơ không được cộng thêm ma sát lần nữa.
6. **NaN phải nổi lên, không được bị nuốt.** `max`, `min`, so sánh với ngưỡng đều có thể bỏ qua NaN
   một cách im lặng.
7. **Kết luận từ bảng quét phải có hướng rõ ràng.** "Mức tệ nhất còn đạt" chỉ có nghĩa khi biết
   chiều nào là tệ hơn, và chênh lệch nhỏ cỡ nhiễu không được quyết định chiều.
8. **Cache build gắn với đường dẫn nguồn.** Đổi hoặc xoá thư mục nguồn thì phải dọn cache.
9. **Hai dự án ROS trên cùng máy phải khác domain,** nếu không topic sẽ trộn vào nhau mà không báo lỗi.
10. **Không sửa ngưỡng để test qua.** Mọi lần E2E trượt trong dự án đều được xử lý bằng cách tìm
    nguyên nhân (gain, mô hình, plant), không lần nào nới ngưỡng.
