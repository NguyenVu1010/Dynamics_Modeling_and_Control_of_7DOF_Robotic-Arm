# Kết quả kiểm nghiệm kịch bản phần cứng (giai đoạn C)

> Công cụ sàng lọc và so sánh phần cứng, không phải chứng nhận: không thay đánh giá an toàn theo
> ISO 10218 hay ISO/TS 15066, không thay đo trên robot thật
> ([ADR-023](../../adr/023-hardware-scenario-verification.md)).

Thời điểm chạy: 2026-11-08, commit `6ae849e`, lần chạy nightly
[42504798382](https://github.com/NguyenVu1010/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/actions/runs/42504798382)
(job `verify`, runner `ubuntu-24.04` 4 lõi, bản Release). Các file trong thư mục này là đầu ra
nguyên vẹn của `cobot_bench verify`: hash sha256 của từng file đầu vào nằm trong mỗi
`report.json` (`versions.inputs`) và trùng với file trên `main` ở commit đó.

Nhiệm vụ: [`reference_move`](../../../config/tasks/reference_move.yaml), quỹ đạo của E2E-01,
không tải. Ngưỡng: 5 mm, 2°, 0,02 rad, thời gian chu kỳ 12 s. Đây là mức E2E-12, **đang chờ chủ
dự án duyệt** (câu hỏi C13).

## Kết luận

Mỗi tổ hợp: tự chỉnh hệ số trên hạt giống 1..10, rồi 100 lần chạy Monte Carlo, kết luận ba mức
theo khoảng tin cậy Wilson 95 % (mục tiêu 95 %, biên 20 %).

| Model | Thuật toán | Hệ số | E2E-16, hạt giống 1..100 | E2E-17, hạt giống 101..200 |
| --- | --- | --- | --- | --- |
| premium_fr3 | computed torque | ω = 50 (tự chỉnh) | **ĐẠT** 100/100, Wilson 0,963..1 | **ĐẠT** 100/100 |
| mid_kr1205 | computed torque | ω = 40 (tự chỉnh) | **ĐẠT** 100/100, Wilson 0,963..1 | **ĐẠT** 100/100 |
| budget_xarm7 | computed torque | ω = 20 (mặc định, không có mức đạt) | **KHÔNG ĐẠT** 0/100, Wilson 0..0,037 | **KHÔNG ĐẠT** 0/100 |
| budget_xarm7 | vòng vị trí trong driver | ω = 50 rad/s (tự chỉnh) | **SÁT NGƯỠNG** 92/100, Wilson 0,850..0,959 | **SÁT NGƯỠNG** 92/100 |

Hai bộ 100 hạt giống khác nhau cho cùng bốn kết luận (kiểm tra độ ổn định theo hạt giống, câu hỏi
mơ hồ 18 của plan). premium_fr3 và mid_kr1205 không khai báo `drive_position`, nên bảng so sánh ghi
"not listed" cho hai cặp đó.

Bảng đầy đủ: [E2E-16](e2e_16/comparison.md), [E2E-17](e2e_17/comparison.md). Báo cáo từng cặp:
[premium CT](e2e_16/premium_fr3__computed_torque/report.md),
[mid CT](e2e_16/mid_kr1205__computed_torque/report.md),
[budget CT](e2e_16/budget_xarm7__computed_torque/report.md),
[budget drive](e2e_16/budget_xarm7__drive_position/report.md).

### Giá trị xấu nhất (E2E-16, các lần chạy không dừng an toàn)

| Cặp | Sai số vị trí cuối | Sai số góc cuối | Sai số bám RMS | Thời gian chu kỳ |
| --- | --- | --- | --- | --- |
| ngưỡng / giới hạn biên 20 % | 5 / 4 mm | 2 / 1,6° | 0,02 / 0,016 rad | 12 / 9,6 s |
| premium CT | 0,19 mm | 0,040° | 4,6e-4 rad | 9,12 s |
| mid CT | 0,85 mm | 0,102° | 1,5e-3 rad | 9,14 s |
| budget drive | **7,5 mm** | 0,66° | 7,4e-3 rad | không đo được (có lần chạy không ổn định trong ngưỡng) |

Thời gian chu kỳ là thời gian từ lúc bắt đầu tới khi TCP vào và ở yên trong ngưỡng cuối quanh
đích (câu hỏi mơ hồ 11). Quỹ đạo kéo dài 10 s, nên khoảng 9,1 s nghĩa là TCP vào ngưỡng 5 mm trước
khi quỹ đạo kết thúc.

## Đọc kết quả

**premium và mid đạt chắc.** 100/100 cho cận dưới Wilson 0,963 ≥ 0,95, và mọi giá trị xấu nhất
dưới giới hạn biên. Phân tích độ nhạy (mỗi thông số giả định ở cận dưới và cận trên của khoảng,
10 hạt giống mỗi đầu) đánh dấu "có thể đổi kết luận" cho:

- premium: `sensor.sensor_delay_cycles`, `actuator.command_delay_cycles`;
- mid: `sensor.qd_filter_hz`, `sensor.sensor_delay_cycles`, `actuator.motor_time_constant_s`,
  `actuator.command_delay_cycles`, `comm.burst_cycles`.

Đây là các thông số cần đo trên phần cứng thật trước khi tin kết luận ĐẠT: trễ cảm biến và trễ
lệnh, cùng với (với mid) bộ lọc vận tốc, hằng số thời gian động cơ và các đợt mất gói.

**budget với mô-men gửi từ máy chủ không đạt, ở mọi hệ số.** Bảng quét có 14 mức ω, mức nào cũng
dừng an toàn trên cả 10 hạt giống. 100 lần chạy chính: 99 lần dừng vì `state_timeout`, 1 lần vì
`overspeed`. Tắt lần lượt từng nhóm nhiễu (20 hạt giống mỗi nhóm): chỉ khi tắt nhóm `comm` (servo
250 Hz, mất gói, đợt trễ) mới có 17/20 lần đạt; mọi nhóm khác 0/20. Gợi ý của báo cáo, theo thứ tự:
cần vòng vị trí trong driver; tăng tần số servo hoặc giảm mất gói; không có hệ số đạt trong dải
quét. Đây đúng là kết luận #51 đã thấy bằng tay, giờ công cụ tự đưa ra.

**budget với vòng vị trí trong driver gần đạt, nhưng chưa.** Vòng vị trí trong driver (#72) bỏ
hết các lần dừng an toàn (0/100 so với 100/100). 92/100 lần đạt, cận trên Wilson 0,959 ≥ 0,95
nên không phải KHÔNG ĐẠT, cận dưới 0,850 < 0,95 nên không phải ĐẠT. Sai số vị trí cuối xấu nhất
7,5 mm vượt ngưỡng 5 mm, và có lần chạy không ổn định được trong ngưỡng. Độ nhạy đánh dấu
`sensor.q_offset_std_rad`, `comm.latency_burst_prob`, `comm.burst_cycles`. Nghĩa là: lớp cobot giá
rẻ này có thể làm được nhiệm vụ nếu sai lệch gốc encoder và chất lượng kênh truyền tốt hơn giả
định, và phải đo trên phần cứng thật.

**Hai điều chủ dự án cần biết:**

- Luật "có thể đổi kết luận" so tỉ lệ đạt của mỗi đầu (10 hoặc 20 hạt giống) với tỉ lệ chính.
  Khi tỉ lệ chính sát mục tiêu (0,92 so với 0,95), một đầu đạt 19/20 hay 10/10 đã tính là vượt
  qua mục tiêu, nên luật báo nhiều thông số hơn. Với 20 hạt giống (đo trên máy, #82) budget drive
  có 29 thông số bị đánh dấu; với 10 hạt giống ở nightly chỉ còn 3. Kết quả gần ngưỡng nên đọc
  cùng số lần đạt của từng đầu trong bảng giả định của báo cáo.
- 92/100 chỉ cách ranh giới KHÔNG ĐẠT (90/100) hai lần chạy, ở cả hai bộ hạt giống.

## Thời gian chạy (runner CI 4 lõi)

| Kịch bản | Nội dung | Thời gian |
| --- | --- | --- |
| E2E-16 | 4 cặp, tự chỉnh, 100 lần chạy, độ nhạy 10 hạt giống, phân tích nguyên nhân cho cặp KHÔNG ĐẠT | 2 giờ 41 phút |
| E2E-17 | 4 cặp, tự chỉnh, 100 lần chạy, không độ nhạy | 34 phút |
| E2E-15 | premium CT nhanh (2 mức, 2 hạt giống chỉnh, 10 lần chạy), mỗi PR | 14,4 s trên máy (#81) |

Trên máy (WSL, 4 luồng, #82): E2E-16 với độ nhạy 20 hạt giống mất 7210 s; RAM tối đa khoảng
56 MB khi chạy song song.

## Tái tạo

```bash
tools/wsl/build.sh
cobot_bench verify \
  --hardware config/hardware/premium_fr3.yaml,config/hardware/mid_kr1205.yaml,config/hardware/budget_xarm7.yaml \
  --algorithm computed_torque,drive_position \
  --task config/tasks/reference_move.yaml --runs 100 --sensitivity-seeds 10 --out out/E2E-16
# E2E-17: --seed0 101 --no-sensitivity
python tools/run_e2e.py --scenario tests/e2e/scenarios/e2e_16_verify_compare.yaml \
  --bench build/src/bench/cobot_bench --out out
```

Cùng commit và cùng đầu vào cho cùng nội dung báo cáo trên mọi số luồng (`--jobs`); đường dẫn
tuyệt đối của file đầu vào trong báo cáo là của runner CI. Tab Kiểm nghiệm của `cobot_studio`
(#83) ghi đúng các file này qua cùng một hàm, nên cho cùng kết luận.
