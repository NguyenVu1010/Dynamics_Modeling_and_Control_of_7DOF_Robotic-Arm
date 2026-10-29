# Kết quả MVP

> Đây là mốc so sánh. Kết quả hiện tại nằm ở [docs/results/realism](../realism/README.md).
> Các số liên quan tới ma sát ở trang này là lạc quan, vì plant lúc đó làm mất ma sát Coulomb
> do lỗi số, và URDF chưa có armature.

Thời điểm chạy: 2026-10-29, trên desktop (WSL Ubuntu 24.04, GCC 13, bản Release). Gain lấy theo
`config/default.yaml`; lý do chỉnh gain ghi trong `docs/derivations/control.md`. CSV gốc không
commit, vì có thể tạo lại bằng các lệnh ở cuối trang.

## Kịch bản E2E

| ID | Nội dung | Ngưỡng | Kết quả |
| --- | --- | --- | --- |
| E2E-01 | bench, lý tưởng | 2 mm / 1° / 0.01 rad | **ĐẠT**: 2.6e-7 mm / 1e-8° / 9.8e-7 rad ([báo cáo](e2e_01/report.md)) |
| E2E-02 | bench, realistic, seed 7 | 5 mm / 2° / 0.02 rad | **ĐẠT**: 0.17 mm / 0.31° / 0.0105 rad ([báo cáo](e2e_02/report.md)) |
| E2E-03 | tất định | CSV trùng từng byte | **ĐẠT** |
| E2E-04 | an toàn, đích ra ngoài giới hạn khớp | vào chế độ giữ, \|τ\| ≤ τ_max | **ĐẠT** |
| E2E-05 | Monte Carlo realistic, 50 seed | ≥ 95 % đạt ngưỡng E2E-02 | **ĐẠT**: 96 % (48/50) ([bảng](montecarlo.md)) |
| E2E-06 | sweep 12 tham số, 171 lần chạy | tạo được bảng yêu cầu phần cứng | **ĐẠT** ([bảng](hardware_requirements.md)) |
| E2E-07 | Gazebo Harmonic, `gz_ros2_control` | 2 mm / 1° / 0.01 rad | **ĐẠT**: 0.000 mm / 0.000° / 3.7e-6 rad ([báo cáo](e2e_07/report.md)) |
| E2E-08 | image arm64 (`-mcpu=cortex-a76`, QEMU) | ctest và E2E-01..04 | **ĐẠT** trên CI (job `arm64`) |
| PF-01 | thời gian một bước, desktop, 100 000 bước | p95 < 200 µs | **ĐẠT**: p50 4.7 µs, p95 4.9 µs, max 202 µs ([số liệu](timing_desktop.txt)) |

## Monte Carlo (E2E-05)

48 trên 50 seed đạt ngưỡng. Với sai số khớp RMS, p95 là 0.018 rad và lớn nhất là 0.022 rad.
Hai seed trượt đều trượt ở chỉ số RMS khớp, sai số vị trí cuối vẫn dưới 0.8 mm. Mô-men đỉnh
dùng tối đa 76 % `τ_max`, và không lần chạy nào bão hoà.

## Yêu cầu phần cứng (E2E-06)

Mỗi dòng trong [hardware_requirements.md](hardware_requirements.md) giữ nguyên các tham số
realistic và chỉ quét một tham số. Kết quả cho Pi5 và driver:

- Encoder từ 12 bit trở lên là đủ; nhiễu đo góc tới 1e-3 rad vẫn đạt.
- Trễ cảm biến và trễ lệnh mỗi loại phải ≤ 4 chu kỳ (4 ms ở 1 kHz); 8 chu kỳ là trượt.
- Jitter chu kỳ tới 300 µs (độ lệch chuẩn) vẫn đạt, nên yêu cầu về jitter trên Pi5 không gắt; PREEMPT_RT (ADR-014) vẫn giữ để trễ đuôi có giới hạn.
- Sai số hệ số mô-men phải ≤ 10 %. Ripple tới 10 % và hằng số thời gian động cơ tới 10 ms vẫn đạt.
- Ma sát Coulomb và độ phân giải lệnh mô-men là hai yếu tố nhạy nhất, xem
  [tornado.png](sweep/tornado.png). Profile realistic (0.24–1.7 Nm; 0.012–0.087 Nm) vẫn đạt.
  Mức quét kế tiếp (1 Nm cho mọi khớp; 0.1 Nm) thì trượt, vì độ cứng thực tế `M_ii k_p` của cổ
  tay nhỏ. Khi lên phần cứng cần nhận dạng ma sát để bù feedforward, hoặc thêm khâu tích phân.
- Tải đầu cuối không khai báo trong mô hình: 0.5 kg đạt, 1 kg trượt ở RMS khớp.

Ngưỡng quét thô (mỗi tham số chỉ vài mức), nên "mức tốt nhất còn đạt" là cận an toàn, không
phải giới hạn chính xác.

## Tái tạo

```bash
tools/wsl/build.sh
python tools/run_e2e.py --all --tag pr      --bench ~/cobot-build/main/bench/cobot_bench --out out/pr
python tools/run_e2e.py --all --tag nightly --bench ~/cobot-build/main/bench/cobot_bench --out out/nightly
~/cobot-build/main/bench/cobot_bench timing --config config/default.yaml --steps 100000
tools/wsl/sim.sh build && source ~/cobot-build/colcon/install/setup.bash
python tools/run_e2e.py --all --tag gazebo --out out/gazebo
```
