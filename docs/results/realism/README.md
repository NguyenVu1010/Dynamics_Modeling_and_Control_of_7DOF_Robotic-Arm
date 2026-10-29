# Kết quả sau đợt tăng độ sát thực tế

Thời điểm chạy: 2026-10-29, trên desktop (WSL Ubuntu 24.04, GCC 13, bản Release). Kết quả MVP
trước đó giữ nguyên ở [docs/results/mvp](../mvp/README.md) để so sánh.

Mô hình và mô phỏng thay đổi so với MVP:

| PR | Thay đổi | Ảnh hưởng |
| --- | --- | --- |
| #23 | Bù ma sát feed-forward trên vận tốc tham chiếu (`cobot.friction_compensation`) | Giảm sai số xác lập do ma sát |
| #24 | Armature: quán tính rotor quy đổi qua hộp số `J_in N²`, thêm vào mô hình, động lực học và plant Gazebo | Quán tính khớp 7 tăng từ khoảng 1.8e-4 lên 0.23 kg·m² |
| #25 | SimPlant áp ma sát Coulomb dạng dính-trượt, giải đồng thời cho 7 khớp | Ma sát Coulomb trong bench có tác dụng thật; trước đó gần như bằng 0 ở cổ tay |
| #26 | Gazebo mặc định dùng profile realistic: ma sát vào engine vật lý, sai lệch mô hình, seed | Thêm kịch bản E2E-09 |

CSV gốc không commit; lệnh tái tạo nằm cuối trang.

## Kịch bản E2E

| ID | Nội dung | Ngưỡng | MVP | Hiện tại |
| --- | --- | --- | --- | --- |
| E2E-01 | bench, lý tưởng | 2 mm / 1° / 0.01 rad | 2.6e-7 mm / 1e-8° / 9.8e-7 rad | **ĐẠT**: 6.7e-8 mm / 9e-9° / 6.7e-7 rad ([báo cáo](e2e_01/report.md)) |
| E2E-02 | bench, realistic, seed 7 | 5 mm / 2° / 0.02 rad | 0.17 mm / 0.31° / 0.0105 rad | **ĐẠT**: 0.060 mm / 0.0075° / 1.8e-4 rad ([báo cáo](e2e_02/report.md)) |
| E2E-03 | tất định | CSV trùng từng byte | đạt | **ĐẠT** |
| E2E-04 | an toàn, đích ra ngoài giới hạn khớp | vào chế độ giữ, \|τ\| ≤ τ_max | đạt | **ĐẠT** |
| E2E-05 | Monte Carlo realistic, 50 seed | ≥ 95 % đạt | 96 % | **ĐẠT**: 100 % ([bảng](montecarlo.md)) |
| E2E-06 | sweep 12 tham số, 171 lần chạy | tạo được bảng yêu cầu phần cứng | đạt | **ĐẠT** ([bảng](hardware_requirements.md)) |
| E2E-07 | Gazebo, lý tưởng | 2 mm / 1° / 0.01 rad | 0.000 mm / 3.7e-6 rad | **ĐẠT**: 0.000 mm / 0.000° / 4.8e-6 rad ([báo cáo](e2e_07/report.md)) |
| E2E-09 | Gazebo, realistic (ma sát trong DART, sai lệch mô hình, nhiễu cảm biến và động cơ) | 5 mm / 2° / 0.02 rad | chưa có | **ĐẠT**: 0.071 mm / 0.014° / 2.3e-4 rad ([báo cáo](e2e_09/report.md)) |
| E2E-08 | image arm64 (`-mcpu=cortex-a76`, QEMU) | ctest và E2E-01..04 | đạt | **ĐẠT** trên CI (job `arm64`) |
| PF-01 | thời gian một bước, desktop, 100 000 bước | p95 < 200 µs | p95 4.9 µs | **ĐẠT**: p50 4.8 µs, p95 5.3 µs, max 214 µs ([số liệu](timing_desktop.txt)) |

Bù ma sát và armature không làm chậm vòng điều khiển đáng kể (p95 tăng khoảng 0.4 µs).

## Vì sao sai số giảm khoảng 60 lần

Sai số xác lập khớp `i` dưới nhiễu mô-men `d` là `e_i ≈ d_i / (M_ii k_p)`
(docs/derivations/control.md). Ở MVP, URDF thiếu armature nên `M_77` của cổ tay chỉ khoảng
1.8e-4 kg·m², và độ cứng thực tế `M_77 k_p` chỉ khoảng 1 N·m/rad. Với robot thật, rotor nhân
`N²` (N = 100) chiếm phần lớn quán tính khớp. Khi thêm vào, cổ tay cứng hơn khoảng 1300 lần.
Ma sát Coulomb, độ phân giải lệnh mô-men và sai số hệ số mô-men vì thế gây sai số nhỏ hơn nhiều.

Lưu ý: số MVP về ma sát là lạc quan. Plant cũ tính Coulomb bằng `tanh(q̇/1e-3)` trong RK4 bước
1 ms, không ổn định với quán tính nhỏ, nên ma sát gần như triệt tiêu
(docs/derivations/sim_plant_friction.md). Số hiện tại có ma sát Coulomb tác dụng thật.

Giá trị armature vẫn là **giả định**: `J_in` lấy theo thông số eRob110I và eRob70I, eRob90 nội
suy, tỉ số truyền giả định 100 (`sim/urdf/mass_budget.yaml`, ADR-005). Cần thay bằng datasheet
eRob dòng H của robot thật trước khi dùng các số trên để chọn phần cứng.

## Monte Carlo (E2E-05)

50/50 seed đạt. Sai số khớp RMS có p95 là 2.7e-4 rad và lớn nhất là 4.2e-4 rad, nhỏ hơn
ngưỡng khoảng 50 lần. Mô-men đỉnh tăng lên tối đa 89 % `τ_max` (MVP: 76 %), vì armature thêm
mô-men tăng tốc cho chính rotor. Không lần chạy nào bão hoà, nhưng biên mô-men giờ hẹp hơn: quỹ
đạo nhanh hơn MVP cần kiểm tra lại `peak_tau_ratio`.

## Yêu cầu phần cứng (E2E-06)

Mỗi dòng trong [hardware_requirements.md](hardware_requirements.md) giữ nguyên profile
realistic và chỉ quét một tham số. Kết quả cho Pi5 và driver:

- Encoder 10 bit (mức thấp nhất được quét) vẫn đạt; nhiễu đo góc tới 1e-3 rad vẫn đạt.
- Trễ cảm biến vẫn là giới hạn chính: ≤ 4 chu kỳ (4 ms ở 1 kHz), 8 chu kỳ thì trượt. Trễ lệnh
  tới 8 chu kỳ giờ vẫn đạt.
- Sai số hệ số mô-men phải ≤ 20 % (MVP: 10 %); 30 % thì trượt ở sai số vị trí cuối.
- Ma sát Coulomb tới 8 N·m và độ phân giải lệnh mô-men tới 2 N·m vẫn đạt. Hai yếu tố này
  không còn là yếu tố nhạy nhất như ở MVP, xem [tornado.png](sweep/tornado.png).
- Tải đầu cuối không khai báo tới 3 kg vẫn đạt (MVP: 0.5 kg).
- Jitter chu kỳ tới 300 µs, hằng số thời gian động cơ tới 10 ms, sai số khối lượng link tới
  20 % vẫn đạt.

Ngưỡng quét thô (mỗi tham số chỉ vài mức), nên "mức tốt nhất còn đạt" là cận an toàn, không
phải giới hạn chính xác. Khi mọi mức của một tham số đều đạt, bảng chọn hướng theo sai số; trước
#28 nó luôn đi theo chiều tăng, nên `encoder_bits` hiện 20 thay vì 10.

## Cấu hình môi trường

Mọi thông số môi trường nằm trong file cấu hình, không cần sửa code:

- `config/imperfections/*.yaml` (`ideal`, `realistic`, `pessimistic`): nhiễu và độ phân giải
  cảm biến, trễ, sai số và ripple động cơ, ma sát, sai lệch mô hình, jitter, seed.
- `config/default.yaml`: gain, `friction_compensation` (bật/tắt, giá trị danh định).
- `sim/urdf/mass_budget.yaml`: khối lượng, armature; chạy `tools/fix_urdf_inertia.py` để sinh
  lại URDF.
- Gazebo: `ros2 launch cobot_sim sim.launch.py controller:=cobot_controller imperfections_path:=<file> seed:=<n>`
  (mặc định là `realistic.yaml`).

## Tái tạo

```bash
tools/wsl/build.sh
python tools/run_e2e.py --all --tag pr      --bench ~/cobot-build/main/bench/cobot_bench --out out/pr
python tools/run_e2e.py --all --tag nightly --bench ~/cobot-build/main/bench/cobot_bench --out out/nightly
~/cobot-build/main/bench/cobot_bench timing --config config/default.yaml --steps 100000
tools/wsl/sim.sh build && source ~/cobot-build/colcon/install/setup.bash
python tools/run_e2e.py --all --tag gazebo --out out/gazebo
```
