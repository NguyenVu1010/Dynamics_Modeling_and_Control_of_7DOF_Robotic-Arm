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

## 4. Hộp số đàn hồi làm vòng điều khiển mất ổn định khi đo vị trí phía khâu (#48)

### Bối cảnh: hai encoder và "lò xo" trong khớp

Mỗi khớp gồm động cơ, hộp số Harmonic Drive tỉ số 100, rồi đến khâu (link). Có thể đo góc ở hai
chỗ:

- **Encoder phía động cơ**: đo góc trục động cơ θ, chia cho tỉ số truyền để ra góc quy về khâu.
- **Encoder phía khâu**: đo trực tiếp góc đầu ra q của hộp số, tức góc khâu thật.

Nếu hộp số cứng tuyệt đối thì hai số đo này bằng nhau. Thực tế bánh răng mềm (flexspline) của
Harmonic Drive là một cốc thép thành mỏng bị ép thành hình elip. Khi có mô-men truyền qua, cốc
này bị xoắn một chút, giống một **lò xo xoắn rất cứng** nối động cơ với khâu. Độ cứng không đều:
mô-men nhỏ thì mềm hơn (răng chưa ăn khớp hết), mô-men lớn thì cứng hơn. Catalog cho ba đoạn
K1, K2, K3. Lò xo này gần như không có giảm chấn: mô hình dùng 1 % giảm chấn tới hạn.

Hiệu hai encoder `θ − q` gồm:

1. **Độ xoắn lò xo** `τ / k`: phần lớn nhất, ví dụ khi đứng yên đỡ trọng lực.
2. **Sai số truyền động**: biên độ nhỏ, lặp 2 lần mỗi vòng động cơ, do hình học răng.
3. **Lost motion**: một vùng chết nhỏ khi đổi chiều.

Encoder phía động cơ không thấy độ xoắn nên khâu thật lệch khỏi chỗ bộ điều khiển "tưởng". #48 đo
được độ lệch khoảng 1e-3 rad, ước khoảng 1 mm ở đầu công cụ.

### Triệu chứng

Bật mô hình hộp số đàn hồi với độ cứng catalog, giữ hệ số mặc định ω = 80 rad/s:

- **Encoder phía khâu:** dao động tăng dần ngay từ đầu, khoảng 120–130 Hz ở cổ tay và khoảng 42 Hz
  ở khớp 4. Mô-men chạm trần, vòng điều khiển dừng giữ vì quá tốc độ sau 0,079 s.
- **Encoder phía động cơ:** ổn định, sai số cuối 1e-6 m đo ở động cơ.

Kế hoạch (thiết kế 4.7) là "nếu mất ổn định thì chỉnh hệ số mặc định một lần". Quét ω từ 80 xuống
10 rad/s trên bốn bộ độ cứng thì **mọi lần chạy với encoder phía khâu đều dừng giữ**, chỉ muộn hơn
khi ω nhỏ hơn (0,08 s ở ω = 80, 0,45 s ở ω = 10). Encoder phía động cơ ổn định ở mọi ω. Trong khi
đó, với khớp cứng, ω = 10 đã làm E2E-02 và E2E-10 trượt ngưỡng 5 mm (6,9 và 6,8 mm).

### Vì sao mất ổn định (giải thích bằng lời)

Hình dung đẩy một thùng hàng nặng bằng một cây lò xo, nhưng chỉ nhìn vào **thùng hàng** để quyết
định đẩy mạnh hay nhẹ. Thấy thùng chậm, bạn đẩy mạnh hơn. Lực chưa tới thùng ngay mà bị lò xo nén
lại trước, rồi lò xo bung ra muộn. Lúc thùng bắt đầu lao đi thì bạn đã đẩy quá tay, phải kéo lại,
và lò xo lại trả lực muộn theo chiều ngược. Mỗi lần sửa đều đến trễ nửa nhịp so với dao động, nên
thay vì hãm, nó **bơm thêm năng lượng**. Lò xo gần như không có ma sát để tiêu năng lượng đó, nên
biên độ cứ lớn dần.

Nếu nhìn vào **tay mình** (phía động cơ) thì khác: lực và phép đo nằm cùng một chỗ. Phần "giảm
chấn" của bộ điều khiển (kd nhân vận tốc) luôn ngược chiều chuyển động của chính khối được đo, nên
luôn lấy bớt năng lượng ra. Lò xo và khâu khi đó chỉ là một tải bị động treo sau động cơ, nên hệ
không thể tự dao động mạnh lên.

Trong lý thuyết điều khiển, đây là khác biệt giữa **cảm biến và cơ cấu chấp hành đặt cùng chỗ**
(collocated) và **đặt khác chỗ** (non-collocated).

### Vì sao mất ổn định (công thức)

Một khớp: quán tính động cơ Jm, quán tính khâu Jl, lò xo k, giảm chấn lò xo d, PD trên góc khâu.
Phương trình đặc trưng:

`Jm·Jl·s⁴ + d(Jm+Jl)·s³ + (k(Jm+Jl) + d·Kd)·s² + (k·Kd + d·Kp)·s + k·Kp = 0`

Hệ số của s³ chỉ đến từ giảm chấn lò xo d, không đến từ bộ điều khiển. Tiêu chuẩn Routh cho điều
kiện xấp xỉ `Kd < d(Jm+Jl)/(Jm·Jl)`. Với d bằng 1 % tới hạn, khớp 1–2 (SHG-25) cho ω ≲ 1,3 rad/s.
Mô phỏng khớp với tính toán:

- **ω = 1:** độ cứng catalog và SHG-25 cho cả 7 khớp đều ổn định.
- **ω = 2:** độ cứng catalog vẫn dừng giữ ở 10,2 s.
- **SHG-14 cho cả 7 khớp:** dự đoán phải dưới 0,6 rad/s, và trong mô phỏng ω = 1 vẫn không ổn định.

Mức hệ số đó không dùng được cho điều khiển thật. Vậy **plant không sai; đổi hệ số không cứu
được**: cấu trúc vòng điều khiển mới là vấn đề.

Bộ computed torque còn giả định động cơ và khâu gắn cứng (armature cộng thẳng vào M). Điều này
đúng ở tần số thấp nhưng sai quanh tần số cộng hưởng của lò xo.

### Với encoder phía khâu, độ xoắn lò xo có được bù tự động không?

Có, và đây là chỗ dễ nhầm. Bộ điều khiển chỉ tính **mô-men** từ sai số góc khâu
`e = q_ref − q_khâu`; driver thực hiện mô-men đó và **không quan tâm góc động cơ**. Không ai tính
"động cơ phải quay bao nhiêu". Mô-men còn làm khâu lệch thì động cơ còn bị đẩy tiếp, nên nó tự
dừng ở `θ = q_ref + g/k`, tức quay xa hơn khâu đúng bằng độ xoắn lò xo. Phần sai số do lò xo vì vậy
được bù bằng **góc quay thêm của động cơ**, sinh ra tự nhiên từ động lực học.

Ở trạng thái xác lập, mô-men vẫn chỉ là `g(q)` cộng phần chống ma sát: lò xo truyền đủ mô-men khi
nó xoắn đúng `g/k`, nên mô-men không cần "thêm phần bù". Phần bù nằm ở **vị trí** động cơ, không ở
**độ lớn** mô-men. Mô hình trong code chạy đúng như vậy: `ControlLoop` chỉ xuất `tau`, `SimPlant`
tích phân hai khối (rotor, khâu) nối qua lò xo, và góc động cơ là kết quả. Encoder phía khâu còn
thấy cả lost motion và sai số truyền động, hai thứ mà bù tĩnh theo mô hình không xử lý được.

Vậy vấn đề không nằm ở trạng thái cuối mà ở **quá trình đi tới đó**. Mô-men đặt vào rotor, khâu
chỉ cảm nhận sau khi lò xo xoắn; quanh tần số cộng hưởng, rotor và khâu dao động **ngược pha**.
Thành phần D thấy khâu đi lên nên đẩy xuống, nhưng lực đó đặt vào rotor đúng lúc rotor đang đi
xuống, nên nó bơm năng lượng vào dao động thay vì hãm. Hệ số nhỏ thì lượng bơm này nhỏ hơn giảm
chấn tự nhiên; hệ số lớn thì dao động lớn dần (giới hạn ω ≲ 1,3 rad/s ở khớp 1–2, mục công thức ở
trên).

Với encoder phía động cơ thì ngược lại. Bộ điều khiển đo `θ` và tưởng đó là góc khâu; động cơ tới
đúng `q_ref` nên nó thấy sai số bằng 0, trong khi khâu thật võng thấp hơn một góc `g/k` (khoảng
1e-3 rad, 1,6 mm ở đầu công cụ). Vì thế bù võng sửa **số đo**, không sửa mô-men:
`q̂ = q_motor − φ(g(q))`. Bộ điều khiển bám theo `q̂` nên tự đẩy động cơ đi xa thêm một góc `φ`,
giống encoder phía khâu, nhưng `φ` lấy từ mô hình độ cứng chứ không từ phép đo.

| | Encoder phía động cơ | Encoder phía khâu | Cả hai encoder |
| --- | --- | --- | --- |
| Bộ điều khiển thấy độ xoắn lò xo? | Không | Có, trực tiếp trong sai số | Có |
| Cái gì tạo góc quay thêm của động cơ? | Bù võng theo mô hình (`q̂`) | Vòng PD, tự nhiên | Vòng ngoài chậm trên encoder khâu |
| Lost motion, sai số truyền động | Không bù được | Tự bù | Tự bù (vòng ngoài) |
| Hệ số dùng được | Cao (ω = 40–80) | Rất thấp (ω ≲ 1,3 rad/s ở khớp 1–2) | Cao ở vòng trong, thấp ở vòng ngoài |
| Độ cứng thật lệch 20 % so với danh định | Còn 2,5e-4 rad (#50) | Không ảnh hưởng | Không ảnh hưởng |

### Giải pháp hiện tại và giải pháp trong thực tế

| Cách làm | Trong thực tế | Ưu | Nhược |
| --- | --- | --- | --- |
| Khép vòng vị trí bằng encoder phía động cơ (semi-closed loop) | Phổ biến nhất ở servo công nghiệp, tay máy công nghiệp | Ổn định ở hệ số cao, không cần mô hình lò xo | Khâu lệch bằng độ xoắn lò xo (khoảng 1 mm ở đây) |
| Encoder động cơ và bù độ xoắn `q ≈ θ − τ/k` | Bù độ võng theo mô hình độ cứng, dùng ở tay máy chính xác | Giữ ổn định, giảm phần lớn sai số tĩnh | Phụ thuộc độ chính xác của k và τ; không bù được dao động |
| Hai vòng: vòng vận tốc trên động cơ, vòng vị trí trên khâu với hệ số thấp (full-closed / dual-loop) | Máy CNC có thước quang, servo có encoder đầu ra | Bám đúng vị trí khâu, vẫn ổn định | Băng thông vòng ngoài bị giới hạn bởi cộng hưởng; phải chỉnh hai vòng |
| Phản hồi mô-men khớp (cảm biến mô-men) và điều khiển thụ động (passivity-based) | Tay máy DLR/KUKA LWR, Franka | Ổn định, tuân thủ va chạm tốt, bù được đàn hồi | Cần cảm biến mô-men ở mỗi khớp |
| Singular perturbation, backstepping trên mô hình hai khối | Nghiên cứu (Spong 1987, De Luca) | Dùng được góc khâu với lý thuyết đầy đủ | Cần mô hình lò xo chính xác, phức tạp |
| Lọc notch hoặc input shaping | Servo thương mại, chống rung | Rẻ, không đổi cấu trúc | Chỉ giảm kích thích, không làm hệ non-collocated ổn định ở hệ số cao |

### Trạng thái: giải pháp đã dùng cho từng cách lắp encoder

Chủ dự án chọn (2026-11-03); ADR-019 và thiết kế realism-2 mục 4.7 đã cập nhật.

| Cách lắp | Gặp ở đâu trong dự án | Giải pháp đã dùng | Kết quả |
| --- | --- | --- | --- |
| Encoder phía động cơ, khớp đàn hồi (`position_source: motor`) | Ba profile phần cứng, test vòng kín đàn hồi | Vòng vị trí trên encoder động cơ; bù độ xoắn tĩnh `q̂ = q_motor − φ(g(q))` theo độ cứng danh định (`cobot.deflection_comp`, #50); hệ số theo profile (mục 5) | Độ cứng danh định: sai số khâu 1,2e-6 rad, 0,0014 mm (không bù: 9,9e-4 rad, 1,6 mm). Lệch độ cứng 20 %: 2,5e-4 rad, 0,40 mm |
| Encoder phía khâu, khớp đàn hồi (`position_source: link`) | Chỉ để nghiên cứu và tái hiện sự cố | Không dùng trong vòng điều khiển: không có hệ số nào vừa ổn định vừa bám đủ. Bù võng tự tắt (`q_from_motor = false`) vì số đo đã chứa độ xoắn; bù thêm sẽ thành bù hai lần | Ổn định chỉ khi ω ≲ 1,3 rad/s ở khớp 1–2; ở ω = 10–80 đều dừng giữ |
| Khớp cứng (không bật `transmission`), Gazebo | Cấu hình mặc định, E2E-01..11, Gazebo | Hai encoder cho cùng một số, không có lò xo nên không cần bù; bù võng không chạy vì state không có `q_from_motor` | Như trước #48, bit-identical |
| Cả hai encoder | Chưa làm | Hướng mở rộng: vòng nhanh trên encoder động cơ, vòng tích phân chậm trên encoder khâu khử sai lệch tĩnh (độ xoắn, lost motion, sai số truyền động) | — |

Bước "chỉnh hệ số một lần" (retune) bị huỷ vì không có hệ số nào thoả mãn với encoder phía khâu.

### Bài học

- **Trước khi lên kế hoạch "chỉnh hệ số cho ổn định", làm phân tích ổn định trên mô hình đơn giản
  nhất (hai khối, Routh).** Ở đây phép tính một trang đã cho thấy giới hạn ω ≲ 1 rad/s, trước khi
  tốn công quét.
- **Vị trí cảm biến quyết định cấu trúc điều khiển, không chỉ độ chính xác.** Encoder phía khâu
  chính xác hơn về vị trí, nhưng không dùng trực tiếp được trong vòng PD tốc độ cao qua một khớp
  đàn hồi ít giảm chấn.
- **Phân biệt sai số vị trí với thiếu mô-men.** Độ xoắn lò xo không đòi thêm mô-men mà đòi động cơ
  quay thêm một góc. Encoder phía khâu tạo góc đó bằng phản hồi; encoder phía động cơ cần sửa số đo
  (`q̂`), không cần sửa luật mô-men.
- **Hệ số tăng chỉ làm sự cố đến sớm hơn hay muộn hơn chứ không làm hết** là dấu hiệu của vấn đề
  cấu trúc. Thời điểm dừng giữ tỉ lệ nghịch với ω chính là dấu hiệu đó.

## 5. Trễ cảm biến và trễ lệnh giới hạn hệ số; mỗi phần cứng một bộ hệ số (#51)

### Triệu chứng

Ba profile phần cứng (cao cấp, trung bình, giá rẻ) có trễ cảm biến 1 chu kỳ, trễ lệnh 1 chu kỳ, khớp
đàn hồi và encoder phía động cơ. Với hệ số chung ω = 80 rad/s:

- **Cao cấp và trung bình:** sau khi tới đích còn rung khoảng 60 Hz, không tắt dần (đỉnh tới đỉnh
  tới 2e-3 rad và 1,5e-2 rad), mô-men chạm mức P khoảng một nửa thời gian. Sai số cuối 0,6–2,7 mm và
  2–10 mm.
- **Giá rẻ (servo 250 Hz):** dừng giữ vì quá tốc ở 0,22–0,24 s.
- Đặt trễ về 0 thì dao động tắt dần: nguyên nhân là trễ, không phải file cấu hình.

### Vì sao

ω = 80 được chọn cho phần cứng lý tưởng: khớp cứng, trễ vòng 3 chu kỳ vẫn nằm trong biên pha. Trễ
làm lệnh mô-men luôn đến muộn so với trạng thái thật; mỗi chu kỳ trễ ở 1 kHz lấy đi khoảng
`ω_c · 1 ms` rad biên pha tại tần số cắt `ω_c`. Hệ số càng cao thì tần số cắt càng cao và phần pha
bị trễ ăn mất càng lớn. Cộng hưởng của lò xo nằm gần đó, nên biên pha còn lại không đủ để dao động
tắt. Ở bản giá rẻ, lệnh mô-men từ máy chủ chỉ cập nhật mỗi 4 ms và giữ nguyên giữa hai lần, cộng
thêm trễ, là quá chậm so với cộng hưởng của khớp ở mọi hệ số.

### Quét hệ số (3 hạt giống, quỹ đạo E2E-01)

| ω (rad/s) | Cao cấp: sai số cuối, bão hoà | Trung bình | Giá rẻ |
| --- | --- | --- | --- |
| 20 | 0,2–1,3 mm, không | 0,4–2,6 mm, không | dừng giữ mọi hạt giống |
| 30 | 0,07–0,6 mm, không | 0,2–1,2 mm, không | dừng giữ |
| 40 | 0,06–0,25 mm, không | 0,07–0,6 mm, không | dừng giữ (quá tốc) |
| 50 | 0,04–0,12 mm, không | 0,4–0,8 mm, có | dừng giữ |
| 80 | 0,6–2,7 mm, có, rung 60 Hz | 2–10 mm, có | dừng giữ (quá tốc) |

Bản giá rẻ với ω = 20 và thời gian chờ trạng thái nới lên 20 ms: hạt giống 1 vẫn dừng vì quá tốc ở
8,6 s.

### Giải pháp hiện tại và giải pháp trong thực tế

| Cách làm | Trong thực tế | Trạng thái trong dự án |
| --- | --- | --- |
| Mỗi phần cứng một bộ hệ số, chỉnh theo trễ và độ cứng của nó | Mọi robot đều được chỉnh hệ số theo phần cứng | Đã dùng: `config/premium_fr3.yaml` ω = 50, `config/mid_kr1205.yaml` ω = 40, `config/budget_xarm7.yaml` ω = 20; `default.yaml` (phần cứng lý tưởng) giữ ω = 80 |
| Vòng vị trí và dòng điện trong driver khớp ở vài kHz, máy chủ gửi điểm đặt vị trí 100–250 Hz | Cobot giá rẻ cỡ xArm, servo công nghiệp | Chưa làm, là tính năng sau; đến lúc đó bản giá rẻ chỉ được ghi nhận, không tính trượt |
| Bộ dự báo bù trễ (Smith predictor), lọc notch tại cộng hưởng | Servo thương mại | Chưa làm |
| Giảm trễ phần cứng (bus đồng bộ, đọc và ghi trong cùng chu kỳ) | Tay máy cao cấp | Ngoài phạm vi mô phỏng; trễ là tham số của profile |

Kết quả với hệ số riêng (hạt giống 1/2/3): cao cấp 0,077 / 0,117 / 0,041 mm, trung bình 0,29 /
0,58 / 0,074 mm, không dừng giữ, dao động sau đích tắt dần. Giá rẻ dừng giữ cả ba lần (hết thời gian
chờ trạng thái ở 0,33 / 0,49 / 0,10 s), ghi nhận là một phát hiện.

### Bài học

- **Hệ số tối ưu là của một phần cứng, không phải của thuật toán.** Một bộ hệ số chung cho mọi
  profile che mất ảnh hưởng của trễ; tách cấu hình bộ điều khiển theo phần cứng giống cách robot
  thật được chỉnh.
- **Trễ và cộng hưởng cộng dồn với nhau.** Từng yếu tố riêng (trễ trên khớp cứng, khớp đàn hồi
  không trễ) đều ổn định ở ω = 80; gộp lại thì không. Test ổn định phải chạy trên tổ hợp đầy đủ.
- **Khi không hệ số nào cứu được thì vấn đề là kiến trúc.** Bản giá rẻ cần vòng trong ở driver,
  không phải hệ số khác; ghi nhận nó là phát hiện thay vì nới ngưỡng.
