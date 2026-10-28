# Ma trận quán tính: thuật toán vật rắn hợp thành (CRBA)

Cài đặt: `core/dynamics/src/crba.cpp` (`mass_matrix`). Ký hiệu như `rnea.md`; mọi vector
biểu diễn trong `b`; `S(x)` là ma trận phản đối xứng với `S(x) y = x x y`.

## Ý tưởng

Cột `i` của `M` là `rnea(q, 0, e_i, 0)`: mômen khớp khi chỉ khớp `i` có gia tốc đơn vị, vận
tốc và trọng lực bằng 0. Khi đó các link `0..i-1` đứng yên, còn các link `i..N-1` chuyển động
như một vật rắn duy nhất `C_i` (vật hợp thành) quay quanh trục `z_i` qua `p_i` với gia tốc góc
`z_i`. Vì `w = 0`, không có số hạng ly tâm hay Coriolis.

## Vật hợp thành

Tích lũy từ ngọn về gốc (O(N)), quanh gốc của `b`:

```
m_C  = sum_{k>=i} m_k
h_C  = sum_{k>=i} m_k c_k                                 first moment, c_k CoM in b
I_oC = sum_{k>=i} R_k I_ck R_k^T - m_k S(c_k) S(c_k)      inertia about the origin of b
```

Dời về điểm `p = p_i` (định lý trục song song, khai triển `-S(c_k - p)^2`):

```
I_pC = I_oC + S(h_C) S(p) + S(p) S(h_C) - m_C S(p) S(p)
```

Cách này không chia cho `m_C`, nên vẫn đúng khi một link có khối lượng 0.

## Lực quán tính và các phần tử của M

Điểm `x` của `C_i` có gia tốc `z_i x (x - p_i)`, nên

```
F   = sum m_k z_i x (c_k - p_i) = z_i x (h_C - m_C p_i)
n_i = I_pC z_i                                            moment about p_i
```

Các link `0..j` (`j <= i`) truyền nguyên lực `F` và mômen; quanh `p_j`:

```
n_j  = n_i + (p_i - p_j) x F
M_ji = M_ij = z_j . n_j,       j = 0..i
```

Mỗi cặp `(i, j)` được tính một lần và ghi vào cả hai nửa, nên `M` đối xứng theo cách dựng.
Tổng chi phí O(N^2). Đường chéo `M_ii = z_i^T I_pC z_i`; với con lắc đơn
(`tests/unit/dynamics/test_analytic.cpp`) là `I_xx + m l^2`.
