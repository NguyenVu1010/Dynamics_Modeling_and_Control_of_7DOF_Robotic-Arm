#include "sha256.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace cobot::bench {
namespace {

using Word = std::uint32_t;

constexpr std::array<Word, 64> kRound = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

constexpr Word rotr(Word x, int n) {
  return (x >> n) | (x << (32 - n));
}

void compress(std::array<Word, 8>& h, const unsigned char* block) {
  std::array<Word, 64> w{};
  for (std::size_t i = 0; i < 16; ++i) {
    w[i] = static_cast<Word>(block[4 * i]) << 24 | static_cast<Word>(block[4 * i + 1]) << 16 |
           static_cast<Word>(block[4 * i + 2]) << 8 | static_cast<Word>(block[4 * i + 3]);
  }
  for (std::size_t i = 16; i < 64; ++i) {
    const Word s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
    const Word s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
    w[i] = w[i - 16] + s0 + w[i - 7] + s1;
  }
  std::array<Word, 8> v = h;  // a b c d e f g h
  for (std::size_t i = 0; i < 64; ++i) {
    const Word s1 = rotr(v[4], 6) ^ rotr(v[4], 11) ^ rotr(v[4], 25);
    const Word ch = (v[4] & v[5]) ^ (~v[4] & v[6]);
    const Word t1 = v[7] + s1 + ch + kRound[i] + w[i];
    const Word s0 = rotr(v[0], 2) ^ rotr(v[0], 13) ^ rotr(v[0], 22);
    const Word maj = (v[0] & v[1]) ^ (v[0] & v[2]) ^ (v[1] & v[2]);
    for (std::size_t k = 7; k > 0; --k) {
      v[k] = v[k - 1];
    }
    v[4] += t1;
    v[0] = t1 + s0 + maj;
  }
  for (std::size_t k = 0; k < 8; ++k) {
    h[k] += v[k];
  }
}

}  // namespace

std::string sha256_hex(const std::string& bytes) {
  std::array<Word, 8> h = {0x6a09e667,
                           0xbb67ae85,
                           0x3c6ef372,
                           0xa54ff53a,
                           0x510e527f,
                           0x9b05688c,
                           0x1f83d9ab,
                           0x5be0cd19};
  const auto* data = reinterpret_cast<const unsigned char*>(bytes.data());
  const std::size_t n = bytes.size();
  std::size_t done = 0;
  for (; done + 64 <= n; done += 64) {
    compress(h, data + done);
  }
  // The tail, 0x80, zeros and the bit length: one block, or two when the length does not fit.
  std::array<unsigned char, 128> tail{};
  const std::size_t rest = n - done;
  for (std::size_t i = 0; i < rest; ++i) {
    tail[i] = data[done + i];
  }
  tail[rest] = 0x80;
  const std::size_t blocks = rest < 56 ? 1 : 2;
  const std::uint64_t bits = static_cast<std::uint64_t>(n) * 8U;
  for (std::size_t i = 0; i < 8; ++i) {
    tail[blocks * 64 - 1 - i] = static_cast<unsigned char>(bits >> (8 * i));
  }
  for (std::size_t b = 0; b < blocks; ++b) {
    compress(h, tail.data() + 64 * b);
  }
  constexpr const char* kHex = "0123456789abcdef";
  std::string out;
  for (const Word word : h) {
    for (int shift = 28; shift >= 0; shift -= 4) {
      out += kHex[(word >> shift) & 0xfU];
    }
  }
  return out;
}

}  // namespace cobot::bench
