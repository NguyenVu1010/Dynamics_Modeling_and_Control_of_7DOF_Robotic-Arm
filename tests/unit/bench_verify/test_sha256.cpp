// sha256_hex (brief verify, section 5): the hash of the report's input files, lower-case hex of
// the bytes. FIPS 180-2 vectors ("", "abc", the 448-bit and 896-bit messages, one million 'a'),
// the lengths around the padding boundary (55, 56, 63, 64, 65 bytes: the length field fits in the
// last block or needs one more), and bytes that are not text (NUL, 0xff). Expected digests checked
// with coreutils sha256sum on Ubuntu 24.04.
#include <cstddef>
#include <string>

#include <gtest/gtest.h>

#include "sha256.hpp"

namespace cobot::bench {
namespace {

TEST(Sha256, FipsVectors) {
  EXPECT_EQ(sha256_hex(std::string()),
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  EXPECT_EQ(sha256_hex(std::string("abc")),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  EXPECT_EQ(sha256_hex(std::string("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq")),
            "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
  EXPECT_EQ(sha256_hex(std::string("abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmn"
                                   "hijklmnoijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu")),
            "cf5b16a778af8380036ce59e7b0492370b249b11e8f07a51afac45037afee9d1");
}

TEST(Sha256, OneMillionA) {
  EXPECT_EQ(sha256_hex(std::string(1000000, 'a')),
            "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

TEST(Sha256, LengthsAroundThePaddingBoundary) {
  const struct {
    std::size_t length;
    const char* digest;
  } cases[] = {
      {55, "d5e285683cd4efc02d021a5c62014694958901005d6f71e89e0989fac77e4072"},
      {56, "04c26261370ee7541549d16dee320c723e3fd14671e66a099afe0a377c16888e"},
      {63, "75220b47218278e656f2013bb8f0c455a25eaf01e86c64924e9d48d89776d6f2"},
      {64, "7ce100971f64e7001e8fe5a51973ecdfe1ced42befe7ee8d5fd6219506b5393c"},
      {65, "9537c5fdf120482f7d58d25e9ed583f52c02b4e304ea814db1633ad565aed7e9"},
  };
  for (const auto& c : cases) {
    EXPECT_EQ(sha256_hex(std::string(c.length, 'x')), c.digest) << c.length;
  }
}

TEST(Sha256, BytesNotText) {
  EXPECT_EQ(sha256_hex(std::string("a\0b\xff", 4)),
            "a37cc3026aae4d519e0b19c298fa913b4dccfdf0658cbccbb7deaa0226d5acdb");
  EXPECT_EQ(sha256_hex(std::string("The quick brown fox jumps over the lazy dog")),
            "d7a8fbb307d7809469ca9abcb0082e4f8d5651e46d3cdb762d02d0bf37c9e592");
}

}  // namespace
}  // namespace cobot::bench
