#pragma once

#include <string>

// SHA-256 (FIPS 180-4) of the report's input files (feature verify): lower-case hex of the digest
// of `bytes`, any bytes (not only text).
namespace cobot::bench {

std::string sha256_hex(const std::string& bytes);

}  // namespace cobot::bench
