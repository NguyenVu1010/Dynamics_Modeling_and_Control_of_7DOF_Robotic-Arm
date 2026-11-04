#include "cobot/studio/stl.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>

namespace cobot::studio {
namespace {

using R = common::Result<TriangleMesh>;
constexpr std::size_t kHeaderBytes = 84;    // 80-byte header + uint32 count
constexpr std::size_t kTriangleBytes = 50;  // normal + 3 vertices as float32, uint16 attribute

float f32_at(const std::string& bytes, std::size_t at) {
  std::uint32_t v = 0;
  for (std::size_t i = 0; i < 4; ++i) {
    v |= static_cast<std::uint32_t>(static_cast<unsigned char>(bytes[at + i])) << (8 * i);
  }
  float f = 0.0F;
  std::memcpy(&f, &v, sizeof f);
  return f;
}

// Appends one triangle (normal n, vertices v[0..8]); a zero normal is taken from the winding.
bool add_triangle(TriangleMesh& m, std::array<float, 3> n, const std::array<float, 9>& v) {
  for (const float f : v) {
    if (!std::isfinite(f)) {
      return false;
    }
  }
  float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
  if (!std::isfinite(len) || len == 0.0F) {
    const std::array<float, 3> a = {v[3] - v[0], v[4] - v[1], v[5] - v[2]};
    const std::array<float, 3> b = {v[6] - v[0], v[7] - v[1], v[8] - v[2]};
    n = {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
  }
  for (float& c : n) {
    c = len > 0.0F ? c / len : 0.0F;
  }
  m.positions.insert(m.positions.end(), v.begin(), v.end());
  for (int k = 0; k < 3; ++k) {
    m.normals.insert(m.normals.end(), n.begin(), n.end());
  }
  ++m.triangles;
  return true;
}

R load_ascii(const std::string& text, const std::string& name) {
  TriangleMesh m;
  std::istringstream in(text);
  std::string word;
  std::getline(in, word);  // solid [name]
  while (in >> word && word != "endsolid") {
    std::array<float, 3> n{};
    std::array<float, 9> v{};
    std::string normal, outer, loop, vertex, endloop, endfacet;
    in >> normal >> n[0] >> n[1] >> n[2] >> outer >> loop;
    bool ok = word == "facet" && normal == "normal" && outer == "outer" && loop == "loop";
    for (std::size_t k = 0; k < 3; ++k) {
      in >> vertex >> v[3 * k] >> v[3 * k + 1] >> v[3 * k + 2];
      ok = ok && vertex == "vertex";
    }
    in >> endloop >> endfacet;
    if (!in || !ok || endloop != "endloop" || endfacet != "endfacet") {
      return R::error(name + ": bad ASCII STL facet " + std::to_string(m.triangles + 1));
    }
    if (!add_triangle(m, n, v)) {
      return R::error(name + ": non-finite vertex in facet " + std::to_string(m.triangles + 1));
    }
  }
  if (word != "endsolid") {
    return R::error(name + ": ASCII STL truncated (no endsolid)");
  }
  return R::ok(std::move(m));
}

}  // namespace

common::Result<TriangleMesh> load_stl(const std::filesystem::path& file) {
  const std::string name = file.string();
  std::ifstream in(file, std::ios::binary);
  if (!in) {
    return R::error(name + ": cannot open the STL file");
  }
  const std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  if (bytes.size() < kHeaderBytes) {
    return R::error(name + ": truncated STL (" + std::to_string(bytes.size()) + " bytes)");
  }
  std::uint32_t count = 0;
  for (std::size_t i = 0; i < 4; ++i) {
    count |= static_cast<std::uint32_t>(static_cast<unsigned char>(bytes[80 + i])) << (8 * i);
  }
  const bool binary = bytes.size() == kHeaderBytes + kTriangleBytes * std::size_t{count};
  if (!binary && bytes.compare(0, 5, "solid") == 0) {
    return load_ascii(bytes, name);
  }
  if (!binary) {
    return R::error(name + ": binary STL says " + std::to_string(count) + " triangles (" +
                    std::to_string(kHeaderBytes + kTriangleBytes * std::size_t{count}) +
                    " bytes) but the file has " + std::to_string(bytes.size()) +
                    " bytes (truncated or count mismatch)");
  }
  TriangleMesh m;
  m.positions.reserve(9 * std::size_t{count});
  m.normals.reserve(9 * std::size_t{count});
  for (std::size_t t = 0; t < count; ++t) {
    const std::size_t at = kHeaderBytes + kTriangleBytes * t;
    std::array<float, 3> n{};
    std::array<float, 9> v{};
    for (std::size_t k = 0; k < 3; ++k) {
      n[k] = f32_at(bytes, at + 4 * k);
    }
    for (std::size_t k = 0; k < 9; ++k) {
      v[k] = f32_at(bytes, at + 12 + 4 * k);
    }
    if (!add_triangle(m, n, v)) {
      return R::error(name + ": non-finite vertex in triangle " + std::to_string(t + 1));
    }
  }
  return R::ok(std::move(m));
}

}  // namespace cobot::studio
