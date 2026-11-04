// load_stl (feature studio-3d): binary and ASCII STL into flat float arrays, one position and one
// normal per vertex (9 floats per triangle each, the layout of one GPU vertex buffer). Errors name
// the file: truncated, triangle count that does not match the file size, non-finite values.
//
// Binary STL layout: 80-byte header, uint32 triangle count, then 50 bytes per triangle (normal,
// three vertices as 12 little-endian float32, uint16 attribute), so a valid file has
// size == 84 + 50 * triangles.
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/studio/stl.hpp"

#include "studio_scene_test_util.hpp"

namespace cobot::studio {
namespace {

using test::kRepo;
namespace fs = std::filesystem;

constexpr std::uintmax_t kHeaderBytes = 84;    // 80-byte header + uint32 count
constexpr std::uintmax_t kTriangleBytes = 50;  // 12 float32 + uint16

struct MeshFact {
  const char* name;
  std::size_t triangles;  // (file size - 84) / 50
  double max_radius_m;    // bound on |vertex| in the link frame
};

// Triangle counts from the file sizes (bytes):
//   base_link (281984 - 84) / 50 = 5638     link_1   (5264884 - 84) / 50 = 105296
//   link_2    (5264884 - 84) / 50 = 105296  link_3   (5154384 - 84) / 50 = 103086
//   link_4    (4866284 - 84) / 50 = 97324   link_5   (4467484 - 84) / 50 = 89348
//   link_6    (4176284 - 84) / 50 = 83524   link_6_2 (4176284 - 84) / 50 = 83524
//   link_7    (240784 - 84) / 50 = 4814
// Units are metres (no <mesh scale> in the URDF). Largest |vertex| measured from the files:
// base_link 0.116, link_1/link_2 0.095, link_3 0.583 (the upper-arm tube reaches down to
// z = -0.581), link_4 0.078, link_5 0.432, link_6/link_6_2 0.063, link_7 0.050. So the bound
// is 0.5 m except link_3 (0.6 m); a millimetre file would be ~1000 times larger.
constexpr std::array<MeshFact, 9> kMeshes = {{{"base_link", 5638, 0.5},
                                              {"link_1", 105296, 0.5},
                                              {"link_2", 105296, 0.5},
                                              {"link_3", 103086, 0.6},
                                              {"link_4", 97324, 0.5},
                                              {"link_5", 89348, 0.5},
                                              {"link_6", 83524, 0.5},
                                              {"link_6_2", 83524, 0.5},
                                              {"link_7", 4814, 0.5}}};

fs::path out_dir() {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "studio_scene";
  fs::create_directories(dir);
  return dir;
}

void write_bytes(const fs::path& file, const std::vector<char>& bytes) {
  std::ofstream os(file, std::ios::binary | std::ios::trunc);
  os.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
}

void put_u32(std::vector<char>& bytes, std::uint32_t v) {
  for (int i = 0; i < 4; ++i) {
    bytes.push_back(static_cast<char>((v >> (8 * i)) & 0xFFU));
  }
}

void put_f32(std::vector<char>& bytes, float f) {
  std::uint32_t v = 0;
  std::memcpy(&v, &f, sizeof v);
  put_u32(bytes, v);
}

// Binary STL with `count` in the header and the given triangles (normal + 3 vertices each).
std::vector<char> binary_stl(std::uint32_t count, const std::vector<std::array<float, 12>>& tris) {
  std::vector<char> bytes(80, '\0');
  put_u32(bytes, count);
  for (const auto& t : tris) {
    for (const float f : t) {
      put_f32(bytes, f);
    }
    bytes.push_back('\0');
    bytes.push_back('\0');
  }
  return bytes;
}

// Normal (0, 0, 1), vertices (0, 0, 0), (0.1, 0, 0), (0, 0.1, 0): counter-clockwise about +z.
constexpr std::array<float, 12> kFlatTriangle = {
    0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F};

TEST(Stl, EveryArmMeshLoadsWithTheCountOfItsFileSize) {
  for (const MeshFact& fact : kMeshes) {
    SCOPED_TRACE(fact.name);
    const fs::path file = kRepo / "src/sim/meshes" / (std::string(fact.name) + ".STL");
    ASSERT_EQ((fs::file_size(file) - kHeaderBytes) % kTriangleBytes, 0U);
    ASSERT_EQ((fs::file_size(file) - kHeaderBytes) / kTriangleBytes, fact.triangles);
    const auto mesh = load_stl(file);
    ASSERT_TRUE(mesh.is_ok()) << mesh.reason();
    const TriangleMesh& m = mesh.value();
    EXPECT_EQ(m.triangles, fact.triangles);
    ASSERT_EQ(m.positions.size(), 9 * fact.triangles);
    ASSERT_EQ(m.normals.size(), 9 * fact.triangles);
    double max_radius = 0.0;
    bool finite = true;
    for (std::size_t v = 0; v + 2 < m.positions.size(); v += 3) {
      const double x = m.positions[v];
      const double y = m.positions[v + 1];
      const double z = m.positions[v + 2];
      finite = finite && std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
      max_radius = std::max(max_radius, std::sqrt(x * x + y * y + z * z));
    }
    for (const float n : m.normals) {
      finite = finite && std::isfinite(n);
    }
    EXPECT_TRUE(finite);
    EXPECT_LE(max_radius, fact.max_radius_m);
    EXPECT_GT(max_radius, 0.01) << "metres, not empty";
  }
}

TEST(Stl, AsciiFileWithThreeTrianglesLoads) {
  // Normals agree with the counter-clockwise vertex order:
  //   t0: e1 = (0.1, 0, 0), e2 = (0, 0.1, 0), e1 x e2 = (0, 0, 0.01)   -> +z
  //   t1: e1 = (0, 0.1, 0), e2 = (0.1, 0, 0), e1 x e2 = (0, 0, -0.01)  -> -z
  //   t2: e1 = (0, 0.1, 0), e2 = (0, 0, 0.1), e1 x e2 = (0.01, 0, 0)   -> +x
  const std::string text =
      "solid three\n"
      "  facet normal 0 0 1\n"
      "    outer loop\n"
      "      vertex 0 0 0\n"
      "      vertex 0.1 0 0\n"
      "      vertex 0 0.1 0\n"
      "    endloop\n"
      "  endfacet\n"
      "  facet normal 0 0 -1\n"
      "    outer loop\n"
      "      vertex 0 0 0.05\n"
      "      vertex 0 0.1 0.05\n"
      "      vertex 0.1 0 0.05\n"
      "    endloop\n"
      "  endfacet\n"
      "  facet normal 1 0 0\n"
      "    outer loop\n"
      "      vertex 0.02 0 0\n"
      "      vertex 0.02 0.1 0\n"
      "      vertex 0.02 0 0.1\n"
      "    endloop\n"
      "  endfacet\n"
      "endsolid three\n";
  const fs::path file = out_dir() / "three_triangles_ascii.stl";
  write_bytes(file, std::vector<char>(text.begin(), text.end()));
  const auto mesh = load_stl(file);
  ASSERT_TRUE(mesh.is_ok()) << mesh.reason();
  const TriangleMesh& m = mesh.value();
  EXPECT_EQ(m.triangles, 3U);
  const std::vector<float> positions = {0.0F,  0.0F, 0.0F,  0.1F,  0.0F, 0.0F,  0.0F,  0.1F, 0.0F,
                                        0.0F,  0.0F, 0.05F, 0.0F,  0.1F, 0.05F, 0.1F,  0.0F, 0.05F,
                                        0.02F, 0.0F, 0.0F,  0.02F, 0.1F, 0.0F,  0.02F, 0.0F, 0.1F};
  const std::vector<float> normals = {0.0F, 0.0F, 1.0F,  0.0F, 0.0F, 1.0F,  0.0F, 0.0F, 1.0F,
                                      0.0F, 0.0F, -1.0F, 0.0F, 0.0F, -1.0F, 0.0F, 0.0F, -1.0F,
                                      1.0F, 0.0F, 0.0F,  1.0F, 0.0F, 0.0F,  1.0F, 0.0F, 0.0F};
  ASSERT_EQ(m.positions.size(), positions.size());
  ASSERT_EQ(m.normals.size(), normals.size());
  for (std::size_t i = 0; i < positions.size(); ++i) {
    EXPECT_FLOAT_EQ(m.positions[i], positions[i]) << "position float " << i;
    EXPECT_NEAR(m.normals[i], normals[i], 1e-6) << "normal float " << i;
  }
}

TEST(Stl, SmallBinaryFileLoads) {
  const fs::path file = out_dir() / "one_triangle.stl";
  write_bytes(file, binary_stl(1, {kFlatTriangle}));
  const auto mesh = load_stl(file);
  ASSERT_TRUE(mesh.is_ok()) << mesh.reason();
  const TriangleMesh& m = mesh.value();
  EXPECT_EQ(m.triangles, 1U);
  ASSERT_EQ(m.positions.size(), 9U);
  ASSERT_EQ(m.normals.size(), 9U);
  for (std::size_t i = 0; i < 9; ++i) {
    EXPECT_FLOAT_EQ(m.positions[i], kFlatTriangle[3 + i]) << i;
    EXPECT_NEAR(m.normals[i], kFlatTriangle[i % 3], 1e-6) << i;
  }
}

TEST(Stl, TruncatedCopyOfLink7IsAnErrorNamingTheFile) {
  // The header still says 4814 triangles, but (1000 - 84) / 50 = 18.32 fit in the file.
  const fs::path src = kRepo / "src/sim/meshes/link_7.STL";
  std::ifstream in(src, std::ios::binary);
  std::vector<char> bytes(1000);
  in.read(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  ASSERT_EQ(in.gcount(), 1000);
  const fs::path file = out_dir() / "truncated_link_7.STL";
  write_bytes(file, bytes);
  const auto mesh = load_stl(file);
  ASSERT_FALSE(mesh.is_ok());
  EXPECT_NE(mesh.reason().find("truncated_link_7.STL"), std::string::npos) << mesh.reason();
}

TEST(Stl, CountThatDoesNotMatchTheSizeIsAnErrorNamingTheFile) {
  // 84 + 3 * 50 = 234 bytes hold three triangles, the header says two.
  const fs::path file = out_dir() / "count_mismatch.stl";
  write_bytes(file, binary_stl(2, {kFlatTriangle, kFlatTriangle, kFlatTriangle}));
  const auto mesh = load_stl(file);
  ASSERT_FALSE(mesh.is_ok());
  EXPECT_NE(mesh.reason().find("count_mismatch.stl"), std::string::npos) << mesh.reason();
}

TEST(Stl, NonFiniteVertexIsAnErrorNamingTheFile) {
  std::array<float, 12> bad = kFlatTriangle;
  bad[7] = std::numeric_limits<float>::quiet_NaN();  // y of the second vertex
  const fs::path file = out_dir() / "non_finite.stl";
  write_bytes(file, binary_stl(2, {kFlatTriangle, bad}));
  const auto mesh = load_stl(file);
  ASSERT_FALSE(mesh.is_ok());
  EXPECT_NE(mesh.reason().find("non_finite.stl"), std::string::npos) << mesh.reason();
}

TEST(Stl, MissingFileIsAnErrorNamingThePath) {
  const fs::path file = out_dir() / "no_such_mesh.stl";
  fs::remove(file);
  const auto mesh = load_stl(file);
  ASSERT_FALSE(mesh.is_ok());
  EXPECT_NE(mesh.reason().find("no_such_mesh.stl"), std::string::npos) << mesh.reason();
}

}  // namespace
}  // namespace cobot::studio
