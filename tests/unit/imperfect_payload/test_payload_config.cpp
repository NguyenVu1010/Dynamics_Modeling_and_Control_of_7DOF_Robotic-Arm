// Feature payload-inertia, loader side: imperfections.model.payload_offset_m ([x, y, z], m, in
// the link_7 frame, finite) and imperfections.model.payload_radius_m (m, >= 0). The offset is a
// vector and cannot be swept; the radius can.
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "cobot/pipeline/config.hpp"
#include "cobot/pipeline/sweep_config.hpp"

namespace cobot::pipeline {
namespace {

namespace fs = std::filesystem;

std::string read_file(const fs::path& p) {
  const std::ifstream in(p, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

// Removes every line that starts with `prefix` (the profile may or may not carry the new keys).
std::string drop_lines(const std::string& text, const std::string& prefix) {
  std::string out;
  std::size_t begin = 0;
  while (begin < text.size()) {
    std::size_t end = text.find('\n', begin);
    end = end == std::string::npos ? text.size() : end + 1;
    const std::string line = text.substr(begin, end - begin);
    if (line.rfind(prefix, 0) != 0) {
      out += line;
    }
    begin = end;
  }
  return out;
}

// config/imperfections/ideal.yaml with the model group's payload keys set to the given YAML
// values (payload_kg, payload_offset_m, payload_radius_m).
std::string ideal_with(const std::string& kg, const std::string& offset, const std::string& r) {
  std::string text = read_file(fs::path(COBOT_REPO_DIR) / "config/imperfections/ideal.yaml");
  text = drop_lines(text, "    payload_offset_m:");
  text = drop_lines(text, "    payload_radius_m:");
  const std::string from = "    payload_kg: 0.0\n";
  const auto pos = text.find(from);
  EXPECT_NE(pos, std::string::npos) << "fixture text not found: " << from;
  if (pos != std::string::npos) {
    text.replace(pos,
                 from.size(),
                 "    payload_kg: " + kg + "\n    payload_offset_m: " + offset +
                     "\n    payload_radius_m: " + r + "\n");
  }
  return text;
}

fs::path write_temp(const std::string& name, const std::string& text) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "imperfect_payload";
  fs::create_directories(dir);
  const fs::path p = dir / name;
  std::ofstream(p, std::ios::binary) << text;
  return p;
}

TEST(LoadPayload, ParsesOffsetAndRadius) {
  const auto r =
      load_imperfections<7>(write_temp("ok.yaml", ideal_with("2.0", "[0.01, -0.02, 0.1]", "0.05")));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  const auto& m = r.value().model;
  EXPECT_DOUBLE_EQ(m.payload_kg, 2.0);
  EXPECT_DOUBLE_EQ(m.payload_offset_m.x(), 0.01);
  EXPECT_DOUBLE_EQ(m.payload_offset_m.y(), -0.02);
  EXPECT_DOUBLE_EQ(m.payload_offset_m.z(), 0.1);
  EXPECT_DOUBLE_EQ(m.payload_radius_m, 0.05);
}

TEST(LoadPayload, ZeroValuesAccepted) {
  const auto r =
      load_imperfections<7>(write_temp("zero.yaml", ideal_with("0.0", "[0, 0, 0]", "0")));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_TRUE(r.value().model.payload_offset_m.isZero());
  EXPECT_EQ(r.value().model.payload_radius_m, 0.0);
}

// Every shipped profile still loads and has a well-formed payload (radius >= 0, finite offset).
TEST(LoadPayload, ShippedProfilesLoad) {
  for (const char* name : {"ideal", "realistic", "pessimistic"}) {
    const auto r = load_imperfections<7>(fs::path(COBOT_REPO_DIR) / "config/imperfections" /
                                         (std::string(name) + ".yaml"));
    ASSERT_TRUE(r.is_ok()) << name << ": " << r.reason();
    EXPECT_GE(r.value().model.payload_radius_m, 0.0) << name;
    EXPECT_TRUE(r.value().model.payload_offset_m.allFinite()) << name;
  }
}

TEST(LoadPayload, NegativeRadiusRejectedNamingKey) {
  const auto r =
      load_imperfections<7>(write_temp("neg_r.yaml", ideal_with("1.0", "[0, 0, 0]", "-0.01")));
  ASSERT_FALSE(r.is_ok());
  EXPECT_NE(r.reason().find("imperfections.model.payload_radius_m"), std::string::npos)
      << r.reason();
}

TEST(LoadPayload, NonFiniteRadiusRejectedNamingKey) {
  const auto r =
      load_imperfections<7>(write_temp("nan_r.yaml", ideal_with("1.0", "[0, 0, 0]", ".nan")));
  ASSERT_FALSE(r.is_ok());
  EXPECT_NE(r.reason().find("imperfections.model.payload_radius_m"), std::string::npos)
      << r.reason();
}

TEST(LoadPayload, BadOffsetRejectedNamingKey) {
  int n = 0;
  for (const char* offset : {"[.nan, 0, 0]",
                             "[0, .inf, 0]",
                             "[0, 0, -.inf]",
                             "[0, 0]",
                             "[0, 0, 0, 0]",
                             "0.1",
                             "[a, 0, 0]"}) {
    const auto r = load_imperfections<7>(
        write_temp("bad_off" + std::to_string(n++) + ".yaml", ideal_with("1.0", offset, "0.0")));
    ASSERT_FALSE(r.is_ok()) << offset;
    EXPECT_NE(r.reason().find("imperfections.model.payload_offset_m"), std::string::npos)
        << offset << ": " << r.reason();
  }
}

TEST(SweepPayload, RadiusIsSweepable) {
  const fs::path p = write_temp("sweep.yaml", ideal_with("1.0", "[0, 0, 0.1]", "0.0"));
  const auto r = load_imperfections_with<7>(p, "model.payload_radius_m", 0.03);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_DOUBLE_EQ(r.value().model.payload_radius_m, 0.03);
  EXPECT_DOUBLE_EQ(r.value().model.payload_offset_m.z(), 0.1);
  EXPECT_DOUBLE_EQ(r.value().model.payload_kg, 1.0);
}

TEST(SweepPayload, OffsetIsNotSweepable) {
  const fs::path p = write_temp("sweep_off.yaml", ideal_with("1.0", "[0, 0, 0.1]", "0.0"));
  const auto r = load_imperfections_with<7>(p, "model.payload_offset_m", 0.1);
  ASSERT_FALSE(r.is_ok());
  EXPECT_NE(r.reason().find("payload_offset_m"), std::string::npos) << r.reason();
}

}  // namespace
}  // namespace cobot::pipeline
