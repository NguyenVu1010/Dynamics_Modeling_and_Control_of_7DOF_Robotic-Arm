#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/pipeline/config.hpp"

// config/imperfections/ideal.yaml with one edit; the loader must reject it naming the key
// (docs: encoder_bits "must be 0 or an integer in [8, 32]"; `source` is required in every group
// and read as a non-empty string).
namespace cobot::pipeline {
namespace {

namespace fs = std::filesystem;

std::string ideal_text() {
  const std::ifstream in(fs::path(COBOT_REPO_DIR) / "config/imperfections/ideal.yaml",
                         std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::string edited(const std::string& from, const std::string& to) {
  std::string text = ideal_text();
  const auto pos = text.find(from);
  EXPECT_NE(pos, std::string::npos) << "fixture text not found: " << from;
  if (pos != std::string::npos) {
    text.replace(pos, from.size(), to);
  }
  return text;
}

Result<imperfect::ImperfectionConfig<7>> load_text(const std::string& name,
                                                   const std::string& text) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "pipeline_config_imperfections";
  fs::create_directories(dir);
  const fs::path p = dir / name;
  std::ofstream(p, std::ios::binary) << text;
  return load_imperfections<7>(p);
}

void expect_rejected(const std::string& name,
                     const std::string& from,
                     const std::string& to,
                     const std::string& key) {
  const auto r = load_text(name, edited(from, to));
  ASSERT_FALSE(r.is_ok()) << key;
  EXPECT_NE(r.reason().find(key), std::string::npos) << r.reason();
}

TEST(LoadImperfections, NonIntegerEncoderBitsRejected) {
  expect_rejected("bits_scalar.yaml",
                  "encoder_bits: 0 ",
                  "encoder_bits: 12.5 ",
                  "imperfections.sensor.encoder_bits");
  expect_rejected("bits_list.yaml",
                  "encoder_bits: 0 ",
                  "encoder_bits: [16, 16, 16, 16.25, 16, 16, 16] ",
                  "imperfections.sensor.encoder_bits[3]");
}

TEST(LoadImperfections, IntegerEncoderBitsAtBoundsAccepted) {
  const auto r = load_text("bits_ok.yaml",
                           edited("encoder_bits: 0 ", "encoder_bits: [8, 32, 0, 19, 8, 32, 0] "));
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().sensor.encoder_bits[0], 8.0);
  EXPECT_EQ(r.value().sensor.encoder_bits[1], 32.0);
  EXPECT_EQ(r.value().sensor.encoder_bits[2], 0.0);
}

TEST(LoadImperfections, EmptySourceRejectedInEveryGroup) {
  // ideal.yaml lists the groups in this order, each with `source: "ideal"`.
  const std::vector<std::string> groups = {"sensor", "actuator", "friction", "model", "timing"};
  std::size_t k = 0;
  for (const std::string& group : groups) {
    std::string text = ideal_text();
    std::size_t pos = text.find("  " + group + ":");
    ASSERT_NE(pos, std::string::npos) << group;
    pos = text.find("source: \"ideal\"", pos);
    ASSERT_NE(pos, std::string::npos) << group;
    text.replace(pos, std::string("source: \"ideal\"").size(), "source: \"\"");
    const auto r = load_text("source_" + std::to_string(k++) + ".yaml", text);
    ASSERT_FALSE(r.is_ok()) << group;
    EXPECT_NE(r.reason().find("imperfections." + group + ".source"), std::string::npos)
        << r.reason();
  }
}

TEST(LoadImperfections, NullSourceRejected) {
  expect_rejected("source_null.yaml",
                  "    source: \"ideal\"\n    encoder_bits",
                  "    source:\n    encoder_bits",
                  "imperfections.sensor.source");
}

}  // namespace
}  // namespace cobot::pipeline
