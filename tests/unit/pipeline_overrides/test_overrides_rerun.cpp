// Exact reruns from written files (feature studio-core, ADR-021): write_config_with and
// write_imperfections_with with no overrides keep every value's source text and make relative
// paths absolute, so cobot_bench's `run` on the written pair gives the same Run CSV bytes as on the
// source pair (2 s, seed 3), and writing a written file again gives identical text.
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/pipeline/overrides.hpp"

#include "run.hpp"

namespace cobot::pipeline {
namespace {

namespace fs = std::filesystem;

const fs::path kRepo = COBOT_REPO_DIR;
constexpr double kRunS = 2.0;  // simulated s per run (ASan budget)
constexpr std::uint64_t kSeed = 3;

struct Pair {
  std::string name;
  fs::path config;
  fs::path imperfections;
  std::optional<fs::path> waypoints;
};

std::string read_text(const fs::path& p) {
  std::ifstream in(p, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

class OverridesRerun : public ::testing::TestWithParam<Pair> {};

TEST_P(OverridesRerun, WrittenPairGivesTheSameRunCsvBytes) {
  const Pair& p = GetParam();
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "pipeline_overrides_rerun" / p.name;
  fs::create_directories(dir);
  const fs::path config = dir / "config.yaml";
  const fs::path imperfections = dir / "imperfections.yaml";
  fs::remove(config);
  fs::remove(imperfections);

  const common::Status wc = write_config_with(p.config, {}, config);
  ASSERT_TRUE(wc.is_ok()) << wc.reason();
  const common::Status wi = write_imperfections_with(p.imperfections, {}, imperfections);
  ASSERT_TRUE(wi.is_ok()) << wi.reason();

  const fs::path from_source = dir / "source.csv";
  const fs::path from_written = dir / "written.csv";
  const common::Status a =
      bench::run(p.config, p.imperfections, kSeed, from_source, kRunS, p.waypoints);
  ASSERT_TRUE(a.is_ok()) << a.reason();
  const common::Status b =
      bench::run(config, imperfections, kSeed, from_written, kRunS, p.waypoints);
  ASSERT_TRUE(b.is_ok()) << b.reason();
  const std::string source_csv = read_text(from_source);
  ASSERT_FALSE(source_csv.empty());
  EXPECT_TRUE(source_csv == read_text(from_written)) << "Run CSV bytes differ for " << p.name;

  // Writing the written files again changes nothing.
  const fs::path config2 = dir / "config_again.yaml";
  const fs::path imperfections2 = dir / "imperfections_again.yaml";
  ASSERT_TRUE(write_config_with(config, {}, config2).is_ok());
  ASSERT_TRUE(write_imperfections_with(imperfections, {}, imperfections2).is_ok());
  EXPECT_EQ(read_text(config2), read_text(config));
  EXPECT_EQ(read_text(imperfections2), read_text(imperfections));
}

INSTANTIATE_TEST_SUITE_P(
    Profiles,
    OverridesRerun,
    ::testing::Values(Pair{"default_ideal",
                           kRepo / "config/default.yaml",
                           kRepo / "config/imperfections/ideal.yaml",
                           std::nullopt},
                      Pair{"default_realistic",
                           kRepo / "config/default.yaml",
                           kRepo / "config/imperfections/realistic.yaml",
                           std::nullopt},
                      Pair{"premium_fr3",
                           kRepo / "config/premium_fr3.yaml",
                           kRepo / "config/imperfections/premium_fr3.yaml",
                           std::nullopt},
                      Pair{"mid_kr1205",
                           kRepo / "config/mid_kr1205.yaml",
                           kRepo / "config/imperfections/mid_kr1205.yaml",
                           std::nullopt},
                      Pair{"budget_xarm7",
                           kRepo / "config/budget_xarm7.yaml",
                           kRepo / "config/imperfections/budget_xarm7.yaml",
                           std::nullopt},
                      // E2E-10: default + realistic with the waypoint list (seed 3 here, 2 s).
                      Pair{"e2e_10_waypoints",
                           kRepo / "config/default.yaml",
                           kRepo / "config/imperfections/realistic.yaml",
                           kRepo / "tests/e2e/waypoints/e2e_10.yaml"}),
    [](const ::testing::TestParamInfo<Pair>& info) { return info.param.name; });

}  // namespace
}  // namespace cobot::pipeline
