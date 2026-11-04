// Base pairs of the studio (feature studio-core, ADR-021, cobot/studio/profile_catalog.hpp): one
// pair per config_dir/imperfections/*.yaml, ordered ideal, realistic, pessimistic, premium_fr3,
// mid_kr1205, budget_xarm7, then the others alphabetically; the controller config is
// config_dir/<name>.yaml when it exists, else config_dir/default.yaml.
#include <cstddef>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/studio/profile_catalog.hpp"

namespace cobot::studio {
namespace {

namespace fs = std::filesystem;

const fs::path kRepo = COBOT_REPO_DIR;
const fs::path kConfigDir = kRepo / "config";

bool same_file(const fs::path& a, const fs::path& b) {
  std::error_code ec;
  return fs::equivalent(a, b, ec) && !ec;
}

std::vector<std::string> names(const std::vector<BasePair>& pairs) {
  std::vector<std::string> out;
  out.reserve(pairs.size());
  for (const BasePair& p : pairs) {
    out.push_back(p.name);
  }
  return out;
}

const std::vector<std::string> kPinned{
    "ideal", "realistic", "pessimistic", "premium_fr3", "mid_kr1205", "budget_xarm7"};

TEST(ProfileCatalog, SixProfilesInThePinnedOrder) {
  const std::vector<BasePair> pairs = discover_base_pairs(kConfigDir);
  EXPECT_EQ(names(pairs), kPinned);
}

TEST(ProfileCatalog, ControllerConfigIsTheProfilesOwnElseDefault) {
  const std::vector<BasePair> pairs = discover_base_pairs(kConfigDir);
  ASSERT_EQ(pairs.size(), kPinned.size());
  for (const BasePair& p : pairs) {
    const fs::path own = kConfigDir / (p.name + ".yaml");
    const fs::path expected = fs::exists(own) ? own : kConfigDir / "default.yaml";
    EXPECT_TRUE(same_file(p.config_yaml, expected)) << p.name << ": " << p.config_yaml;
    EXPECT_TRUE(same_file(p.imperfections_yaml, kConfigDir / "imperfections" / (p.name + ".yaml")))
        << p.name << ": " << p.imperfections_yaml;
  }
  EXPECT_TRUE(same_file(pairs[3].config_yaml, kConfigDir / "premium_fr3.yaml"));
  EXPECT_TRUE(same_file(pairs[1].config_yaml, kConfigDir / "default.yaml"));  // realistic
}

TEST(ProfileCatalog, OtherProfilesFollowAlphabetically) {
  // A copy of config/ with two extra profiles: both after the six, in alphabetical order.
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "studio_profile_catalog" / "config";
  fs::remove_all(dir);
  fs::create_directories(dir.parent_path());
  fs::copy(kConfigDir, dir, fs::copy_options::recursive);
  fs::copy_file(dir / "imperfections/ideal.yaml", dir / "imperfections/zz_test.yaml");
  fs::copy_file(dir / "imperfections/ideal.yaml", dir / "imperfections/aa_test.yaml");
  const std::vector<BasePair> pairs = discover_base_pairs(dir);
  std::vector<std::string> expected = kPinned;
  expected.emplace_back("aa_test");
  expected.emplace_back("zz_test");
  EXPECT_EQ(names(pairs), expected);
  ASSERT_EQ(pairs.size(), expected.size());
  EXPECT_TRUE(same_file(pairs.back().config_yaml, dir / "default.yaml"));
  EXPECT_TRUE(same_file(pairs.back().imperfections_yaml, dir / "imperfections/zz_test.yaml"));
}

}  // namespace
}  // namespace cobot::studio
