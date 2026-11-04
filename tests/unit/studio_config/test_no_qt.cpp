// cobot_studio_core has no Qt (ADR-021): no file under src/studio/core includes a <Q...> header.
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <regex>
#include <string>

#include <gtest/gtest.h>

namespace {

namespace fs = std::filesystem;

TEST(StudioCoreNoQt, NoSourceIncludesAQtHeader) {
  const fs::path core = fs::path(COBOT_REPO_DIR) / "src/studio/core";
  ASSERT_TRUE(fs::is_directory(core)) << core;
  const std::regex qt_include(R"(^\s*#\s*include\s*<Q)");
  std::size_t files = 0;
  for (const fs::directory_entry& e : fs::recursive_directory_iterator(core)) {
    if (!e.is_regular_file()) {
      continue;
    }
    ++files;
    std::ifstream in(e.path(), std::ios::binary);
    std::string line;
    std::size_t number = 0;
    while (std::getline(in, line)) {
      ++number;
      EXPECT_FALSE(std::regex_search(line, qt_include))
          << e.path() << ":" << number << ": " << line;
    }
  }
  EXPECT_GT(files, 0U);
}

}  // namespace
