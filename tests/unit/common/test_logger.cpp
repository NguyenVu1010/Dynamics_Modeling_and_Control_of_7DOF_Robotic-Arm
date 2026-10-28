#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/logger.hpp"

namespace cobot::common {
namespace {

class RecordingLogger final : public Logger {
 public:
  void log(LogLevel level, std::string_view msg) override {
    entries.emplace_back(level, std::string(msg));
  }
  std::vector<std::pair<LogLevel, std::string>> entries;
};

TEST(Logger, RoutesEveryLevelToTheInstalledSink) {
  RecordingLogger sink;
  set_logger(&sink);
  log_debug("d");
  log_info("i");
  log_warn("w");
  log_error("e");
  set_logger(nullptr);
  ASSERT_EQ(sink.entries.size(), 4U);
  EXPECT_EQ(sink.entries[0], std::make_pair(LogLevel::kDebug, std::string("d")));
  EXPECT_EQ(sink.entries[3], std::make_pair(LogLevel::kError, std::string("e")));
}

TEST(Logger, NullRestoresTheDefaultSink) {
  set_logger(nullptr);
  EXPECT_NE(&logger(), nullptr);
  // The default sink writes to stderr for every level and must not crash.
  log_debug("default sink debug");
  log_info("default sink info");
  log_warn("default sink warn");
  log_error("default sink error");
}

}  // namespace
}  // namespace cobot::common
