#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "cobot/pipeline/log_record.hpp"

// The new LoopMode value in the Run CSV `mode` column (docs/conventions.md).
namespace cobot::pipeline {
namespace {

TEST(LoopModeStandby, ToStringIsStandby) {
  EXPECT_STREQ(to_string(LoopMode::kStandby), "standby");
  // The existing values are unchanged.
  EXPECT_STREQ(to_string(LoopMode::kIdle), "idle");
  EXPECT_STREQ(to_string(LoopMode::kTracking), "tracking");
  EXPECT_STREQ(to_string(LoopMode::kHolding), "holding");
  EXPECT_STREQ(to_string(LoopMode::kFinished), "finished");
  EXPECT_STREQ(to_string(LoopMode::kFault), "fault");
}

TEST(LoopModeStandby, CsvRowWritesStandby) {
  LogRecord<7> r;
  r.t.ns = 42;
  r.mode = LoopMode::kStandby;
  std::ostringstream os;
  write_csv_row<7>(os, r);
  EXPECT_EQ(os.str().rfind("42,standby,", 0), 0U) << os.str();
}

}  // namespace
}  // namespace cobot::pipeline
