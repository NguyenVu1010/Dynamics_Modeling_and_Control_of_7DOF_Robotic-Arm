#include <string>

#include <gtest/gtest.h>

#include "cobot/common/result.hpp"

namespace cobot::common {
namespace {

TEST(Status, OkHasNoReason) {
  const Status s = Status::ok();
  EXPECT_TRUE(s.is_ok());
  EXPECT_TRUE(static_cast<bool>(s));
  EXPECT_EQ(s.reason(), "");
}

TEST(Status, LiteralError) {
  const Status s = Status::error("hal timeout");
  EXPECT_FALSE(s.is_ok());
  EXPECT_EQ(s.reason(), "hal timeout");
}

TEST(Status, OwnedErrorAndNullLiteral) {
  EXPECT_EQ(Status::error(std::string("joint_3: bad limit")).reason(), "joint_3: bad limit");
  EXPECT_EQ(Status::error(static_cast<const char*>(nullptr)).reason(), "unknown error");
}

TEST(Result, ValueAndError) {
  const Result<int> ok = Result<int>::ok(7);
  ASSERT_TRUE(ok.is_ok());
  EXPECT_EQ(ok.value(), 7);
  const Result<int> bad = Result<int>::error("expected 7 revolute joints, got 6");
  EXPECT_FALSE(bad.is_ok());
  EXPECT_EQ(bad.reason(), "expected 7 revolute joints, got 6");
}

}  // namespace
}  // namespace cobot::common
