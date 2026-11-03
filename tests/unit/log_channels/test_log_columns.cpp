// Run CSV channels of the log (feature log-channels, cobot_studio design 6.2, ADR-020): LogRecord
// gains qdd_ref, q_raw, the plant truth (q, qd, qdd, q_motor, tau_motor, tau_friction) and the
// command mode, written after `hold` in this order. 57 + 8 * 7 + 1 = 114 columns for N = 7.
// Truth fields default to NaN (unknown, real robot), always written as `nan`.
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/pipeline/log_record.hpp"

namespace cobot::pipeline {
namespace {

constexpr int kN = 7;
constexpr std::size_t kLegacyColumns = 57;  // 2 + 5 * 7 + 12 + 6 + step_us + hold
constexpr std::size_t kColumns = 114;       // 57 + 8 * 7 + cmd
const double kNan = std::numeric_limits<double>::quiet_NaN();

// Cells of one '\n'-terminated CSV line.
std::vector<std::string> cells(const std::string& text) {
  EXPECT_EQ(text.find('\n'), text.size() - 1);
  std::vector<std::string> out;
  std::stringstream ss(text.substr(0, text.size() - 1));
  std::string cell;
  while (std::getline(ss, cell, ',')) {
    out.push_back(cell);
  }
  return out;
}

std::vector<std::string> header() {
  std::ostringstream os;
  write_csv_header<kN>(os);
  return cells(os.str());
}

std::vector<std::string> row(const LogRecord<kN>& r) {
  std::ostringstream os;
  write_csv_row<kN>(os, r);
  return cells(os.str());
}

// The new blocks in contract order, each N wide, starting at column 57 + 7 k.
const std::vector<std::string> kBlocks = {
    "qddref", "qraw", "qt", "qdt", "qddt", "qmt", "taum", "tauf"};

TEST(LogChannelsCsv, HeaderAppendsTheChannelBlocksAndCmdAfterHold) {
  const std::vector<std::string> cols = header();
  ASSERT_EQ(cols.size(), kColumns);
  EXPECT_EQ(cols[55], "step_us");
  EXPECT_EQ(cols[56], "hold");
  EXPECT_EQ(cols[57], "qddref1");
  for (std::size_t b = 0; b < kBlocks.size(); ++b) {
    for (int j = 1; j <= kN; ++j) {
      const std::size_t c = kLegacyColumns + (7 * b) + static_cast<std::size_t>(j - 1);
      EXPECT_EQ(cols[c], kBlocks[b] + std::to_string(j)) << "column " << c;
    }
  }
  EXPECT_EQ(cols[112], "tauf7");  // 57 + 7 * 7 + 6
  EXPECT_EQ(cols[113], "cmd");
}

TEST(LogChannelsCsv, DefaultsAreNanTruthZeroReferenceAndConfigMode) {
  const LogRecord<kN> r;
  EXPECT_EQ(r.cmd_mode, CommandMode::kConfig);
  for (int j = 0; j < kN; ++j) {
    EXPECT_EQ(r.qdd_ref[j], 0.0);
    EXPECT_EQ(r.q_raw[j], 0.0);
    EXPECT_TRUE(std::isnan(r.q_true[j]));
    EXPECT_TRUE(std::isnan(r.qd_true[j]));
    EXPECT_TRUE(std::isnan(r.qdd_true[j]));
    EXPECT_TRUE(std::isnan(r.q_motor_true[j]));
    EXPECT_TRUE(std::isnan(r.tau_motor[j]));
    EXPECT_TRUE(std::isnan(r.tau_friction[j]));
  }
  const std::vector<std::string> c = row(r);
  ASSERT_EQ(c.size(), kColumns);
  EXPECT_EQ(c[56], "none");
  for (std::size_t i = 57; i < 71; ++i) {  // qddref, qraw: 57 + 2 * 7 = 71
    EXPECT_EQ(c[i], "0") << "column " << i;
  }
  for (std::size_t i = 71; i < 113; ++i) {  // six truth blocks: 71 + 6 * 7 = 113
    EXPECT_EQ(c[i], "nan") << "column " << i;
  }
  EXPECT_EQ(c[113], "config");
}

TEST(LogChannelsCsv, PlantTruthDefaultsToNanEverywhere) {
  const PlantTruth<kN> t;
  for (int j = 0; j < kN; ++j) {
    EXPECT_TRUE(std::isnan(t.q[j]));
    EXPECT_TRUE(std::isnan(t.qd[j]));
    EXPECT_TRUE(std::isnan(t.qdd[j]));
    EXPECT_TRUE(std::isnan(t.q_motor[j]));
    EXPECT_TRUE(std::isnan(t.tau_motor[j]));
    EXPECT_TRUE(std::isnan(t.tau_friction[j]));
  }
}

TEST(LogChannelsCsv, NegativeNanIsWrittenAsNanAndFiniteValuesRoundTripBitExact) {
  LogRecord<kN> r;
  r.cmd_mode = CommandMode::kWaypoints;
  for (int j = 0; j < kN; ++j) {
    const double s = j + 1.0;
    r.qdd_ref[j] = -s / 3.0;
    r.q_raw[j] = 1e-17 * s + 0.1;
    r.q_true[j] = std::sqrt(2.0) * s;
    r.qd_true[j] = -1e300 / s;
    r.qdd_true[j] = 4.9406564584124654e-324 * s;  // subnormal
    r.q_motor_true[j] = std::exp(s);
    r.tau_motor[j] = -123.456789 / s;
    r.tau_friction[j] = -0.0001 * s;
  }
  r.q_true[2] = -kNan;    // sign bit set: printf would write "-nan"
  r.tau_motor[4] = kNan;  // plain NaN
  const std::vector<std::string> c = row(r);
  ASSERT_EQ(c.size(), kColumns);
  EXPECT_EQ(c[57 + 14 + 2], "nan");  // qt3: qt block starts at 57 + 2 * 7 = 71
  EXPECT_EQ(c[57 + 42 + 4], "nan");  // taum5: taum block starts at 57 + 6 * 7 = 99
  const std::vector<const common::VecN<kN>*> blocks = {&r.qdd_ref,
                                                       &r.q_raw,
                                                       &r.q_true,
                                                       &r.qd_true,
                                                       &r.qdd_true,
                                                       &r.q_motor_true,
                                                       &r.tau_motor,
                                                       &r.tau_friction};
  for (std::size_t b = 0; b < blocks.size(); ++b) {
    for (int j = 0; j < kN; ++j) {
      const std::size_t i = kLegacyColumns + (7 * b) + static_cast<std::size_t>(j);
      const double want = (*blocks[b])[j];
      if (std::isnan(want)) {
        EXPECT_EQ(c[i], "nan") << "column " << i;
        continue;
      }
      EXPECT_EQ(std::strtod(c[i].c_str(), nullptr), want) << "column " << i << ": " << c[i];
    }
  }
  for (const std::string& cell : c) {
    EXPECT_EQ(cell.find("-nan"), std::string::npos) << cell;
  }
  EXPECT_EQ(c[113], "waypoints");
}

TEST(LogChannelsCsv, CommandModeStrings) {
  EXPECT_STREQ(to_string(CommandMode::kConfig), "config");
  EXPECT_STREQ(to_string(CommandMode::kWaypoints), "waypoints");
  EXPECT_STREQ(to_string(CommandMode::kJogJoint), "jog_joint");
  EXPECT_STREQ(to_string(CommandMode::kJogTcp), "jog_tcp");
  EXPECT_STREQ(to_string(CommandMode::kMoveJoint), "move_joint");
  EXPECT_STREQ(to_string(CommandMode::kMoveTcp), "move_tcp");
}

}  // namespace
}  // namespace cobot::pipeline
