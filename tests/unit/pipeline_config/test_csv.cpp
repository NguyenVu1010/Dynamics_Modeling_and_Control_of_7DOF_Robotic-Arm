#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "cobot/pipeline/log_record.hpp"

namespace cobot::pipeline {
namespace {

constexpr double kPi = 3.14159265358979323846;

// Splits one CSV line (which must end in '\n') into its cells; a trailing comma gives an
// empty last cell.
std::vector<std::string> cells(const std::string& text) {
  EXPECT_FALSE(text.empty());
  EXPECT_EQ(text.find('\r'), std::string::npos);
  EXPECT_EQ(text.find('\n'), text.size() - 1);  // exactly one line, '\n' terminated
  const std::string line = text.empty() ? text : text.substr(0, text.size() - 1);
  std::vector<std::string> out;
  std::string cell;
  std::stringstream ss(line);
  while (std::getline(ss, cell, ',')) {
    out.push_back(cell);
  }
  if (!line.empty() && line.back() == ',') {
    out.emplace_back();
  }
  return out;
}

LogRecord<7> sample_record() {
  LogRecord<7> r;
  r.t.ns = 1234567891011LL;
  r.mode = LoopMode::kTracking;
  for (int i = 0; i < 7; ++i) {
    r.q[i] = 0.1 * (i + 1) + 1.0 / 3.0;
    r.qd[i] = -std::sqrt(2.0) * (i + 1);
    r.q_ref[i] = 1e-17 * (i + 1) + std::exp(1.0 + i);
    r.qd_ref[i] = 123456.789 / (i + 7);
    r.tau[i] = -0.0001 * (i + 1) - (i == 6 ? 1e300 : 0.0);
  }
  r.T_b_e.p = common::Vec3(0.1, -0.2, 0.30000000000000004);
  r.T_b_e.R = Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
  r.T_b_d.p = common::Vec3(-0.5, -0.6, 0.9);
  r.T_b_d.R = Eigen::AngleAxisd(kPi / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  r.task_error << 1e-9, -2e-9 / 7.0, 3.0 / 7.0, -4.5e-3, 5e-7 / 3.0, 6.0;
  r.step_time_us = 42.5;
  return r;
}

TEST(LoopMode, Strings) {
  EXPECT_STREQ(to_string(LoopMode::kIdle), "idle");
  EXPECT_STREQ(to_string(LoopMode::kTracking), "tracking");
  EXPECT_STREQ(to_string(LoopMode::kHolding), "holding");
  EXPECT_STREQ(to_string(LoopMode::kFinished), "finished");
  EXPECT_STREQ(to_string(LoopMode::kFault), "fault");
}

TEST(Csv, HeaderHasTheContractColumns) {
  std::vector<std::string> expected = {"t_ns", "mode"};
  for (const char* prefix : {"q", "qd", "qref", "qdref", "tau"}) {
    for (int i = 1; i <= 7; ++i) {
      expected.push_back(prefix + std::to_string(i));
    }
  }
  for (const char* c :
       {"px", "py", "pz", "rx", "ry", "rz", "pdx", "pdy", "pdz", "rdx", "rdy", "rdz"}) {
    expected.emplace_back(c);
  }
  for (int i = 1; i <= 6; ++i) {
    expected.push_back("e" + std::to_string(i));
  }
  expected.emplace_back("step_us");
  std::ostringstream os;
  write_csv_header<7>(os);
  const auto cols = cells(os.str());
  ASSERT_EQ(cols.size(), 56U);
  EXPECT_EQ(cols, expected);
}

TEST(Csv, RowRoundTripsBitExact) {
  const LogRecord<7> r = sample_record();
  std::ostringstream os;
  write_csv_row<7>(os, r);
  const auto cols = cells(os.str());
  ASSERT_EQ(cols.size(), 56U);
  for (const auto& c : cols) {
    EXPECT_FALSE(c.empty());
  }
  EXPECT_EQ(cols[0], "1234567891011");
  EXPECT_EQ(cols[1], "tracking");
  auto num = [&](std::size_t i) { return std::strtod(cols[i].c_str(), nullptr); };
  for (int i = 0; i < 7; ++i) {
    EXPECT_EQ(num(2 + i), r.q[i]) << cols[2 + i];
    EXPECT_EQ(num(9 + i), r.qd[i]);
    EXPECT_EQ(num(16 + i), r.q_ref[i]);
    EXPECT_EQ(num(23 + i), r.qd_ref[i]);
    EXPECT_EQ(num(30 + i), r.tau[i]);
  }
  // p exact; r = so3_log(R) computed by the writer.
  const Eigen::Vector3d r_e = 0.7 * Eigen::Vector3d(1, 2, 3).normalized();
  const Eigen::Vector3d r_d(0.0, 0.0, kPi / 2.0);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(num(37 + i), r.T_b_e.p[i]);
    EXPECT_NEAR(num(40 + i), r_e[i], 1e-12);
    EXPECT_EQ(num(43 + i), r.T_b_d.p[i]);
    EXPECT_NEAR(num(46 + i), r_d[i], 1e-12);
  }
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(num(49 + i), r.task_error[i]);
  }
  EXPECT_EQ(num(55), 42.5);
}

TEST(Csv, DefaultRecordFaultMode) {
  LogRecord<7> r;
  r.mode = LoopMode::kFault;
  std::ostringstream os;
  write_csv_row<7>(os, r);
  const auto cols = cells(os.str());
  ASSERT_EQ(cols.size(), 56U);
  EXPECT_EQ(cols[0], "0");
  EXPECT_EQ(cols[1], "fault");
  for (std::size_t i = 2; i < cols.size(); ++i) {
    EXPECT_EQ(std::strtod(cols[i].c_str(), nullptr), 0.0) << i;
  }
}

TEST(Csv, IdenticalRecordsGiveIdenticalBytes) {
  std::ostringstream a;
  std::ostringstream b;
  write_csv_header<7>(a);
  write_csv_row<7>(a, sample_record());
  write_csv_header<7>(b);
  write_csv_row<7>(b, sample_record());
  EXPECT_EQ(a.str(), b.str());
}

}  // namespace
}  // namespace cobot::pipeline
