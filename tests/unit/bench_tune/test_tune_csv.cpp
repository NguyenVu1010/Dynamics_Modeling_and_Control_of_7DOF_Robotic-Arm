// tune.csv (brief auto-tune, section 6; docs/conventions.md "Tune CSV"): the header, %.17g values
// that read back exactly, 0/1 flags as `held` and `passed` of the Summary CSV, and the `chosen`
// column.
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/control/algorithm.hpp"

#include "tune.hpp"

namespace cobot::bench {
namespace {

namespace fs = std::filesystem;

const char* const kHeader =
    "level,seeds,held,sat_frac_max,decays_all,rms_joint_err_median,final_pos_err_max,chosen";

fs::path out_path(const std::string& name) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "bench_tune_csv";
  fs::create_directories(dir);
  const fs::path p = dir / name;
  fs::remove(p);
  return p;
}

std::string read_text(const fs::path& p) {
  std::ifstream in(p, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

std::vector<std::string> split(const std::string& text, char sep) {
  std::vector<std::string> parts;
  std::stringstream stream(text);
  std::string part;
  while (std::getline(stream, part, sep)) {
    parts.push_back(part);
  }
  return parts;
}

TuneRow make_row(
    double level, std::size_t held, double sat, bool decays, double rms, double final_pos) {
  TuneRow r{};
  r.level = level;
  r.seeds = 3;
  r.held = held;
  r.sat_frac_max = sat;
  r.decays_all = decays;
  r.rms_joint_err_median = rms;
  r.final_pos_err_max = final_pos;
  return r;
}

TuneResult make_result(std::vector<TuneRow> rows, std::optional<double> chosen) {
  TuneResult result{};
  result.algorithm = control::Algorithm::kComputedTorque;
  result.key = "config.computed_torque.omega";
  for (const TuneRow& r : rows) {
    result.levels.push_back(r.level);
  }
  result.rows = std::move(rows);
  result.chosen = chosen;
  result.reason = "a reason";
  return result;
}

TEST(TuneCsv, HeaderAndExactLines) {
  const fs::path p = out_path("exact.csv");
  // Binary fractions, so %.17g gives the short text: 0.5, 0.25, 0.125 and 62.5.
  const TuneResult result = make_result(
      {make_row(50.0, 0, 0.5, true, 0.25, 0.125), make_row(62.5, 2, 0.125, false, 0.5, 0.25)},
      50.0);
  const common::Status st = write_tune_csv(p, result);
  ASSERT_TRUE(st.is_ok()) << st.reason();
  const std::string expected = std::string(kHeader) +
                               "\n"
                               "50,3,0,0.5,1,0.25,0.125,1\n"
                               "62.5,3,2,0.125,0,0.5,0.25,0\n";
  EXPECT_EQ(read_text(p), expected);
}

TEST(TuneCsv, ValuesRoundTrip) {
  const fs::path p = out_path("round_trip.csv");
  const double sat = 1.0 / 3.0;
  const double rms = 0.1 + 0.2;
  const double final_pos = 1.2345678901234567e-300;
  const double level = 33.333333333333336;
  ASSERT_TRUE(
      write_tune_csv(p, make_result({make_row(level, 1, sat, false, rms, final_pos)}, std::nullopt))
          .is_ok());
  const std::vector<std::string> lines = split(read_text(p), '\n');
  ASSERT_EQ(lines.size(), 2U);
  const std::vector<std::string> cells = split(lines[1], ',');
  ASSERT_EQ(cells.size(), 8U);
  EXPECT_EQ(std::strtod(cells[0].c_str(), nullptr), level);
  EXPECT_EQ(cells[1], "3");
  EXPECT_EQ(cells[2], "1");
  EXPECT_EQ(std::strtod(cells[3].c_str(), nullptr), sat);
  EXPECT_EQ(cells[4], "0");
  EXPECT_EQ(std::strtod(cells[5].c_str(), nullptr), rms);
  EXPECT_EQ(std::strtod(cells[6].c_str(), nullptr), final_pos);
  EXPECT_EQ(cells[7], "0");
}

TEST(TuneCsv, NanErrorColumnsReadBackAsNan) {
  const fs::path p = out_path("nan.csv");
  const double nan = std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(
      write_tune_csv(p, make_result({make_row(20.0, 3, 0.0, false, nan, nan)}, std::nullopt))
          .is_ok());
  const std::vector<std::string> lines = split(read_text(p), '\n');
  ASSERT_EQ(lines.size(), 2U);
  const std::vector<std::string> cells = split(lines[1], ',');
  ASSERT_EQ(cells.size(), 8U);
  EXPECT_TRUE(std::isnan(std::strtod(cells[5].c_str(), nullptr))) << cells[5];
  EXPECT_TRUE(std::isnan(std::strtod(cells[6].c_str(), nullptr))) << cells[6];
}

TEST(TuneCsv, ChosenColumnMarksOnlyTheChosenLevel) {
  const fs::path p = out_path("chosen.csv");
  const std::vector<TuneRow> rows{make_row(50.0, 0, 0.0, true, 0.25, 0.125),
                                  make_row(60.0, 0, 0.0, true, 0.125, 0.125),
                                  make_row(70.0, 0, 0.0, true, 0.5, 0.125)};
  ASSERT_TRUE(write_tune_csv(p, make_result(rows, 60.0)).is_ok());
  const std::vector<std::string> lines = split(read_text(p), '\n');
  ASSERT_EQ(lines.size(), 4U);
  EXPECT_EQ(split(lines[1], ',').back(), "0");
  EXPECT_EQ(split(lines[2], ',').back(), "1");
  EXPECT_EQ(split(lines[3], ',').back(), "0");

  ASSERT_TRUE(write_tune_csv(p, make_result(rows, std::nullopt)).is_ok());
  const std::vector<std::string> none = split(read_text(p), '\n');
  ASSERT_EQ(none.size(), 4U) << "a second write replaces the file";
  for (std::size_t i = 1; i < none.size(); ++i) {
    EXPECT_EQ(split(none[i], ',').back(), "0") << i;
  }
}

TEST(TuneCsv, AnUnwritablePathIsAnErrorNamingIt) {
  const fs::path file = out_path("a_file");
  std::ofstream(file) << "not a directory";
  const fs::path p = file / "tune.csv";
  const common::Status st = write_tune_csv(p, make_result({}, std::nullopt));
  ASSERT_FALSE(st.is_ok());
  EXPECT_NE(st.reason().find(p.string()), std::string::npos) << st.reason();
}

}  // namespace
}  // namespace cobot::bench
