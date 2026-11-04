// bridge::copy_csv_range (feature controller-bridge, plan ambiguity 6): a segment is a copy of
// rows already in the Run CSV. Header line copied byte for byte, then every row whose leading
// integer t_ns lies in [t0_ns, t1_ns], scanned from byte `offset` (0 or the start of a line);
// truncated when the first copied row is later than t0_ns + period_ns.
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "cobot/bridge/segment.hpp"

namespace cobot::bridge {
namespace {

namespace fs = std::filesystem;

constexpr std::int64_t kFirstT = 1000;
constexpr std::int64_t kLastT = 1999;
constexpr std::int64_t kPeriodNs = 1;  // one row per ns
const char* const kHeader = "t_ns,mode,x\n";

// "<t>,<mode>,<x>\n": 3 columns; the other fields differ per row so a misplaced row shows.
std::string row_text(std::int64_t t) {
  return std::to_string(t) + "," + (t % 2 == 0 ? "standby" : "tracking") + "," +
         std::to_string(t * 3) + ".25\n";
}

fs::path test_dir(const std::string& name) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "bridge_segment" / name;
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir;
}

std::string read_file(const fs::path& path) {
  const std::ifstream in(path, std::ios::binary);
  std::ostringstream os;
  os << in.rdbuf();
  return os.str();
}

// Header + rows t = 1000 .. 1999 (1000 rows). Returns the path.
fs::path write_run_csv(const fs::path& dir) {
  const fs::path csv = dir / "run.csv";
  std::ofstream out(csv, std::ios::binary);
  out << kHeader;
  for (std::int64_t t = kFirstT; t <= kLastT; ++t) {
    out << row_text(t);
  }
  return csv;
}

// Expected file content: the header then rows a .. b.
std::string expected_segment(std::int64_t a, std::int64_t b) {
  std::string s = kHeader;
  for (std::int64_t t = a; t <= b; ++t) {
    s += row_text(t);
  }
  return s;
}

// Byte offset of the line of row t in the file written by write_run_csv.
std::uint64_t offset_of_row(std::int64_t t) {
  std::uint64_t off = std::string(kHeader).size();
  for (std::int64_t k = kFirstT; k < t; ++k) {
    off += row_text(k).size();
  }
  return off;
}

TEST(BridgeSegment, CopiesTheRowsInsideTheWindowWithTheHeader) {
  const fs::path dir = test_dir("inside");
  const fs::path csv = write_run_csv(dir);
  const fs::path out = dir / "segment.csv";
  const auto r = copy_csv_range(csv, 0, 1100, 1199, kPeriodNs, out);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  // 1199 - 1100 + 1 = 100 rows.
  EXPECT_EQ(r.value().rows, 100U);
  EXPECT_EQ(r.value().first_t_ns, 1100);
  EXPECT_EQ(r.value().last_t_ns, 1199);
  // First row 1100 is not later than t0 + period = 1101.
  EXPECT_FALSE(r.value().truncated);
  const std::string got = read_file(out);
  ASSERT_GE(got.size(), std::string(kHeader).size());
  EXPECT_EQ(got.substr(0, std::string(kHeader).size()), kHeader) << "header byte-identical";
  EXPECT_EQ(got, expected_segment(1100, 1199));
}

TEST(BridgeSegment, WindowStartingBeforeTheFirstRowIsTruncated) {
  const fs::path dir = test_dir("before");
  const fs::path csv = write_run_csv(dir);
  const fs::path out = dir / "segment.csv";
  const auto r = copy_csv_range(csv, 0, 900, 1050, kPeriodNs, out);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  // Rows 1000 .. 1050: 1050 - 1000 + 1 = 51.
  EXPECT_EQ(r.value().rows, 51U);
  EXPECT_EQ(r.value().first_t_ns, 1000);
  EXPECT_EQ(r.value().last_t_ns, 1050);
  // First row 1000 > t0 + period = 900 + 1 = 901: rows before it were lost.
  EXPECT_TRUE(r.value().truncated);
  EXPECT_EQ(read_file(out), expected_segment(1000, 1050));
}

TEST(BridgeSegment, WindowStartingAtTheFirstRowIsNotTruncated) {
  const fs::path dir = test_dir("at_first");
  const fs::path csv = write_run_csv(dir);
  const fs::path out = dir / "segment.csv";
  const auto r = copy_csv_range(csv, 0, 1000, 1050, kPeriodNs, out);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().rows, 51U);
  EXPECT_EQ(r.value().first_t_ns, 1000);
  // 1000 <= 1000 + 1.
  EXPECT_FALSE(r.value().truncated);
}

TEST(BridgeSegment, OffsetSkipsTheRowsBeforeIt) {
  const fs::path dir = test_dir("offset");
  const fs::path csv = write_run_csv(dir);
  const fs::path out = dir / "segment.csv";
  const std::uint64_t offset = offset_of_row(1500);
  {
    // The offset points at the line of row 1500.
    const std::string all = read_file(csv);
    ASSERT_EQ(all.compare(offset, row_text(1500).size(), row_text(1500)), 0);
  }
  const auto r = copy_csv_range(csv, offset, 1100, 1600, kPeriodNs, out);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  // Rows 1500 .. 1600: 101 (1100 .. 1499 lie before the offset and are not scanned).
  EXPECT_EQ(r.value().rows, 101U);
  EXPECT_EQ(r.value().first_t_ns, 1500);
  EXPECT_EQ(r.value().last_t_ns, 1600);
  // 1500 > 1100 + 1.
  EXPECT_TRUE(r.value().truncated);
  // The header still comes from the start of the file.
  EXPECT_EQ(read_file(out), expected_segment(1500, 1600));
}

TEST(BridgeSegment, BoundsAreInclusiveAtTheLastRow) {
  const fs::path dir = test_dir("last");
  const fs::path csv = write_run_csv(dir);
  const fs::path out = dir / "segment.csv";
  const auto r = copy_csv_range(csv, 0, 1999, 5000, kPeriodNs, out);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().rows, 1U);
  EXPECT_EQ(r.value().first_t_ns, 1999);
  EXPECT_EQ(r.value().last_t_ns, 1999);
  EXPECT_FALSE(r.value().truncated);
  EXPECT_EQ(read_file(out), expected_segment(1999, 1999));
}

TEST(BridgeSegment, ExistingOutputIsReplaced) {
  const fs::path dir = test_dir("replace");
  const fs::path csv = write_run_csv(dir);
  const fs::path out = dir / "segment.csv";
  {
    std::ofstream old(out, std::ios::binary);
    old << "old content that must disappear\n";
  }
  const auto r = copy_csv_range(csv, 0, 1200, 1201, kPeriodNs, out);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(read_file(out), expected_segment(1200, 1201));
}

TEST(BridgeSegment, NoRowInTheWindowIsAnError) {
  const fs::path dir = test_dir("empty");
  const fs::path csv = write_run_csv(dir);
  const auto r = copy_csv_range(csv, 0, 3000, 4000, kPeriodNs, dir / "segment.csv");
  ASSERT_FALSE(r.is_ok());
  EXPECT_NE(r.reason().find("no row"), std::string::npos) << r.reason();
}

TEST(BridgeSegment, MissingRunCsvIsAnErrorNamingIt) {
  const fs::path dir = test_dir("missing");
  const fs::path csv = dir / "does_not_exist.csv";
  const auto r = copy_csv_range(csv, 0, 1000, 2000, kPeriodNs, dir / "segment.csv");
  ASSERT_FALSE(r.is_ok());
  EXPECT_NE(r.reason().find(csv.string()), std::string::npos) << r.reason();
}

}  // namespace
}  // namespace cobot::bridge
