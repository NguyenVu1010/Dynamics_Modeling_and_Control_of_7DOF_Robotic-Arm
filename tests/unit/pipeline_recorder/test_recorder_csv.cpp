#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/result.hpp"
#include "cobot/pipeline/log_record.hpp"
#include "cobot/pipeline/recorder.hpp"

#include "recorder_test_util.hpp"

// CSV export of recorded rows (feature pipeline-recorder, design 6.1): write_csv and export_csv
// write exactly what write_csv_header / write_csv_row write (Run CSV contract,
// docs/conventions.md), rows oldest first; export_csv writes the header only into a new or empty
// file.
namespace cobot::pipeline::test {
namespace {

namespace fs = std::filesystem;

std::string header_text() {
  std::ostringstream os;
  write_csv_header<kN>(os);
  return os.str();
}

// write_csv_row of make_record(t) for each t, in that order.
std::string rows_text(const std::vector<std::int64_t>& ts) {
  std::ostringstream os;
  for (const std::int64_t t : ts) {
    write_csv_row<kN>(os, make_record(t));
  }
  return os.str();
}

std::string read_file(const fs::path& p) {
  const std::ifstream in(p, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

fs::path out_dir() {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "pipeline_recorder";
  fs::create_directories(dir);
  return dir;
}

// A fresh path in out_dir() (any file from an earlier run removed).
fs::path fresh(const std::string& name) {
  const fs::path p = out_dir() / name;
  fs::remove(p);
  return p;
}

void record_range(Recorder<kN>& rec, std::int64_t first, std::int64_t last) {
  for (std::int64_t t = first; t <= last; ++t) {
    rec.record(make_record(t));
  }
}

TEST(RecorderCsv, WriteCsvIsTheHeaderPlusOneContractRowPerRecord) {
  HeapStorage<kN> storage(8, 1);
  Recorder<kN> rec(storage.memory(), FullPolicy::kStopWhenFull);
  record_range(rec, 0, 4);
  std::ostringstream with_header;
  write_csv<kN>(with_header, rec.active(), true);
  EXPECT_EQ(with_header.str(), header_text() + rows_text(range(0, 4)));
  std::ostringstream without_header;
  write_csv<kN>(without_header, rec.active(), false);
  EXPECT_EQ(without_header.str(), rows_text(range(0, 4)));
}

TEST(RecorderCsv, EmptySpanWritesOnlyTheHeader) {
  std::ostringstream os;
  write_csv<kN>(os, Span{}, true);
  EXPECT_EQ(os.str(), header_text());
  std::ostringstream none;
  write_csv<kN>(none, Span{}, false);
  EXPECT_EQ(none.str(), "");
}

// Capacity 3, kOverwriteOldest, t = 0..4: the wrapped buffer keeps t = 2, 3, 4 and writes them
// in that order.
TEST(RecorderCsv, WrappedSpanIsWrittenOldestFirst) {
  HeapStorage<kN> storage(3, 1);
  Recorder<kN> rec(storage.memory(), FullPolicy::kOverwriteOldest);
  record_range(rec, 0, 4);
  ASSERT_GT(rec.active().second_size, 0U);
  std::ostringstream os;
  write_csv<kN>(os, rec.active(), true);
  EXPECT_EQ(os.str(), header_text() + rows_text({2, 3, 4}));
}

TEST(RecorderCsv, ExportToANewFileWritesTheHeader) {
  HeapStorage<kN> storage(8, 1);
  Recorder<kN> rec(storage.memory(), FullPolicy::kStopWhenFull);
  record_range(rec, 0, 2);
  const fs::path a = fresh("new_append.csv");
  const common::Status sa = export_csv<kN>(a, rec.active(), true);
  ASSERT_TRUE(sa.is_ok()) << sa.reason();
  EXPECT_EQ(read_file(a), header_text() + rows_text(range(0, 2)));
  const fs::path b = fresh("new_truncate.csv");
  const common::Status sb = export_csv<kN>(b, rec.active(), false);
  ASSERT_TRUE(sb.is_ok()) << sb.reason();
  EXPECT_EQ(read_file(b), header_text() + rows_text(range(0, 2)));
}

// The ROS node appends each retired buffer to run.csv: one header, then every row in order.
TEST(RecorderCsv, AppendToANonEmptyFileWritesNoSecondHeader) {
  HeapStorage<kN> storage(4, 2);
  Recorder<kN> rec(storage.memory(), FullPolicy::kOverwriteOldest);
  const fs::path p = fresh("append.csv");
  record_range(rec, 0, 2);
  ASSERT_TRUE(rec.request_swap());
  rec.record(make_record(3));
  ASSERT_TRUE(rec.swap_done());
  const common::Status s1 = export_csv<kN>(p, rec.retired(), true);
  ASSERT_TRUE(s1.is_ok()) << s1.reason();
  rec.release_retired();
  record_range(rec, 4, 5);
  const common::Status s2 = export_csv<kN>(p, rec.active(), true);
  ASSERT_TRUE(s2.is_ok()) << s2.reason();
  EXPECT_EQ(read_file(p), header_text() + rows_text(range(0, 5)));
}

// An existing empty file counts as new: append writes the header first.
TEST(RecorderCsv, AppendToAnEmptyFileWritesTheHeader) {
  HeapStorage<kN> storage(4, 1);
  Recorder<kN> rec(storage.memory(), FullPolicy::kStopWhenFull);
  record_range(rec, 0, 1);
  const fs::path p = fresh("empty.csv");
  std::ofstream(p, std::ios::binary).close();
  ASSERT_TRUE(fs::exists(p));
  const common::Status s = export_csv<kN>(p, rec.active(), true);
  ASSERT_TRUE(s.is_ok()) << s.reason();
  EXPECT_EQ(read_file(p), header_text() + rows_text(range(0, 1)));
}

// append = false replaces the file: header and the rows, nothing of the old content.
TEST(RecorderCsv, ExportWithoutAppendReplacesTheFile) {
  HeapStorage<kN> storage(4, 1);
  Recorder<kN> rec(storage.memory(), FullPolicy::kStopWhenFull);
  record_range(rec, 7, 8);
  const fs::path p = fresh("replace.csv");
  std::ofstream(p, std::ios::binary) << "old content\n";
  const common::Status s = export_csv<kN>(p, rec.active(), false);
  ASSERT_TRUE(s.is_ok()) << s.reason();
  EXPECT_EQ(read_file(p), header_text() + rows_text({7, 8}));
}

// A path under a regular file can be neither opened nor created (also as root): the error
// names the path.
TEST(RecorderCsv, UnwritablePathIsAnErrorNamingIt) {
  HeapStorage<kN> storage(4, 1);
  Recorder<kN> rec(storage.memory(), FullPolicy::kStopWhenFull);
  record_range(rec, 0, 1);
  const fs::path blocker = out_dir() / "not_a_directory";
  fs::remove_all(blocker);
  std::ofstream(blocker, std::ios::binary) << "x\n";
  const fs::path p = blocker / "run.csv";
  for (const bool append : {false, true}) {
    const common::Status s = export_csv<kN>(p, rec.active(), append);
    ASSERT_FALSE(s.is_ok()) << "append " << append;
    EXPECT_NE(s.reason().find(p.string()), std::string::npos) << s.reason();
  }
}

}  // namespace
}  // namespace cobot::pipeline::test
