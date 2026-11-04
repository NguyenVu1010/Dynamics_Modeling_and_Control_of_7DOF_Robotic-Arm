// cobot_logdump <controller> [--out <csv>] [--keep] [--remove] (feature blackbox-log, cobot_studio
// design 6.1 "Hộp đen"): exports the black-box region "/cobot_log_<controller>" to a Run CSV.
// Prints "<rows> rows -> <csv>" (plus ", <lost> rows lost" when records were overwritten before
// any export); default out "./<controller>-<YYYYmmdd-HHMMSS>.csv"; exit 0 ok, 1 no region or
// nothing to export, 2 bad arguments or layout mismatch (reason on stderr).
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <string>
#include <sys/wait.h>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/pipeline/recorder.hpp"
#include "cobot/pipeline/recorder_shm.hpp"

#include "shm_test_util.hpp"

namespace cobot::pipeline::shm_test {
namespace {

const char* const kExe = COBOT_LOGDUMP_EXE;

struct Cli {
  int exit_code = -1;
  std::string out;  // stdout
  std::string err;  // stderr
};

std::string quoted(const std::string& s) {
  return "'" + s + "'";
}

// Runs cobot_logdump with `args` in the working directory `cwd`.
Cli run_logdump(const std::string& args, const fs::path& cwd) {
  Cli cli;
  if (std::string(kExe).empty()) {
    ADD_FAILURE() << "cobot_logdump is not built (no target cobot_logdump)";
    return cli;
  }
  const fs::path err = cwd / "stderr.txt";
  const std::string cmd = "cd " + quoted(cwd.string()) + " && " + quoted(std::string(kExe)) + " " +
                          args + " 2>" + quoted(err.string());
  FILE* pipe = popen(cmd.c_str(), "r");
  if (pipe == nullptr) {
    ADD_FAILURE() << "popen failed: " << cmd;
    return cli;
  }
  std::array<char, 4096> buf{};
  std::size_t n = 0;
  while ((n = std::fread(buf.data(), 1, buf.size(), pipe)) > 0) {
    cli.out.append(buf.data(), n);
  }
  const int status = pclose(pipe);
  cli.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  cli.err = read_file(err);
  fs::remove(err);
  return cli;
}

// A region "/cobot_log_<name>" of `capacity` holding records t = 1 .. last, storage unmapped.
void fill_region(const std::string& name, std::size_t capacity, std::int64_t last) {
  auto created = Storage::create(name, capacity, false);
  ASSERT_TRUE(created.is_ok()) << created.reason();
  Recorder<kDof> rec(created.value().memory(), FullPolicy::kOverwriteOldest);
  for (std::int64_t t = 1; t <= last; ++t) {
    rec.record(make_record(t));
  }
}

fs::path cli_dir(const std::string& test) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "logdump_cli" / test;
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir;
}

bool contains(const std::string& text, const std::string& part) {
  return text.find(part) != std::string::npos;
}

TEST(LogdumpCli, ExportsTheRegionAndPrintsRowsAndPath) {
  const RegionGuard g;
  const fs::path dir = cli_dir("export");
  fill_region(g.name(), 100, 10);
  const fs::path csv = dir / "log.csv";
  const Cli cli = run_logdump(g.name() + " --out " + quoted(csv.string()), dir);
  EXPECT_EQ(cli.exit_code, 0) << cli.out << cli.err;
  EXPECT_TRUE(contains(cli.out, "10 rows -> " + csv.string())) << cli.out;
  EXPECT_FALSE(contains(cli.out, "lost")) << cli.out;
  const CsvFile f = read_csv(csv);
  EXPECT_EQ(f.header_lines, 1);
  EXPECT_EQ(csv_stamps(f), range(1, 10));
  expect_rows_well_formed(f, true);
  EXPECT_TRUE(g.present()) << "the region stays without --remove";
}

TEST(LogdumpCli, SecondRunHasNothingToExportAndExitsOne) {
  const RegionGuard g;
  const fs::path dir = cli_dir("second");
  fill_region(g.name(), 100, 10);
  const Cli first = run_logdump(g.name() + " --out " + quoted((dir / "a.csv").string()), dir);
  ASSERT_EQ(first.exit_code, 0) << first.out << first.err;
  const Cli second = run_logdump(g.name() + " --out " + quoted((dir / "b.csv").string()), dir);
  EXPECT_EQ(second.exit_code, 1) << second.out << second.err;
  if (fs::exists(dir / "b.csv")) {
    EXPECT_TRUE(read_csv(dir / "b.csv").rows.empty());
  }
}

TEST(LogdumpCli, KeepLeavesTheRowsForTheNextRun) {
  const RegionGuard g;
  const fs::path dir = cli_dir("keep");
  fill_region(g.name(), 100, 12);
  for (const char* file : {"k1.csv", "k2.csv"}) {
    const fs::path csv = dir / file;
    const Cli cli = run_logdump(g.name() + " --out " + quoted(csv.string()) + " --keep", dir);
    EXPECT_EQ(cli.exit_code, 0) << file << ": " << cli.out << cli.err;
    EXPECT_TRUE(contains(cli.out, "12 rows -> " + csv.string())) << cli.out;
    EXPECT_EQ(csv_stamps(read_csv(csv)), range(1, 12)) << file;
  }
  const Cli take = run_logdump(g.name() + " --out " + quoted((dir / "k3.csv").string()), dir);
  EXPECT_EQ(take.exit_code, 0) << take.out << take.err;
  const Cli none = run_logdump(g.name() + " --out " + quoted((dir / "k4.csv").string()), dir);
  EXPECT_EQ(none.exit_code, 1) << none.out << none.err;
}

// Capacity 10 and 25 records: rows 17 .. 25 (9 = capacity - 1, the slot the next record() would
// overwrite is skipped), 25 - 9 = 16 records lost.
TEST(LogdumpCli, WrappedRegionPrintsLostRows) {
  const RegionGuard g;
  const fs::path dir = cli_dir("lost");
  fill_region(g.name(), 10, 25);
  const fs::path csv = dir / "log.csv";
  const Cli cli = run_logdump(g.name() + " --out " + quoted(csv.string()), dir);
  EXPECT_EQ(cli.exit_code, 0) << cli.out << cli.err;
  EXPECT_TRUE(contains(cli.out, "9 rows -> " + csv.string() + ", 16 rows lost")) << cli.out;
  EXPECT_EQ(csv_stamps(read_csv(csv)), range(17, 25));
}

TEST(LogdumpCli, DefaultOutIsControllerAndTimeInTheWorkingDirectory) {
  const RegionGuard g;
  const fs::path dir = cli_dir("default_out");
  fill_region(g.name(), 100, 7);
  const Cli cli = run_logdump(g.name(), dir);
  EXPECT_EQ(cli.exit_code, 0) << cli.out << cli.err;
  std::vector<fs::path> csvs;
  for (const auto& e : fs::directory_iterator(dir)) {
    if (e.path().extension() == ".csv") {
      csvs.push_back(e.path());
    }
  }
  ASSERT_EQ(csvs.size(), 1U);
  const std::string file = csvs[0].filename().string();
  EXPECT_TRUE(is_stamped_name(file, g.name() + "-", ".csv")) << file;
  EXPECT_TRUE(contains(cli.out, "7 rows -> ")) << cli.out;
  EXPECT_TRUE(contains(cli.out, file)) << cli.out;
  EXPECT_EQ(csv_stamps(read_csv(csvs[0])), range(1, 7));
}

TEST(LogdumpCli, RemoveLeavesNoRegion) {
  const RegionGuard g;
  const fs::path dir = cli_dir("remove");
  fill_region(g.name(), 100, 5);
  const fs::path csv = dir / "log.csv";
  const Cli cli = run_logdump(g.name() + " --out " + quoted(csv.string()) + " --remove", dir);
  EXPECT_EQ(cli.exit_code, 0) << cli.out << cli.err;
  EXPECT_TRUE(contains(cli.out, "5 rows -> " + csv.string())) << cli.out;
  EXPECT_EQ(csv_stamps(read_csv(csv)), range(1, 5));
  EXPECT_FALSE(Storage::exists(g.name()));
  EXPECT_FALSE(g.present());
}

TEST(LogdumpCli, MissingRegionExitsOne) {
  const RegionGuard g;
  const fs::path dir = cli_dir("missing");
  const fs::path csv = dir / "log.csv";
  const Cli cli = run_logdump(g.name() + " --out " + quoted(csv.string()), dir);
  EXPECT_EQ(cli.exit_code, 1) << cli.out << cli.err;
  EXPECT_FALSE(cli.err.empty()) << "reason on stderr";
  EXPECT_FALSE(g.present());
}

// A region whose fingerprint differs from this build's LogRecord<7>: exit 2, reason on stderr,
// and the region is left for a matching build.
TEST(LogdumpCli, MismatchedLayoutExitsTwo) {
  const RegionGuard g;
  const fs::path dir = cli_dir("mismatch");
  fill_region(g.name(), 37, 5);
  {
    RawRegion raw(g.name());
    ASSERT_TRUE(raw.ok());
    const auto at = raw.find(log_record_fingerprint<kDof>(), 0, 4096, 4);
    ASSERT_TRUE(at) << "fingerprint not found in the header";
    raw.write(*at, ~log_record_fingerprint<kDof>());
  }
  const fs::path csv = dir / "log.csv";
  const Cli cli = run_logdump(g.name() + " --out " + quoted(csv.string()), dir);
  EXPECT_EQ(cli.exit_code, 2) << cli.out << cli.err;
  EXPECT_TRUE(contains(cli.err, "fingerprint")) << cli.err;
  EXPECT_TRUE(g.present());
}

TEST(LogdumpCli, BadArgumentsExitTwo) {
  const RegionGuard g;
  const fs::path dir = cli_dir("bad_args");
  fill_region(g.name(), 16, 3);
  const std::vector<std::string> bad = {
      "",                          // no controller
      g.name() + " --frobnicate",  // unknown option
      g.name() + " --out",         // --out without a value
  };
  for (const std::string& args : bad) {
    const Cli cli = run_logdump(args, dir);
    EXPECT_EQ(cli.exit_code, 2) << "args '" << args << "': " << cli.out << cli.err;
    EXPECT_FALSE(cli.err.empty()) << "args '" << args << "'";
  }
  // Nothing was exported by the rejected calls.
  const Cli ok = run_logdump(g.name() + " --out " + quoted((dir / "log.csv").string()), dir);
  EXPECT_EQ(ok.exit_code, 0) << ok.out << ok.err;
  EXPECT_TRUE(contains(ok.out, "3 rows -> ")) << ok.out;
}

}  // namespace
}  // namespace cobot::pipeline::shm_test
