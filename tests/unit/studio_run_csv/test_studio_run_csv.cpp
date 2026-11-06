// The Run CSV read back by the studio (feature studio-control, cobot/studio/run_csv.hpp, the Run
// CSV contract of docs/conventions.md). Pinned:
//   - a 1 s default + ideal bench run (run_once) written by bench::write_run_files reads back
//     with the same row count; every double of the record is bit-equal (written with %.17g, NaN
//     as `nan`), T_b_e / T_b_d positions bit-equal and their rotations within 1e-12 of the
//     record after so3_exp of the logged rotation vector; mode, hold and cmd strings map back to
//     every enum value (rows edited in a copy of the file);
//   - the legacy layout (the first 57 columns of every line, ending with `hold`): truth columns
//     NaN, qraw = q, qddref 0, cmd config;
//   - errors name the file, the 1-based line (a row with 113 fields) and the column (`mode`
//     "flying").
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/result.hpp"
#include "cobot/common/types.hpp"
#include "cobot/pipeline/log_record.hpp"
#include "cobot/studio/profile_catalog.hpp"
#include "cobot/studio/run_csv.hpp"
#include "cobot/studio/run_store.hpp"
#include "cobot/studio/runner.hpp"
#include "cobot/studio/studio_config.hpp"

#include "metrics.hpp"
#include "run_data.hpp"

namespace cobot::studio {
namespace {

namespace fs = std::filesystem;
using Record = pipeline::LogRecord<bench::kN>;
using Records = std::vector<Record>;

const fs::path kRepo = COBOT_REPO_DIR;
constexpr std::size_t kColumns = 114;  // 57 + 8 * 7 + 1 (docs/conventions.md)
constexpr std::size_t kLegacyColumns = 57;

fs::path work_dir(const std::string& name) {
  const fs::path dir = fs::path(COBOT_TEST_OUTPUT_DIR) / "studio_run_csv" / name;
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir;
}

std::string read_text(const fs::path& p) {
  std::ifstream in(p, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

void write_text(const fs::path& p, const std::string& text) {
  std::ofstream(p, std::ios::binary) << text;
}

std::vector<std::string> lines_of(const std::string& text) {
  std::vector<std::string> lines;
  std::istringstream in(text);
  for (std::string line; std::getline(in, line);) {
    lines.push_back(line);
  }
  return lines;
}

std::string join_lines(const std::vector<std::string>& lines) {
  std::string out;
  for (const std::string& l : lines) {
    out += l;
    out += '\n';
  }
  return out;
}

std::vector<std::string> fields_of(const std::string& line) {
  std::vector<std::string> fields;
  std::size_t start = 0;
  for (;;) {
    const std::size_t comma = line.find(',', start);
    fields.push_back(line.substr(start, comma == std::string::npos ? comma : comma - start));
    if (comma == std::string::npos) {
      return fields;
    }
    start = comma + 1;
  }
}

std::string join_fields(const std::vector<std::string>& fields) {
  std::string out;
  for (std::size_t i = 0; i < fields.size(); ++i) {
    out += (i == 0 ? "" : ",") + fields[i];
  }
  return out;
}

// Index of a header column; npos when absent.
std::size_t column(const std::string& header, const std::string& name) {
  const std::vector<std::string> names = fields_of(header);
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (names[i] == name) {
      return i;
    }
  }
  return std::string::npos;
}

// The same double: equal bits, or both NaN (the file writes every NaN as `nan`).
bool same_bits(double a, double b) {
  if (std::isnan(a) && std::isnan(b)) {
    return true;
  }
  std::uint64_t bits_a = 0;
  std::uint64_t bits_b = 0;
  std::memcpy(&bits_a, &a, sizeof(double));
  std::memcpy(&bits_b, &b, sizeof(double));
  return bits_a == bits_b;
}

template <typename Derived>
void expect_same_bits(const Eigen::MatrixBase<Derived>& got,
                      const Eigen::MatrixBase<Derived>& want,
                      const char* what,
                      std::size_t row) {
  for (Eigen::Index i = 0; i < want.size(); ++i) {
    EXPECT_TRUE(same_bits(got(i), want(i)))
        << what << "[" << i << "] row " << row << ": " << got(i) << " vs " << want(i);
  }
}

void expect_pose(const common::Pose& got,
                 const common::Pose& want,
                 const char* what,
                 std::size_t row) {
  expect_same_bits(got.p, want.p, what, row);
  // R back from so3_exp(so3_log(R)): not bit-equal, within 1e-12 element-wise.
  EXPECT_LT((got.R - want.R).cwiseAbs().maxCoeff(), 1e-12) << what << ".R row " << row;
}

// A 1 s run of config/default.yaml + the ideal profile (1000 rows at 1 kHz), written to `csv`.
std::optional<bench::RunData> written_run(const fs::path& dir, const fs::path& csv) {
  StudioConfig config;
  config.base =
      BasePair{"ideal", kRepo / "config/default.yaml", kRepo / "config/imperfections/ideal.yaml"};
  config.duration_s = 1.0;
  common::Result<RunEntry> entry = run_once(config, dir / "work");
  if (!entry) {
    ADD_FAILURE() << entry.reason();
    return std::nullopt;
  }
  if (!entry.value().data) {
    ADD_FAILURE() << "run_once kept no records";
    return std::nullopt;
  }
  const common::Status written = bench::write_run_files(*entry.value().data, csv);
  if (!written) {
    ADD_FAILURE() << written.reason();
    return std::nullopt;
  }
  return *entry.value().data;
}

TEST(StudioRunCsv, BenchRunReadsBackBitForBit) {
  const fs::path dir = work_dir("round_trip");
  const fs::path csv = dir / "run.csv";
  const std::optional<bench::RunData> run = written_run(dir, csv);
  ASSERT_TRUE(run);
  ASSERT_EQ(run->records.size(), 1000U) << "1 s at 1 kHz";
  ASSERT_EQ(fields_of(lines_of(read_text(csv)).front()).size(), kColumns);

  const common::Result<Records> read = read_run_csv(csv);
  ASSERT_TRUE(read.is_ok()) << read.reason();
  const Records& got = read.value();
  ASSERT_EQ(got.size(), run->records.size());
  for (std::size_t k = 0; k < got.size(); ++k) {
    const Record& a = got[k];
    const Record& b = run->records[k];
    EXPECT_EQ(a.t.ns, b.t.ns) << "row " << k;
    EXPECT_EQ(a.mode, b.mode) << "row " << k;
    EXPECT_EQ(a.hold_reason, b.hold_reason) << "row " << k;
    EXPECT_EQ(a.cmd_mode, b.cmd_mode) << "row " << k;
    expect_same_bits(a.q, b.q, "q", k);
    expect_same_bits(a.qd, b.qd, "qd", k);
    expect_same_bits(a.q_ref, b.q_ref, "q_ref", k);
    expect_same_bits(a.qd_ref, b.qd_ref, "qd_ref", k);
    expect_same_bits(a.tau, b.tau, "tau", k);
    expect_same_bits(a.task_error, b.task_error, "task_error", k);
    EXPECT_TRUE(same_bits(a.step_time_us, b.step_time_us)) << "step_us row " << k;
    expect_same_bits(a.qdd_ref, b.qdd_ref, "qdd_ref", k);
    expect_same_bits(a.q_raw, b.q_raw, "q_raw", k);
    expect_same_bits(a.q_true, b.q_true, "q_true", k);
    expect_same_bits(a.qd_true, b.qd_true, "qd_true", k);
    expect_same_bits(a.qdd_true, b.qdd_true, "qdd_true", k);
    expect_same_bits(a.q_motor_true, b.q_motor_true, "q_motor_true", k);
    expect_same_bits(a.tau_motor, b.tau_motor, "tau_motor", k);
    expect_same_bits(a.tau_friction, b.tau_friction, "tau_friction", k);
    expect_pose(a.T_b_e, b.T_b_e, "T_b_e", k);
    expect_pose(a.T_b_d, b.T_b_d, "T_b_d", k);
    if (::testing::Test::HasFailure()) {
      break;  // one row is enough to see what differs
    }
  }
}

// Every mode, hold and cmd string of log_record.hpp, written into rows of a copy of the file.
TEST(StudioRunCsv, EnumStringsMapBackToEveryValue) {
  const fs::path dir = work_dir("enums");
  const fs::path csv = dir / "run.csv";
  ASSERT_TRUE(written_run(dir, csv));
  std::vector<std::string> lines = lines_of(read_text(csv));
  ASSERT_GT(lines.size(), 10U);
  const std::size_t mode_col = column(lines.front(), "mode");
  const std::size_t hold_col = column(lines.front(), "hold");
  const std::size_t cmd_col = column(lines.front(), "cmd");
  ASSERT_EQ(mode_col, 1U);
  ASSERT_EQ(hold_col, 56U);
  ASSERT_EQ(cmd_col, 113U);

  const std::vector<pipeline::LoopMode> modes = {pipeline::LoopMode::kIdle,
                                                 pipeline::LoopMode::kTracking,
                                                 pipeline::LoopMode::kHolding,
                                                 pipeline::LoopMode::kFinished,
                                                 pipeline::LoopMode::kFault,
                                                 pipeline::LoopMode::kStandby};
  const std::vector<pipeline::HoldReason> holds = {pipeline::HoldReason::kNone,
                                                   pipeline::HoldReason::kNonFinite,
                                                   pipeline::HoldReason::kStampBackwards,
                                                   pipeline::HoldReason::kStateTimeout,
                                                   pipeline::HoldReason::kJointLimit,
                                                   pipeline::HoldReason::kOverspeed,
                                                   pipeline::HoldReason::kEstop};
  const std::vector<pipeline::CommandMode> cmds = {pipeline::CommandMode::kConfig,
                                                   pipeline::CommandMode::kWaypoints,
                                                   pipeline::CommandMode::kJogJoint,
                                                   pipeline::CommandMode::kJogTcp,
                                                   pipeline::CommandMode::kMoveJoint,
                                                   pipeline::CommandMode::kMoveTcp};
  // Data row i is line i + 1 (line 0 is the header).
  for (std::size_t i = 0; i < 7; ++i) {
    std::vector<std::string> f = fields_of(lines[i + 1]);
    ASSERT_EQ(f.size(), kColumns);
    if (i < modes.size()) {
      f[mode_col] = pipeline::to_string(modes[i]);
    }
    f[hold_col] = pipeline::to_string(holds[i]);
    if (i < cmds.size()) {
      f[cmd_col] = pipeline::to_string(cmds[i]);
    }
    lines[i + 1] = join_fields(f);
  }
  const fs::path edited = dir / "edited.csv";
  write_text(edited, join_lines(lines));

  const common::Result<Records> read = read_run_csv(edited);
  ASSERT_TRUE(read.is_ok()) << read.reason();
  ASSERT_EQ(read.value().size(), lines.size() - 1);
  for (std::size_t i = 0; i < modes.size(); ++i) {
    EXPECT_EQ(read.value()[i].mode, modes[i]) << pipeline::to_string(modes[i]);
  }
  for (std::size_t i = 0; i < holds.size(); ++i) {
    EXPECT_EQ(read.value()[i].hold_reason, holds[i]) << pipeline::to_string(holds[i]);
  }
  for (std::size_t i = 0; i < cmds.size(); ++i) {
    EXPECT_EQ(read.value()[i].cmd_mode, cmds[i]) << pipeline::to_string(cmds[i]);
  }
}

// Files written before feature log-channels: the first 57 columns, ending with `hold`.
TEST(StudioRunCsv, LegacyLayoutFillsTheMissingColumns) {
  const fs::path dir = work_dir("legacy");
  const fs::path csv = dir / "run.csv";
  const std::optional<bench::RunData> run = written_run(dir, csv);
  ASSERT_TRUE(run);
  std::vector<std::string> lines = lines_of(read_text(csv));
  for (std::string& line : lines) {
    std::vector<std::string> f = fields_of(line);
    ASSERT_EQ(f.size(), kColumns);
    f.resize(kLegacyColumns);
    line = join_fields(f);
  }
  ASSERT_EQ(fields_of(lines.front()).back(), "hold");
  const fs::path legacy = dir / "legacy.csv";
  write_text(legacy, join_lines(lines));

  const common::Result<Records> read = read_run_csv(legacy);
  ASSERT_TRUE(read.is_ok()) << read.reason();
  ASSERT_EQ(read.value().size(), run->records.size());
  for (std::size_t k = 0; k < read.value().size(); ++k) {
    const Record& a = read.value()[k];
    const Record& b = run->records[k];
    EXPECT_EQ(a.t.ns, b.t.ns) << "row " << k;
    EXPECT_EQ(a.mode, b.mode) << "row " << k;
    EXPECT_EQ(a.hold_reason, b.hold_reason) << "row " << k;
    expect_same_bits(a.q, b.q, "q", k);
    expect_same_bits(a.tau, b.tau, "tau", k);
    expect_pose(a.T_b_e, b.T_b_e, "T_b_e", k);
    // Not in the file: truth unknown (NaN), qraw = q, qddref 0, cmd config.
    EXPECT_TRUE(a.q_true.array().isNaN().all()) << "row " << k;
    EXPECT_TRUE(a.qd_true.array().isNaN().all()) << "row " << k;
    EXPECT_TRUE(a.qdd_true.array().isNaN().all()) << "row " << k;
    EXPECT_TRUE(a.q_motor_true.array().isNaN().all()) << "row " << k;
    EXPECT_TRUE(a.tau_motor.array().isNaN().all()) << "row " << k;
    EXPECT_TRUE(a.tau_friction.array().isNaN().all()) << "row " << k;
    expect_same_bits(a.q_raw, a.q, "q_raw (= q)", k);
    EXPECT_TRUE((a.qdd_ref.array() == 0.0).all()) << "row " << k;
    EXPECT_EQ(a.cmd_mode, pipeline::CommandMode::kConfig) << "row " << k;
    if (::testing::Test::HasFailure()) {
      break;
    }
  }
}

// A data row with 113 fields (its last one dropped): the error names the file and the line.
// Data row 5 is line 7 of the file (1-based, the header is line 1).
TEST(StudioRunCsv, RowWithAMissingFieldNamesTheLine) {
  const fs::path dir = work_dir("short_row");
  const fs::path csv = dir / "run.csv";
  ASSERT_TRUE(written_run(dir, csv));
  std::vector<std::string> lines = lines_of(read_text(csv));
  std::vector<std::string> f = fields_of(lines[6]);
  ASSERT_EQ(f.size(), kColumns);
  f.pop_back();
  ASSERT_EQ(f.size(), 113U);
  lines[6] = join_fields(f);
  const fs::path bad = dir / "short_row.csv";
  write_text(bad, join_lines(lines));

  const common::Result<Records> read = read_run_csv(bad);
  ASSERT_FALSE(read.is_ok());
  const std::string reason = read.reason();
  EXPECT_NE(reason.find("short_row.csv"), std::string::npos) << reason;
  EXPECT_TRUE(reason.find("line 7") != std::string::npos ||
              reason.find("short_row.csv:7") != std::string::npos)
      << reason;
}

// An unknown mode string: the error names the column `mode` (and the file and line).
TEST(StudioRunCsv, UnknownModeNamesTheColumn) {
  const fs::path dir = work_dir("bad_mode");
  const fs::path csv = dir / "run.csv";
  ASSERT_TRUE(written_run(dir, csv));
  std::vector<std::string> lines = lines_of(read_text(csv));
  std::vector<std::string> f = fields_of(lines[3]);
  ASSERT_EQ(f.size(), kColumns);
  f[1] = "flying";
  lines[3] = join_fields(f);
  const fs::path bad = dir / "bad_mode.csv";
  write_text(bad, join_lines(lines));

  const common::Result<Records> read = read_run_csv(bad);
  ASSERT_FALSE(read.is_ok());
  const std::string reason = read.reason();
  EXPECT_NE(reason.find("bad_mode.csv"), std::string::npos) << reason;
  EXPECT_NE(reason.find("mode"), std::string::npos) << reason;
  EXPECT_TRUE(reason.find("line 4") != std::string::npos ||
              reason.find("bad_mode.csv:4") != std::string::npos)
      << reason;
}

TEST(StudioRunCsv, MissingFileNamesIt) {
  const fs::path missing = work_dir("missing") / "nothing_here.csv";
  const common::Result<Records> read = read_run_csv(missing);
  ASSERT_FALSE(read.is_ok());
  EXPECT_NE(read.reason().find("nothing_here.csv"), std::string::npos) << read.reason();
}

}  // namespace
}  // namespace cobot::studio
