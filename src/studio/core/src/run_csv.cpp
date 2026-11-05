#include "cobot/studio/run_csv.hpp"

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <string>
#include <string_view>

#include "cobot/common/lie.hpp"

namespace cobot::studio {
namespace {

constexpr int kN = bench::kN;
constexpr auto kJoints = static_cast<std::size_t>(kN);
constexpr std::size_t kLegacyColumns = 2 + 5 * kJoints + 12 + 6 + 2;  // up to `hold`
constexpr std::size_t kColumns = kLegacyColumns + 8 * kJoints + 1;    // + qddref .. tauf, cmd
using Record = pipeline::LogRecord<kN>;

std::vector<std::string_view> split(std::string_view line) {
  std::vector<std::string_view> out;
  for (std::size_t start = 0;;) {
    const std::size_t comma = line.find(',', start);
    out.push_back(line.substr(start, comma == std::string_view::npos ? comma : comma - start));
    if (comma == std::string_view::npos) {
      return out;
    }
    start = comma + 1;
  }
}

// The fields of one line, read in column order; the first failure is kept in `error`.
class Row {
 public:
  Row(const std::vector<std::string>& names, std::vector<std::string_view> fields)
      : names_(names), fields_(std::move(fields)) {}

  double number() {
    const std::string cell(next());
    char* end = nullptr;
    errno = 0;
    const double v = std::strtod(cell.c_str(), &end);
    if (cell.empty() || end != cell.c_str() + cell.size() || errno == ERANGE) {
      fail(cell);
    }
    return v;
  }
  template <int M>
  common::VecN<M> vec() {
    common::VecN<M> v;
    for (int i = 0; i < M; ++i) {
      v(i) = number();
    }
    return v;
  }
  common::Pose pose() {
    common::Pose T;
    T.p = vec<3>();
    const common::Vec3 r = vec<3>();
    T.R = common::so3_exp(r);
    return T;
  }
  template <typename Enum, std::size_t K>
  Enum choice(const std::array<Enum, K>& values) {
    const std::string_view cell = next();
    for (const Enum v : values) {
      if (cell == pipeline::to_string(v)) {
        return v;
      }
    }
    fail(cell);
    return values[0];
  }
  std::int64_t integer() {
    const std::string cell(next());
    char* end = nullptr;
    errno = 0;
    const long long v = std::strtoll(cell.c_str(), &end, 10);
    if (cell.empty() || end != cell.c_str() + cell.size() || errno == ERANGE) {
      fail(cell);
    }
    return v;
  }
  const std::string& error() const { return error_; }

 private:
  std::string_view next() { return fields_[at_++]; }
  void fail(std::string_view cell) {
    if (error_.empty()) {
      error_ = "column " + names_[at_ - 1] + ": bad value '" + std::string(cell) + "'";
    }
  }
  const std::vector<std::string>& names_;
  std::vector<std::string_view> fields_;
  std::size_t at_ = 0;
  std::string error_;
};

constexpr std::array<pipeline::LoopMode, 6> kModes = {pipeline::LoopMode::kIdle,
                                                      pipeline::LoopMode::kTracking,
                                                      pipeline::LoopMode::kHolding,
                                                      pipeline::LoopMode::kFinished,
                                                      pipeline::LoopMode::kFault,
                                                      pipeline::LoopMode::kStandby};
constexpr std::array<pipeline::HoldReason, 7> kHolds = {pipeline::HoldReason::kNone,
                                                        pipeline::HoldReason::kNonFinite,
                                                        pipeline::HoldReason::kStampBackwards,
                                                        pipeline::HoldReason::kStateTimeout,
                                                        pipeline::HoldReason::kJointLimit,
                                                        pipeline::HoldReason::kOverspeed,
                                                        pipeline::HoldReason::kEstop};
constexpr std::array<pipeline::CommandMode, 6> kCmds = {pipeline::CommandMode::kConfig,
                                                        pipeline::CommandMode::kWaypoints,
                                                        pipeline::CommandMode::kJogJoint,
                                                        pipeline::CommandMode::kJogTcp,
                                                        pipeline::CommandMode::kMoveJoint,
                                                        pipeline::CommandMode::kMoveTcp};

Record parse(Row& row, bool full) {
  Record r;
  r.t.ns = row.integer();
  r.mode = row.choice(kModes);
  r.q = row.vec<kN>();
  r.qd = row.vec<kN>();
  r.q_ref = row.vec<kN>();
  r.qd_ref = row.vec<kN>();
  r.tau = row.vec<kN>();
  r.T_b_e = row.pose();
  r.T_b_d = row.pose();
  r.task_error = row.vec<6>();
  r.step_time_us = row.number();
  r.hold_reason = row.choice(kHolds);
  if (!full) {
    r.q_raw = r.q;
    r.qdd_ref.setZero();
    return r;  // truth NaN and cmd config: the LogRecord defaults
  }
  r.qdd_ref = row.vec<kN>();
  r.q_raw = row.vec<kN>();
  r.q_true = row.vec<kN>();
  r.qd_true = row.vec<kN>();
  r.qdd_true = row.vec<kN>();
  r.q_motor_true = row.vec<kN>();
  r.tau_motor = row.vec<kN>();
  r.tau_friction = row.vec<kN>();
  r.cmd_mode = row.choice(kCmds);
  return r;
}

}  // namespace

common::Result<std::vector<Record>> read_run_csv(const fs::path& csv) {
  using Out = common::Result<std::vector<Record>>;
  std::ifstream in(csv, std::ios::binary);
  if (!in) {
    return Out::error(csv.string() + ": cannot open");
  }
  std::string line;
  if (!std::getline(in, line)) {
    return Out::error(csv.string() + ": empty file");
  }
  std::vector<std::string> names;
  for (const std::string_view n : split(line)) {
    names.emplace_back(n);
  }
  const bool full = names.size() == kColumns;
  if (!full && names.size() != kLegacyColumns) {
    return Out::error(csv.string() + " line 1: header has " + std::to_string(names.size()) +
                      " columns, want " + std::to_string(kColumns) + " or " +
                      std::to_string(kLegacyColumns));
  }
  std::vector<Record> rows;
  for (std::size_t number = 2; std::getline(in, line); ++number) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty()) {
      continue;
    }
    std::vector<std::string_view> fields = split(line);
    const std::string where = csv.string() + " line " + std::to_string(number);
    if (fields.size() != names.size()) {
      return Out::error(where + ": " + std::to_string(fields.size()) + " fields, want " +
                        std::to_string(names.size()));
    }
    Row row(names, std::move(fields));
    rows.push_back(parse(row, full));
    if (!row.error().empty()) {
      return Out::error(where + ", " + row.error());
    }
  }
  return Out::ok(std::move(rows));
}

}  // namespace cobot::studio
