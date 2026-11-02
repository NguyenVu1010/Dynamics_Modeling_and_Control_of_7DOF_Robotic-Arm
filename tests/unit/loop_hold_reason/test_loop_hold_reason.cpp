#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/timestamp.hpp"
#include "cobot/pipeline/log_record.hpp"

#include "control_loop_test_util.hpp"

// Feature comm-timing: ControlLoop names why it entered kHolding (pipeline::HoldReason). The
// reason is stored when the loop enters kHolding, the first matching one in the enum order
// (non-finite, stamp backwards, state timeout, joint limit, overspeed) wins, and it is kept in
// every record and by hold_reason() until reset(); kNone before and after. The Run CSV gains the
// last column `hold` (to_string of the reason). Rig: tests/component/control_loop (ideal SimPlant,
// tests/e2e/configs/ideal_world.yaml: 1 kHz, hal_timeout 5 ms, soft margin 3 deg).
namespace cobot::pipeline::test {
namespace {

using Edit = std::function<void(State&)>;

constexpr int kWarmup = 100;  // cycles of plain tracking before the trigger

void no_edit(State& /*unused*/) {}

// Starts a rig on `config` and runs kWarmup tracking cycles.
std::unique_ptr<Rig> warm_rig(const Setup& setup, const Config<kN>& config) {
  std::unique_ptr<Rig> rig = make_rig(setup, config);
  EXPECT_TRUE(rig->start().is_ok());
  State s;
  Command cmd;
  for (int k = 0; k < kWarmup; ++k) {
    EXPECT_TRUE(rig->cycle(s, cmd).is_ok());
  }
  EXPECT_NE(rig->loop.mode(), LoopMode::kHolding);
  EXPECT_EQ(rig->loop.hold_reason(), HoldReason::kNone);
  return rig;
}

// Stamp of the last state the loop saw, frozen: repeats it `repeats` times with `last` applied on
// the final repeat. hal_timeout 5 ms at 1 kHz: the 6th repeat (6 ms > 5 ms) is stale, the 5th is
// not (5 ms is not > 5 ms).
void frozen_cycles(Rig& rig, int repeats, const Edit& last) {
  State s;
  Command cmd;
  rig.cycle(s, cmd);
  const common::Timestamp t_frozen = s.t;
  for (int k = 1; k <= repeats; ++k) {
    rig.cycle(s, cmd, [&](State& x) {
      x.t = t_frozen;
      if (k == repeats) {
        last(x);
      }
    });
  }
}

void stamp_backwards(Rig& rig, const Edit& also) {
  State s;
  Command cmd;
  rig.cycle(s, cmd);
  const common::Timestamp t_last = s.t;
  rig.cycle(s, cmd, [&](State& x) {
    x.t = common::Timestamp{t_last.ns - 1};
    also(x);
  });
}

void one_cycle(Rig& rig, const Edit& edit) {
  State s;
  Command cmd;
  rig.cycle(s, cmd, edit);
}

void expect_holding_for(Rig& rig, HoldReason reason) {
  EXPECT_EQ(rig.loop.mode(), LoopMode::kHolding);
  EXPECT_EQ(rig.loop.hold_reason(), reason) << to_string(rig.loop.hold_reason());
  EXPECT_EQ(rig.loop.last_record().mode, LoopMode::kHolding);
  EXPECT_EQ(rig.loop.last_record().hold_reason, reason);
}

struct Limits {
  double q_max0;
  double qd_max0;
  double margin;
};

Limits limits_of(const Setup& setup) {
  return Limits{
      q_max_of(setup.model)(0), qd_max_of(setup.model)(0), setup.config.safety.soft_margin_rad};
}

TEST(HoldReason, Strings) {
  EXPECT_STREQ(to_string(HoldReason::kNone), "none");
  EXPECT_STREQ(to_string(HoldReason::kNonFinite), "non_finite");
  EXPECT_STREQ(to_string(HoldReason::kStampBackwards), "stamp_backwards");
  EXPECT_STREQ(to_string(HoldReason::kStateTimeout), "state_timeout");
  EXPECT_STREQ(to_string(HoldReason::kJointLimit), "joint_limit");
  EXPECT_STREQ(to_string(HoldReason::kOverspeed), "overspeed");
  const LogRecord<kN> r;
  EXPECT_EQ(r.hold_reason, HoldReason::kNone);
}

TEST(HoldReason, NonFiniteState) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const std::unique_ptr<Rig> rig = warm_rig(*setup, setup->config);
  one_cycle(*rig, [](State& x) { x.qd(3) = std::numeric_limits<double>::quiet_NaN(); });
  expect_holding_for(*rig, HoldReason::kNonFinite);
}

TEST(HoldReason, StampGoingBackwards) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const std::unique_ptr<Rig> rig = warm_rig(*setup, setup->config);
  stamp_backwards(*rig, no_edit);
  expect_holding_for(*rig, HoldReason::kStampBackwards);
}

TEST(HoldReason, StateTimeout) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const std::unique_ptr<Rig> rig = warm_rig(*setup, setup->config);
  frozen_cycles(*rig, 5, no_edit);
  ASSERT_NE(rig->loop.mode(), LoopMode::kHolding) << "5 ms of repeats is not > 5 ms";
  EXPECT_EQ(rig->loop.hold_reason(), HoldReason::kNone);
  const std::unique_ptr<Rig> rig6 = warm_rig(*setup, setup->config);
  frozen_cycles(*rig6, 6, no_edit);
  expect_holding_for(*rig6, HoldReason::kStateTimeout);
}

TEST(HoldReason, JointLimit) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const Limits l = limits_of(*setup);
  const std::unique_ptr<Rig> rig = warm_rig(*setup, setup->config);
  one_cycle(*rig, [&](State& x) { x.q(0) = l.q_max0 - 0.5 * l.margin; });
  expect_holding_for(*rig, HoldReason::kJointLimit);
}

TEST(HoldReason, Overspeed) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const Limits l = limits_of(*setup);
  const std::unique_ptr<Rig> rig = warm_rig(*setup, setup->config);
  one_cycle(*rig, [&](State& x) { x.qd(0) = -(l.qd_max0 + 0.1); });
  expect_holding_for(*rig, HoldReason::kOverspeed);
}

// Several conditions in the same step: the first in enum order names the hold.
TEST(HoldReason, NonFiniteWinsOverAStaleStamp) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const Edit nan_q = [](State& x) { x.q(2) = std::numeric_limits<double>::quiet_NaN(); };
  const std::unique_ptr<Rig> backwards = warm_rig(*setup, setup->config);
  stamp_backwards(*backwards, nan_q);
  expect_holding_for(*backwards, HoldReason::kNonFinite);
  const std::unique_ptr<Rig> timeout = warm_rig(*setup, setup->config);
  frozen_cycles(*timeout, 6, nan_q);
  expect_holding_for(*timeout, HoldReason::kNonFinite);
}

TEST(HoldReason, StaleStampWinsOverTheSafeSet) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const Limits l = limits_of(*setup);
  const std::unique_ptr<Rig> backwards = warm_rig(*setup, setup->config);
  stamp_backwards(*backwards, [&](State& x) { x.q(0) = l.q_max0 - 0.5 * l.margin; });
  expect_holding_for(*backwards, HoldReason::kStampBackwards);
  const std::unique_ptr<Rig> timeout = warm_rig(*setup, setup->config);
  frozen_cycles(*timeout, 6, [&](State& x) { x.qd(0) = l.qd_max0 + 0.1; });
  expect_holding_for(*timeout, HoldReason::kStateTimeout);
}

TEST(HoldReason, JointLimitWinsOverOverspeed) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const Limits l = limits_of(*setup);
  const std::unique_ptr<Rig> rig = warm_rig(*setup, setup->config);
  one_cycle(*rig, [&](State& x) {
    x.q(0) = l.q_max0 - 0.5 * l.margin;
    x.qd(1) = l.qd_max0 + 5.0;
  });
  expect_holding_for(*rig, HoldReason::kJointLimit);
}

// Later conditions while holding do not rename the hold; every record keeps the reason of the
// entry, rows before it say none.
TEST(HoldReason, KeptInEveryRecordWhileHolding) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const Limits l = limits_of(*setup);
  const std::unique_ptr<Rig> rig = warm_rig(*setup, setup->config);
  std::vector<Record> before;
  drain(rig->loop, before);
  ASSERT_EQ(before.size(), static_cast<std::size_t>(kWarmup));
  for (const Record& r : before) {
    EXPECT_EQ(r.hold_reason, HoldReason::kNone);
  }
  one_cycle(*rig, [&](State& x) { x.qd(0) = l.qd_max0 + 0.1; });
  expect_holding_for(*rig, HoldReason::kOverspeed);
  // (A non-finite state while holding would fault on the non-finite hold torque.)
  one_cycle(*rig, [&](State& x) { x.q(0) = l.q_max0 - 0.5 * l.margin; });
  stamp_backwards(*rig, no_edit);
  frozen_cycles(*rig, 10, no_edit);
  for (int k = 0; k < 50; ++k) {
    one_cycle(*rig, no_edit);
  }
  expect_holding_for(*rig, HoldReason::kOverspeed);
  std::vector<Record> held;
  drain(rig->loop, held);
  ASSERT_FALSE(held.empty());
  for (const Record& r : held) {
    EXPECT_EQ(r.mode, LoopMode::kHolding);
    EXPECT_EQ(r.hold_reason, HoldReason::kOverspeed);
  }
}

// reset() (external mode) clears the reason; the next hold gets its own.
TEST(HoldReason, ClearedByReset) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const Limits l = limits_of(*setup);
  Config<kN> cfg = setup->config;
  cfg.command_source = CommandSource::kExternal;
  const std::unique_ptr<Rig> rig = warm_rig(*setup, cfg);
  ASSERT_EQ(rig->loop.mode(), LoopMode::kStandby);
  one_cycle(*rig, [&](State& x) { x.qd(1) = l.qd_max0 + 0.1; });
  expect_holding_for(*rig, HoldReason::kOverspeed);
  State s;
  Command cmd;
  for (int k = 0; k < 200; ++k) {
    rig->cycle(s, cmd);
  }
  EXPECT_EQ(rig->loop.hold_reason(), HoldReason::kOverspeed);
  ASSERT_TRUE(rig->loop.reset(s).is_ok());
  EXPECT_EQ(rig->loop.mode(), LoopMode::kStandby);
  EXPECT_EQ(rig->loop.hold_reason(), HoldReason::kNone);
  one_cycle(*rig, no_edit);
  EXPECT_EQ(rig->loop.mode(), LoopMode::kStandby);
  EXPECT_EQ(rig->loop.last_record().hold_reason, HoldReason::kNone);
  frozen_cycles(*rig, 6, no_edit);
  expect_holding_for(*rig, HoldReason::kStateTimeout);
}

std::vector<std::string> cells(const std::string& line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, ',')) {
    out.push_back(item);
  }
  return out;
}

TEST(HoldReasonCsv, HoldIsTheLastRunCsvColumn) {
  std::ostringstream header;
  write_csv_header<kN>(header);
  std::string h = header.str();
  ASSERT_FALSE(h.empty());
  ASSERT_EQ(h.back(), '\n');
  h.pop_back();
  const std::vector<std::string> cols = cells(h);
  // 2 + 5 * 7 + 12 + 6 + step_us + hold = 57 columns.
  ASSERT_EQ(cols.size(), 57U);
  EXPECT_EQ(cols[55], "step_us");
  EXPECT_EQ(cols[56], "hold");

  for (const HoldReason reason : {HoldReason::kNone,
                                  HoldReason::kNonFinite,
                                  HoldReason::kStampBackwards,
                                  HoldReason::kStateTimeout,
                                  HoldReason::kJointLimit,
                                  HoldReason::kOverspeed}) {
    LogRecord<kN> r;
    r.mode = reason == HoldReason::kNone ? LoopMode::kTracking : LoopMode::kHolding;
    r.hold_reason = reason;
    r.step_time_us = 12.5;
    std::ostringstream row;
    write_csv_row<kN>(row, r);
    std::string line = row.str();
    ASSERT_EQ(line.back(), '\n');
    line.pop_back();
    const std::vector<std::string> c = cells(line);
    ASSERT_EQ(c.size(), 57U);
    EXPECT_EQ(c[55], "12.5");
    EXPECT_EQ(c[56], to_string(reason));
  }
}

}  // namespace
}  // namespace cobot::pipeline::test
