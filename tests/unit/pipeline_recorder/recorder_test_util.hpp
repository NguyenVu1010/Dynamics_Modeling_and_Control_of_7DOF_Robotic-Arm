#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/pipeline/log_record.hpp"
#include "cobot/pipeline/recorder.hpp"

// Shared helpers of the Recorder tests (feature pipeline-recorder, design
// docs/design/2026-11-02-cobot-studio-design.md 6.1): records tagged by their stamp t (ns), and
// the stamps of a RecordSpan in its order (oldest first).
namespace cobot::pipeline::test {

constexpr int kN = 7;
using Record = LogRecord<kN>;
using Span = RecordSpan<kN>;

// A record whose stamp is `t` ns; q, tau and the task error carry t too, so a row copied from
// the wrong slot or only partly copied shows in more than one field.
inline Record make_record(std::int64_t t) {
  Record r;
  r.t.ns = t;
  r.mode = (t % 2 == 0) ? LoopMode::kTracking : LoopMode::kHolding;
  const auto v = static_cast<double>(t);
  for (int i = 0; i < kN; ++i) {
    r.q(i) = v + 0.125 * i;
    r.qd(i) = -v - 0.25 * i;
    r.q_ref(i) = 2.0 * v + i;
    r.qd_ref(i) = 0.5 * v - i;
    r.tau(i) = 3.0 * v + 0.0625 * i;
  }
  r.T_b_e.p = common::Vec3(v, 1.0, -v);
  r.T_b_d.p = common::Vec3(-v, 2.0, v);
  r.task_error.setConstant(v * 1e-3);
  r.step_time_us = v + 0.5;
  r.hold_reason = (t % 3 == 0) ? HoldReason::kNone : HoldReason::kOverspeed;
  return r;
}

// Stamps of every row of `span`, through size() and operator[] (oldest first).
inline std::vector<std::int64_t> stamps(const Span& span) {
  std::vector<std::int64_t> out;
  out.reserve(span.size());
  for (std::size_t i = 0; i < span.size(); ++i) {
    out.push_back(span[i].t.ns);
  }
  return out;
}

// t = first, first + 1, ..., last.
inline std::vector<std::int64_t> range(std::int64_t first, std::int64_t last) {
  std::vector<std::int64_t> out;
  for (std::int64_t t = first; t <= last; ++t) {
    out.push_back(t);
  }
  return out;
}

// Every field of `got` equals make_record(t) of its own stamp.
inline void expect_row_of_stamp(const Record& got) {
  const Record want = make_record(got.t.ns);
  EXPECT_EQ(got.mode, want.mode) << "t " << got.t.ns;
  EXPECT_EQ(got.q, want.q) << "t " << got.t.ns;
  EXPECT_EQ(got.qd, want.qd) << "t " << got.t.ns;
  EXPECT_EQ(got.q_ref, want.q_ref) << "t " << got.t.ns;
  EXPECT_EQ(got.qd_ref, want.qd_ref) << "t " << got.t.ns;
  EXPECT_EQ(got.tau, want.tau) << "t " << got.t.ns;
  EXPECT_EQ(got.T_b_e.p, want.T_b_e.p) << "t " << got.t.ns;
  EXPECT_EQ(got.T_b_d.p, want.T_b_d.p) << "t " << got.t.ns;
  EXPECT_EQ(got.task_error, want.task_error) << "t " << got.t.ns;
  EXPECT_EQ(got.step_time_us, want.step_time_us) << "t " << got.t.ns;
  EXPECT_EQ(got.hold_reason, want.hold_reason) << "t " << got.t.ns;
}

}  // namespace cobot::pipeline::test
