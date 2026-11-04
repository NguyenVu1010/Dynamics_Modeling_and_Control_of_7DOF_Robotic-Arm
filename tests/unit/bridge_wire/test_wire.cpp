// Wire types of the studio bridge (feature shm-bridge, cobot_studio design 6.5, ADR-022 "Bản ghi
// trên dây là POD"): every type placed in the region is trivially copyable and standard layout,
// to_sample copies the LogRecord fields bit for bit, poses go row-major and back exactly, and
// copy_text never leaves a field unterminated.
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <type_traits>

#include <gtest/gtest.h>

#include "cobot/bridge/seqlock.hpp"
#include "cobot/bridge/wire.hpp"
#include "cobot/common/types.hpp"
#include "cobot/pipeline/log_record.hpp"
#include "cobot/planning/waypoint_trajectory.hpp"

namespace cobot::bridge {
namespace {

constexpr int kDof = 7;

template <typename T>
bool is_wire_type() {
  return std::is_trivially_copyable_v<T> && std::is_standard_layout_v<T>;
}

TEST(BridgeWire, EveryRegionTypeIsTriviallyCopyableAndStandardLayout) {
  EXPECT_TRUE(is_wire_type<WirePose>());
  EXPECT_TRUE(is_wire_type<StateSample<1>>());
  EXPECT_TRUE(is_wire_type<StateSample<2>>());
  EXPECT_TRUE(is_wire_type<StateSample<kDof>>());
  EXPECT_TRUE(is_wire_type<Command<1>>());
  EXPECT_TRUE(is_wire_type<Command<2>>());
  EXPECT_TRUE(is_wire_type<Command<kDof>>());
  EXPECT_TRUE(is_wire_type<WireWaypoint>());
  EXPECT_TRUE(is_wire_type<WireIssue>());
  EXPECT_TRUE(is_wire_type<Response>());
  EXPECT_TRUE(is_wire_type<JogRequest>());
  EXPECT_TRUE(is_wire_type<SessionInfo<1>>());
  EXPECT_TRUE(is_wire_type<SessionInfo<2>>());
  EXPECT_TRUE(is_wire_type<SessionInfo<kDof>>());
  EXPECT_TRUE(std::atomic<std::uint64_t>::is_always_lock_free);

  // Sizes go into the PR body and the test report.
  const auto record = [](const char* key, std::size_t size) {
    ::testing::Test::RecordProperty(key, std::to_string(size));
    std::printf("sizeof %s = %zu\n", key, size);
  };
  record("WirePose", sizeof(WirePose));
  record("StateSample7", sizeof(StateSample<kDof>));
  record("Command7", sizeof(Command<kDof>));
  record("Response", sizeof(Response));
  record("JogRequest", sizeof(JogRequest));
  record("SessionInfo7", sizeof(SessionInfo<kDof>));
  // WirePose: 3 + 9 doubles.
  EXPECT_EQ(sizeof(WirePose), 12 * sizeof(double));
}

TEST(BridgeWire, ConstantsMatchTheContract) {
  // "COTBRDG1" read as a little-endian 64-bit integer.
  EXPECT_EQ(kBridgeMagic, 0x3147445242544F43ULL);
  EXPECT_EQ(std::memcmp(&kBridgeMagic, "COTBRDG1", 8), 0);
  EXPECT_GE(kBridgeLayoutVersion, 1U);
  EXPECT_EQ(kStateSlots, 4096U);
  EXPECT_EQ(kCommandSlots, 64U);
  EXPECT_EQ(kResponseSlots, 16U);
  EXPECT_EQ(kWireWaypoints, planning::kMaxWaypoints);
  EXPECT_EQ(kWireIssues, 4 * kWireWaypoints);
  EXPECT_EQ(kPreviewPoints, 512U);
  EXPECT_EQ(kTextChars, 256U);
  EXPECT_EQ(kPathChars, 512U);
  EXPECT_EQ(kHeartbeatPeriodNs, 50000000);
  EXPECT_EQ(kHeartbeatTimeoutNs, 200000000);
  EXPECT_EQ(static_cast<std::uint32_t>(CommandType::kMoveJoint), 1U);
  EXPECT_EQ(static_cast<std::uint32_t>(CommandType::kExportSegment), 9U);
  EXPECT_EQ(static_cast<std::uint32_t>(CommandType::kJog), 100U);
}

// Rotation about z by `a`: [[c, -s, 0], [s, c, 0], [0, 0, 1]].
common::Mat3 rot_z(double a) {
  common::Mat3 R;
  R << std::cos(a), -std::sin(a), 0.0, std::sin(a), std::cos(a), 0.0, 0.0, 0.0, 1.0;
  return R;
}

// Rotation about x by `a`: [[1, 0, 0], [0, c, -s], [0, s, c]].
common::Mat3 rot_x(double a) {
  common::Mat3 R;
  R << 1.0, 0.0, 0.0, 0.0, std::cos(a), -std::sin(a), 0.0, std::sin(a), std::cos(a);
  return R;
}

// A LogRecord<7> with distinct values in every field the sample carries: q_i = 1 + i/10,
// qd_i = 2 + i/10, q_ref 3 +, qd_ref 4 +, tau 5 +, q_true 6 + i/10; e_k = 7 + k/10.
pipeline::LogRecord<kDof> distinct_record() {
  pipeline::LogRecord<kDof> r;
  r.t.ns = 123456789012;
  r.mode = pipeline::LoopMode::kStandby;              // 5
  r.hold_reason = pipeline::HoldReason::kJointLimit;  // 4
  r.cmd_mode = pipeline::CommandMode::kJogTcp;        // 3
  for (int i = 0; i < kDof; ++i) {
    const double f = i / 10.0;
    r.q(i) = 1.0 + f;
    r.qd(i) = 2.0 + f;
    r.q_ref(i) = 3.0 + f;
    r.qd_ref(i) = 4.0 + f;
    r.tau(i) = 5.0 + f;
    r.q_true(i) = 6.0 + f;
    r.qd_true(i) = -9.0;  // not on the wire
  }
  r.T_b_e.R = rot_z(0.3);
  r.T_b_e.p = common::Vec3(0.1, 0.2, 0.3);
  r.T_b_d.R = rot_x(0.7);
  r.T_b_d.p = common::Vec3(0.4, 0.5, 0.6);
  for (int k = 0; k < 6; ++k) {
    r.task_error(k) = 7.0 + k / 10.0;
  }
  r.step_time_us = 12.5;
  return r;
}

void expect_pose_bits(const WirePose& w, const common::Pose& T, const char* what) {
  for (int k = 0; k < 3; ++k) {
    EXPECT_EQ(w.p[k], T.p(k)) << what << " p[" << k << "]";
  }
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      EXPECT_EQ(w.R[3 * r + c], T.R(r, c)) << what << " R(" << r << ", " << c << ")";
    }
  }
}

TEST(BridgeWire, ToSampleCopiesEveryFieldBitForBit) {
  const pipeline::LogRecord<kDof> r = distinct_record();
  const StateSample<kDof> s = to_sample(r, 42, 3);
  EXPECT_EQ(s.step, 42U);
  EXPECT_EQ(s.manual_stop, 3U);
  EXPECT_EQ(s.t_ns, 123456789012);
  // Enums as their integer values.
  EXPECT_EQ(s.mode, 5U);
  EXPECT_EQ(s.mode, static_cast<std::uint8_t>(pipeline::LoopMode::kStandby));
  EXPECT_EQ(s.hold_reason, 4U);
  EXPECT_EQ(s.cmd_mode, 3U);
  for (int i = 0; i < kDof; ++i) {
    EXPECT_EQ(s.q[i], r.q(i)) << i;
    EXPECT_EQ(s.qd[i], r.qd(i)) << i;
    EXPECT_EQ(s.q_ref[i], r.q_ref(i)) << i;
    EXPECT_EQ(s.qd_ref[i], r.qd_ref(i)) << i;
    EXPECT_EQ(s.tau[i], r.tau(i)) << i;
    EXPECT_EQ(s.q_true[i], r.q_true(i)) << i;
  }
  EXPECT_EQ(s.q[3], 1.0 + 3 / 10.0);
  expect_pose_bits(s.T_b_e, r.T_b_e, "T_b_e");
  expect_pose_bits(s.T_b_d, r.T_b_d, "T_b_d");
  // Row-major: R[1] is row 0, column 1 of Rz(0.3), i.e. -sin 0.3; R[3] is +sin 0.3.
  EXPECT_EQ(s.T_b_e.R[1], -std::sin(0.3));
  EXPECT_EQ(s.T_b_e.R[3], std::sin(0.3));
  EXPECT_EQ(s.T_b_e.p[2], 0.3);
  for (int k = 0; k < 6; ++k) {
    EXPECT_EQ(s.task_error[k], r.task_error(k)) << k;
  }
  EXPECT_EQ(s.step_time_us, 12.5);
}

TEST(BridgeWire, ToSampleKeepsNanTruth) {
  pipeline::LogRecord<kDof> r = distinct_record();
  r.q_true.setConstant(pipeline::kUnknownTruth);  // a real robot does not know the truth
  const StateSample<kDof> s = to_sample(r, 1, 0);
  for (int i = 0; i < kDof; ++i) {
    EXPECT_TRUE(std::isnan(s.q_true[i])) << i;
  }
}

TEST(BridgeWire, PoseRoundTripIsExact) {
  common::Pose T;
  T.R = rot_x(-1.1) * rot_z(0.3);
  T.p = common::Vec3(0.1, -0.2, 1.0 / 3.0);
  const WirePose w = to_wire(T);
  expect_pose_bits(w, T, "to_wire");
  const common::Pose back = from_wire(w);
  EXPECT_TRUE(back.R == T.R);
  EXPECT_TRUE(back.p == T.p);
}

// ---------------------------------------------------------------- copy_text

TEST(BridgeWire, CopyTextTruncatesWithEllipsisAndTerminates) {
  char buf[300];
  std::memset(buf, 'Z', sizeof(buf));
  copy_text(buf, kTextChars, std::string(300, 'x'));
  // 256 bytes: 255 characters, the last three "...", then the terminating 0.
  EXPECT_EQ(buf[kTextChars - 1], '\0');
  EXPECT_EQ(std::strlen(buf), kTextChars - 1);
  EXPECT_EQ(std::string(buf), std::string(kTextChars - 4, 'x') + "...");
  // Nothing written past `size`.
  EXPECT_EQ(buf[kTextChars], 'Z');
}

TEST(BridgeWire, CopyTextFitsExactlyAndEmpty) {
  char buf[kTextChars];
  std::memset(buf, 'Z', sizeof(buf));
  copy_text(buf, sizeof(buf), "");
  EXPECT_EQ(std::string(buf), "");

  // 255 characters + 0 fit in 256: copied as is.
  const std::string fits(kTextChars - 1, 'y');
  copy_text(buf, sizeof(buf), fits);
  EXPECT_EQ(std::string(buf), fits);

  // One more character no longer fits: 252 characters + "...".
  copy_text(buf, sizeof(buf), std::string(kTextChars, 'y'));
  EXPECT_EQ(std::string(buf), std::string(kTextChars - 4, 'y') + "...");

  copy_text(buf, sizeof(buf), "apply rejected: not in standby");
  EXPECT_EQ(std::string(buf), "apply rejected: not in standby");
}

// ---------------------------------------------------------------- seqlock slot

TEST(BridgeSeqlock, WriteThenReadAndOddSequenceIsTorn) {
  SeqSlot<JogRequest> slot{};
  JogRequest j;
  j.owner_token = 77;
  j.refresh = 5;
  j.kind = 2;
  j.axis = 3;
  j.direction = -1;
  j.speed = 0.25;
  slot.write(j);
  EXPECT_EQ(slot.seq.load() % 2, 0U) << "stable after write";
  JogRequest out;
  ASSERT_TRUE(slot.try_read(out));
  EXPECT_EQ(out.owner_token, 77U);
  EXPECT_EQ(out.refresh, 5U);
  EXPECT_EQ(out.kind, 2);
  EXPECT_EQ(out.axis, 3);
  EXPECT_EQ(out.direction, -1);
  EXPECT_EQ(out.speed, 0.25);

  // A writer in the middle of a write (odd sequence): the read fails at once, never spins.
  const std::uint64_t stable = slot.seq.load();
  slot.seq.store(stable + 1);
  EXPECT_FALSE(slot.try_read(out));
  slot.seq.store(stable + 2);
  EXPECT_TRUE(slot.try_read(out));
}

}  // namespace
}  // namespace cobot::bridge
