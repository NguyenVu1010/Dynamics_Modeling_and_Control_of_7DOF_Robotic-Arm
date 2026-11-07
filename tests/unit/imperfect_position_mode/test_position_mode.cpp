// Position commands through ImperfectJointInterface (brief drive-loop 2): a command with mode
// kPosition is the set-point packet of the bus. The servo hold (a packet forwards the command,
// otherwise the last forwarded one is re-sent) and the command delay ring apply to the whole
// JointCommand and nothing else does: no torque scale, ripple, deadband, resolution, slew, bus or
// torque-speed ceiling. A delayed position command reaches the inner interface unchanged; a delayed
// torque command (the zero commands before the delay ring fills) takes the torque path.
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/hal/joint_interface.hpp"
#include "cobot/imperfect/config.hpp"
#include "cobot/imperfect/imperfect_joint_interface.hpp"
#include "cobot/pipeline/config.hpp"

namespace cobot::imperfect {
namespace {

constexpr int kN = 7;
constexpr double kDt = 1e-3;  // s, control rate 1 kHz
using Vec = common::VecN<kN>;
using Io = ImperfectJointInterface<kN>;
using Command = JointCommand<kN>;

std::uint64_t bits_of(double v) {
  std::uint64_t u = 0;
  std::memcpy(&u, &v, sizeof(u));
  return u;
}

bool bits_equal(const Vec& a, const Vec& b) {
  for (int j = 0; j < kN; ++j) {
    if (bits_of(a(j)) != bits_of(b(j))) {
      return false;
    }
  }
  return true;
}

// Every field of the command, bit for bit.
::testing::AssertionResult same_command(const Command& got, const Command& want) {
  if (got.mode != want.mode) {
    return ::testing::AssertionFailure()
           << "mode " << static_cast<int>(got.mode) << " != " << static_cast<int>(want.mode);
  }
  if (got.seq != want.seq) {
    return ::testing::AssertionFailure() << "seq " << got.seq << " != " << want.seq;
  }
  if (!bits_equal(got.q, want.q)) {
    return ::testing::AssertionFailure()
           << "q " << got.q.transpose() << " != " << want.q.transpose();
  }
  if (!bits_equal(got.qd, want.qd)) {
    return ::testing::AssertionFailure()
           << "qd " << got.qd.transpose() << " != " << want.qd.transpose();
  }
  if (!bits_equal(got.tau, want.tau)) {
    return ::testing::AssertionFailure()
           << "tau " << got.tau.transpose() << " != " << want.tau.transpose();
  }
  return ::testing::AssertionSuccess();
}

// Inner interface with a constant state; keeps every command it receives.
class RecordingInner final : public JointInterface<kN> {
 public:
  RecordingInner() {
    state.q = Vec::Constant(0.3);
    state.qd = Vec::Constant(1.0);
    received.reserve(256);
  }
  Status read(JointState<kN>& out) override {
    out = state;
    return Status::ok();
  }
  Status write(const JointCommand<kN>& cmd) override {
    received.push_back(cmd);
    return Status::ok();
  }
  JointState<kN> state;
  std::vector<Command> received;
};

// The budget profile (torque scale std 0.10, asym, ripple, deadband 0.05-0.435 N m, resolution,
// slew, torque-speed curve, bus) with the given comm group and command delay.
ImperfectionConfig<kN> budget(const CommConfig& comm, int command_delay_cycles) {
  const auto imp = pipeline::load_imperfections<kN>(std::filesystem::path(COBOT_REPO_DIR) /
                                                    "config/imperfections/budget_xarm7.yaml");
  EXPECT_TRUE(imp) << imp.reason();
  ImperfectionConfig<kN> p = imp ? imp.value() : ImperfectionConfig<kN>{};
  p.comm = comm;
  p.actuator.command_delay_cycles = command_delay_cycles;
  return p;
}

Vec budget_peak() {
  return (Vec() << 204.0, 204.0, 107.0, 107.0, 36.0, 36.0, 36.0).finished();
}

// Set point number `seq`: q, qd and a torque feed-forward that differ per seq. The feed-forward
// mixes values below the budget deadband (0.01 N m), off the resolution grid and above the slew
// step, so any torque shaping would change some bit.
Command position_command(std::uint32_t seq) {
  Command c;
  c.mode = hal::DriveMode::kPosition;
  c.seq = seq;
  const auto s = static_cast<double>(seq);
  for (int j = 0; j < kN; ++j) {
    c.q(j) = 0.1 * j - 0.01 * s;
    c.qd(j) = 0.02 * s - 0.03 * j;
  }
  c.tau << 0.01, -0.01, 123.456, -98.7654 * s, 0.0123, -35.9, 1.0 / 3.0;
  return c;
}

void cycle(Io& io, const Command& cmd) {
  JointState<kN> s;
  EXPECT_TRUE(io.read(s).is_ok());
  EXPECT_TRUE(io.write(cmd).is_ok());
}

TEST(PositionMode, CommandReachesTheInnerUnchanged) {
  RecordingInner inner;
  Io io(inner, budget(CommConfig{}, 0), kDt, false, budget_peak());
  for (std::uint32_t seq = 1; seq <= 50; ++seq) {
    cycle(io, position_command(seq));
    ASSERT_EQ(inner.received.size(), static_cast<std::size_t>(seq));
    EXPECT_TRUE(same_command(inner.received.back(), position_command(seq))) << "seq " << seq;
  }
}

// Command delay 1, comm off: write k sends the command of write k - 1, and the zero torque command
// before the ring fills (torque path: zeros stay zeros through slew, bus and ceiling).
TEST(PositionMode, CommandDelayAppliesToTheWholeCommand) {
  RecordingInner inner;
  Io io(inner, budget(CommConfig{}, 1), kDt, false, budget_peak());
  for (std::uint32_t seq = 1; seq <= 20; ++seq) {
    cycle(io, position_command(seq));
  }
  ASSERT_EQ(inner.received.size(), 20U);
  EXPECT_EQ(inner.received[0].mode, hal::DriveMode::kTorque);
  EXPECT_TRUE(inner.received[0].tau.isZero(0.0)) << inner.received[0].tau.transpose();
  for (std::uint32_t k = 1; k < 20; ++k) {
    EXPECT_TRUE(same_command(inner.received[k], position_command(k))) << "write " << k;
  }
}

// Servo 250 Hz at 1 kHz (a packet on reads 0, 4, 8, ...), command delay 1, no loss, no bursts:
// read then write per cycle with seq 1..8. Forwarded: seq 1 (read 0) and seq 5 (read 4); the held
// command is re-sent in between; the delay shifts by one write:
//   write 0: ring not filled -> zero torque command; writes 1-4: seq 1; writes 5-7: seq 5.
TEST(PositionMode, ServoHoldAndDelay) {
  RecordingInner inner;
  CommConfig comm;
  comm.servo_rate_hz = 250.0;
  Io io(inner, budget(comm, 1), kDt, false, budget_peak());
  for (std::uint32_t seq = 1; seq <= 8; ++seq) {
    cycle(io, position_command(seq));
  }
  ASSERT_EQ(inner.received.size(), 8U);
  EXPECT_EQ(inner.received[0].mode, hal::DriveMode::kTorque);
  EXPECT_TRUE(inner.received[0].tau.isZero(0.0)) << inner.received[0].tau.transpose();
  const std::uint32_t want[8] = {0, 1, 1, 1, 1, 5, 5, 5};
  for (int k = 1; k < 8; ++k) {
    EXPECT_TRUE(
        same_command(inner.received[static_cast<std::size_t>(k)], position_command(want[k])))
        << "write " << k;
  }
}

// Every packet lost: nothing is ever forwarded, so the inner only gets the zero torque command.
TEST(PositionMode, LostPacketsNeverForwardAPositionCommand) {
  RecordingInner inner;
  CommConfig comm;
  comm.servo_rate_hz = 250.0;
  comm.packet_loss_prob = 1.0;
  Io io(inner, budget(comm, 1), kDt, false, budget_peak());
  for (std::uint32_t seq = 1; seq <= 40; ++seq) {
    cycle(io, position_command(seq));
  }
  ASSERT_EQ(inner.received.size(), 40U);
  for (std::size_t k = 0; k < inner.received.size(); ++k) {
    EXPECT_EQ(inner.received[k].mode, hal::DriveMode::kTorque) << "write " << k;
    EXPECT_TRUE(inner.received[k].tau.isZero(0.0)) << "write " << k;
  }
  EXPECT_GT(io.packets_lost(), 0);
}

// Static facts of the HAL change (owner question C5): default command is a zero torque command.
TEST(PositionMode, JointCommandDefaults) {
  static_assert(std::is_same_v<std::underlying_type_t<hal::DriveMode>, std::uint8_t>);
  static_assert(static_cast<int>(hal::DriveMode::kTorque) == 0);
  static_assert(static_cast<int>(hal::DriveMode::kPosition) == 1);
  const Command c{};
  EXPECT_EQ(c.mode, hal::DriveMode::kTorque);
  EXPECT_TRUE(c.tau.isZero(0.0));
  EXPECT_TRUE(c.q.isZero(0.0));
  EXPECT_TRUE(c.qd.isZero(0.0));
  EXPECT_EQ(c.seq, 0U);
}

}  // namespace
}  // namespace cobot::imperfect
