#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <new>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "cobot/common/types.hpp"
#include "cobot/hal/joint_interface.hpp"
#include "cobot/imperfect/config.hpp"
#include "cobot/imperfect/imperfect_joint_interface.hpp"

// Counts global operator new calls while g_count_allocs is set (catches allocations inside the
// compiled imperfect library, where EIGEN_RUNTIME_NO_MALLOC is not defined). Every form is
// replaced so all allocations and deallocations stay paired through malloc/free under ASan.
namespace {
std::atomic<bool> g_count_allocs{false};
std::atomic<long> g_alloc_count{0};

void* counted_malloc(std::size_t size) {
  if (g_count_allocs.load()) {
    g_alloc_count.fetch_add(1);
  }
  return std::malloc(size == 0 ? 1 : size);
}
void* counted_or_throw(std::size_t size) {
  void* p = counted_malloc(size);
  if (p == nullptr) {
    throw std::bad_alloc();
  }
  return p;
}
}  // namespace

void* operator new(std::size_t size) {
  return counted_or_throw(size);
}
void* operator new[](std::size_t size) {
  return counted_or_throw(size);
}
void* operator new(std::size_t size, const std::nothrow_t& /*tag*/) noexcept {
  return counted_malloc(size);
}
void* operator new[](std::size_t size, const std::nothrow_t& /*tag*/) noexcept {
  return counted_malloc(size);
}
void operator delete(void* p) noexcept {
  std::free(p);
}
void operator delete[](void* p) noexcept {
  std::free(p);
}
void operator delete(void* p, std::size_t /*size*/) noexcept {
  std::free(p);
}
void operator delete[](void* p, std::size_t /*size*/) noexcept {
  std::free(p);
}
void operator delete(void* p, const std::nothrow_t& /*tag*/) noexcept {
  std::free(p);
}
void operator delete[](void* p, const std::nothrow_t& /*tag*/) noexcept {
  std::free(p);
}

// Feature comm-timing (realism-2 design 5 and 6.3, brief "Behaviour (pinned)"):
// - servo zero-order hold: S = control_rate / servo_rate cycles; a packet on read k (k counted
//   from 0) when k % S == 0. A received packet takes a fresh inner sample through the sensor chain
//   and forwards the shaped command (after resolution, before the command delay); between packets
//   and on a lost packet read() repeats the previous output (same stamp) and write() re-sends the
//   last forwarded command. The inner read() is called every cycle.
// - one Bernoulli draw per packet decides the loss of both directions.
// - latency burst: while none is active, one draw per cycle; on success the next burst_cycles
//   reads repeat the previous output.
// - before the first received packet, read() returns the first inner sample processed normally.
// Fake inner HAL: stamp k ms and a q, qd that change on every read, so a repeated output is
// recognisable bit for bit.
namespace cobot::imperfect {
namespace {

constexpr int kN = 7;
constexpr double kDt = 1e-3;  // s, control rate 1 kHz
constexpr std::int64_t kNsPerMs = 1'000'000;
using Vec = common::VecN<kN>;
using Io = ImperfectJointInterface<kN>;

Vec q_at(std::int64_t k) {
  Vec v;
  for (int j = 0; j < kN; ++j) {
    v[j] = 0.001 * static_cast<double>(k + 1) + 0.1 * j;
  }
  return v;
}

Vec qd_at(std::int64_t k) {
  Vec v;
  for (int j = 0; j < kN; ++j) {
    v[j] = 0.002 * static_cast<double>(k + 1) - 0.05 * j;
  }
  return v;
}

// Command of control cycle k: distinct on every cycle.
Vec cmd_at(std::int64_t k) {
  Vec v;
  for (int j = 0; j < kN; ++j) {
    v[j] = 0.5 * static_cast<double>(k + 1) + j;
  }
  return v;
}

class RampInner final : public JointInterface<kN> {
 public:
  Status read(JointState<kN>& out) override {
    out.t = common::Timestamp{reads * kNsPerMs};
    out.q = q_at(reads);
    out.qd = qd_at(reads);
    out.tau_measured = Vec::Constant(0.25 * static_cast<double>(reads));
    out.tau_valid = true;
    ++reads;
    return Status::ok();
  }
  Status write(const JointCommand<kN>& cmd) override {
    last = cmd;
    ++writes;
    return Status::ok();
  }
  std::int64_t reads = 0;
  std::int64_t writes = 0;
  JointCommand<kN> last;
};

ImperfectionConfig<kN> comm(double servo_rate_hz,
                            double loss = 0.0,
                            double burst_prob = 0.0,
                            int burst_cycles = 0) {
  ImperfectionConfig<kN> c;
  c.comm.servo_rate_hz = servo_rate_hz;
  c.comm.packet_loss_prob = loss;
  c.comm.latency_burst_prob = burst_prob;
  c.comm.burst_cycles = burst_cycles;
  return c;
}

JointState<kN> read_ok(Io& io) {
  JointState<kN> s;
  EXPECT_TRUE(io.read(s).is_ok());
  return s;
}

void write_ok(Io& io, std::int64_t k) {
  JointCommand<kN> c;
  c.tau = cmd_at(k);
  EXPECT_TRUE(io.write(c).is_ok());
}

// Last packet cycle at or before k for S cycles per packet.
std::int64_t packet_of(std::int64_t k, std::int64_t s) {
  return k - k % s;
}

TEST(CommServoRate, At250HzTheMeasurementChangesOnlyEveryFourthRead) {
  RampInner inner;
  Io io(inner, comm(250.0), kDt);  // S = 1000 Hz / 250 Hz = 4
  std::int64_t prev_ns = -1;
  for (std::int64_t k = 0; k < 40; ++k) {
    const JointState<kN> s = read_ok(io);
    const std::int64_t p = packet_of(k, 4);  // 0,0,0,0,4,4,4,4,8,...
    EXPECT_EQ(s.t.ns, p * kNsPerMs) << "read " << k;
    EXPECT_EQ(s.q, q_at(p)) << "read " << k;
    EXPECT_EQ(s.qd, qd_at(p)) << "read " << k;
    if (k % 4 == 0) {
      EXPECT_GT(s.t.ns, prev_ns) << "read " << k << ": a packet brings a newer stamp";
    } else {
      EXPECT_EQ(s.t.ns, prev_ns) << "read " << k << ": same stamp within the servo period";
    }
    prev_ns = s.t.ns;
  }
  EXPECT_EQ(inner.reads, 40) << "the inner read() is called every cycle";
  EXPECT_EQ(io.packets_lost(), 0);
  EXPECT_EQ(io.last_lost_cycle(), -1);
}

TEST(CommServoRate, At500HzTheMeasurementChangesEverySecondRead) {
  RampInner inner;
  Io io(inner, comm(500.0), kDt);  // S = 1000 / 500 = 2
  for (std::int64_t k = 0; k < 20; ++k) {
    const JointState<kN> s = read_ok(io);
    EXPECT_EQ(s.t.ns, packet_of(k, 2) * kNsPerMs) << "read " << k;
    EXPECT_EQ(s.q, q_at(packet_of(k, 2))) << "read " << k;
  }
}

TEST(CommServoRate, At250HzCommandsReachTheInnerOnlyOnPacketCycles) {
  RampInner inner;
  Io io(inner, comm(250.0), kDt);
  for (std::int64_t k = 0; k < 40; ++k) {
    read_ok(io);
    write_ok(io, k);
    // Cycles 0..3 send cmd(0), 4..7 cmd(4), ...: the last forwarded command is re-sent.
    EXPECT_EQ(inner.last.tau, cmd_at(packet_of(k, 4))) << "write " << k;
  }
  EXPECT_EQ(inner.writes, 40) << "the last forwarded command is re-sent every cycle";
}

// Order of the torque chain (design 6): ... resolution -> servo hold -> command delay -> ...
// With a delay of 1 the inner gets at cycle k the held command of cycle k - 1, i.e.
// cmd(4 floor((k - 1) / 4)); zeros at k = 0. (Hold after the delay would give cmd(4 floor(k/4) -
// 1).)
TEST(CommServoRate, HoldSitsBeforeTheCommandDelay) {
  RampInner inner;
  ImperfectionConfig<kN> c = comm(250.0);
  c.actuator.command_delay_cycles = 1;
  Io io(inner, c, kDt);
  for (std::int64_t k = 0; k < 24; ++k) {
    read_ok(io);
    write_ok(io, k);
    const Vec want = k == 0 ? Vec::Zero() : cmd_at(packet_of(k - 1, 4));
    EXPECT_EQ(inner.last.tau, want) << "write " << k;
  }
}

TEST(CommServoRate, ControlRateMustBeAnIntegerMultipleOfTheServoRate) {
  RampInner inner;
  // 1000 / 300 = 3.33 cycles: not an integer.
  try {
    const Io io(inner, comm(300.0), kDt);
    ADD_FAILURE() << "servo_rate_hz 300 at 1 kHz was accepted";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("comm.servo_rate_hz"), std::string::npos) << e.what();
  }
  // 1000 / 2000 = 0.5 cycles: below 1.
  try {
    const Io io(inner, comm(2000.0), kDt);
    ADD_FAILURE() << "servo_rate_hz 2000 at 1 kHz was accepted";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("comm.servo_rate_hz"), std::string::npos) << e.what();
  }
  // 0 (every cycle), 1000 (S = 1), 500 (S = 2), 250 (S = 4), 100 (S = 10) are fine.
  for (const double rate : {0.0, 1000.0, 500.0, 250.0, 100.0}) {
    EXPECT_NO_THROW(Io(inner, comm(rate), kDt)) << rate;
  }
}

TEST(CommPacketLoss, ProbabilityOneNeverGivesAFreshSampleAfterTheFirst) {
  RampInner inner;
  ImperfectionConfig<kN> c = comm(0.0, 1.0);
  c.sensor.q_offset_std_rad.setConstant(0.01);
  c.seed = 11;
  Io io(inner, c, kDt);
  EXPECT_EQ(io.packets_lost(), 0);
  EXPECT_EQ(io.last_lost_cycle(), -1);
  // First read: the first inner sample processed normally (offset added), never uninitialised.
  const Vec first_q = q_at(0) + io.q_offset_rad();
  for (std::int64_t k = 0; k < 50; ++k) {
    const JointState<kN> s = read_ok(io);
    write_ok(io, k);
    EXPECT_EQ(s.t.ns, 0) << "read " << k;
    EXPECT_EQ(s.q, first_q) << "read " << k;
    EXPECT_EQ(s.qd, qd_at(0)) << "read " << k;
    // No command was ever forwarded: the motor keeps the zero command.
    EXPECT_EQ(inner.last.tau, Vec::Zero()) << "write " << k;
  }
  // servo_rate 0: a packet every cycle, all 50 lost (cycles 0..49).
  EXPECT_EQ(io.packets_lost(), 50);
  EXPECT_EQ(io.last_lost_cycle(), 49);
  EXPECT_EQ(inner.reads, 50);
}

TEST(CommPacketLoss, AtServoRateOnlyPacketsCount) {
  RampInner inner;
  Io io(inner, comm(250.0, 1.0), kDt);
  for (std::int64_t k = 0; k < 50; ++k) {
    const JointState<kN> s = read_ok(io);
    EXPECT_EQ(s.t.ns, 0) << "read " << k;
    EXPECT_EQ(s.q, q_at(0)) << "read " << k;
  }
  // Packets on reads 0, 4, ..., 48: 48 / 4 + 1 = 13, the last one at cycle 48.
  EXPECT_EQ(io.packets_lost(), 13);
  EXPECT_EQ(io.last_lost_cycle(), 48);
}

// p = 0.1, a packet every cycle, 20000 packets: mean 2000 lost, std sqrt(20000 * 0.1 * 0.9) =
// 42.4; within 5 std: [1788, 2212]. A lost packet repeats the previous output (same stamp, same q);
// a received one is the fresh sample of that cycle (stamp k ms).
TEST(CommPacketLoss, LostPacketsRepeatTheOutputAndFollowTheProbability) {
  RampInner inner;
  ImperfectionConfig<kN> c = comm(0.0, 0.1);
  c.seed = 3;
  Io io(inner, c, kDt);
  JointState<kN> prev = read_ok(io);
  std::int64_t lost = io.packets_lost();
  for (std::int64_t k = 1; k < 20000; ++k) {
    const JointState<kN> s = read_ok(io);
    if (io.packets_lost() > lost) {
      ASSERT_EQ(io.packets_lost(), lost + 1) << "read " << k;
      ASSERT_EQ(io.last_lost_cycle(), k);
      ASSERT_EQ(s.t.ns, prev.t.ns) << "read " << k;
      ASSERT_EQ(s.q, prev.q) << "read " << k;
    } else {
      ASSERT_EQ(s.t.ns, k * kNsPerMs) << "read " << k;
      ASSERT_EQ(s.q, q_at(k)) << "read " << k;
    }
    lost = io.packets_lost();
    prev = s;
  }
  EXPECT_GE(io.packets_lost(), 1788);
  EXPECT_LE(io.packets_lost(), 2212);
}

// p = 0.5 at 250 Hz, 8000 reads = 2000 packets: mean 1000 lost, std sqrt(2000 * 0.25) = 22.4;
// within 5 std: [888, 1112].
TEST(CommPacketLoss, LossIsDrawnPerPacketNotPerCycle) {
  RampInner inner;
  ImperfectionConfig<kN> c = comm(250.0, 0.5);
  c.seed = 4;
  Io io(inner, c, kDt);
  for (std::int64_t k = 0; k < 8000; ++k) {
    read_ok(io);
    if (io.last_lost_cycle() >= 0) {
      ASSERT_EQ(io.last_lost_cycle() % 4, 0) << "a loss only on packet cycles";
    }
  }
  EXPECT_GE(io.packets_lost(), 888);
  EXPECT_LE(io.packets_lost(), 1112);
}

// One draw per packet for both directions: a lost packet also drops that cycle's command (the
// inner gets the previous one again); a received packet forwards it.
TEST(CommPacketLoss, OneDrawDecidesBothDirections) {
  RampInner inner;
  ImperfectionConfig<kN> c = comm(0.0, 0.5);
  c.seed = 5;
  Io io(inner, c, kDt);
  Vec sent = Vec::Zero();
  int lost_cycles = 0;
  for (std::int64_t k = 0; k < 2000; ++k) {
    const std::int64_t before = io.packets_lost();
    read_ok(io);
    write_ok(io, k);
    if (io.packets_lost() > before) {
      ++lost_cycles;
    } else {
      sent = cmd_at(k);
    }
    ASSERT_EQ(inner.last.tau, sent) << "cycle " << k;
  }
  EXPECT_GT(lost_cycles, 0);
}

// prob 1, burst_cycles 3: read 0 is fresh and starts a burst, reads 1..3 repeat it, read 4 is
// fresh (no burst active: draw, success) and reads 5..7 repeat it, ...: runs of 4 equal outputs.
TEST(CommLatencyBurst, BurstsRepeatTheOutputInRuns) {
  RampInner inner;
  Io io(inner, comm(0.0, 0.0, 1.0, 3), kDt);
  for (std::int64_t k = 0; k < 40; ++k) {
    const JointState<kN> s = read_ok(io);
    write_ok(io, k);
    EXPECT_EQ(s.t.ns, packet_of(k, 4) * kNsPerMs) << "read " << k;
    EXPECT_EQ(s.q, q_at(packet_of(k, 4))) << "read " << k;
    // A burst delays measurements only; packets and commands are untouched.
    EXPECT_EQ(inner.last.tau, cmd_at(k)) << "write " << k;
  }
  EXPECT_EQ(io.packets_lost(), 0);
}

TEST(CommLatencyBurst, ZeroBurstCyclesChangeNothing) {
  RampInner inner;
  Io io(inner, comm(0.0, 0.0, 1.0, 0), kDt);
  for (std::int64_t k = 0; k < 20; ++k) {
    const JointState<kN> s = read_ok(io);
    EXPECT_EQ(s.t.ns, k * kNsPerMs) << "read " << k;
    EXPECT_EQ(s.q, q_at(k)) << "read " << k;
  }
}

// 10 random configs x 10000 cycles = 1e5 cycles (fixed seed): stamps never go backwards, never
// come from the future (the output is never newer than the inner sample of the same cycle), and
// every output is finite.
TEST(CommLatencyBurst, StampsNeverDecreaseUnderRandomConfigs) {
  std::mt19937_64 gen(2026);
  const std::vector<double> rates = {0.0, 1000.0, 500.0, 250.0, 200.0, 125.0, 100.0};
  std::uniform_int_distribution<std::size_t> pick_rate(0, rates.size() - 1);
  std::uniform_real_distribution<double> prob(0.0, 0.3);
  std::uniform_int_distribution<int> burst(0, kMaxDelayCycles);
  std::uniform_int_distribution<int> delay(0, 4);
  for (int run = 0; run < 10; ++run) {
    ImperfectionConfig<kN> c = comm(rates[pick_rate(gen)], prob(gen), prob(gen), burst(gen));
    c.sensor.sensor_delay_cycles = delay(gen);
    c.sensor.q_noise_std_rad.setConstant(1e-4);
    c.sensor.qd_from_diff = run % 2 == 0;
    c.actuator.command_delay_cycles = delay(gen);
    c.seed = static_cast<std::uint64_t>(run) + 100U;
    RampInner inner;
    Io io(inner, c, kDt);
    std::int64_t prev_ns = -1;
    for (std::int64_t k = 0; k < 10000; ++k) {
      const JointState<kN> s = read_ok(io);
      write_ok(io, k);
      ASSERT_GE(s.t.ns, prev_ns) << "run " << run << " read " << k;
      ASSERT_LE(s.t.ns, k * kNsPerMs) << "run " << run << " read " << k;
      ASSERT_TRUE(s.q.allFinite() && s.qd.allFinite()) << "run " << run << " read " << k;
      ASSERT_TRUE(inner.last.tau.allFinite()) << "run " << run << " write " << k;
      prev_ns = s.t.ns;
    }
  }
}

// Outputs of `cycles` read/write cycles and the last command the inner received.
struct Recording {
  std::vector<JointState<kN>> states{};
  Vec last_tau = Vec::Zero();
};

Recording record(const ImperfectionConfig<kN>& c, int cycles) {
  RampInner inner;
  Io io(inner, c, kDt);
  Recording out;
  for (int k = 0; k < cycles; ++k) {
    out.states.push_back(read_ok(io));
    write_ok(io, k);
  }
  out.last_tau = inner.last.tau;
  return out;
}

bool same(const Recording& a, const Recording& b) {
  if (a.states.size() != b.states.size() || a.last_tau != b.last_tau) {
    return false;
  }
  for (std::size_t k = 0; k < a.states.size(); ++k) {
    const JointState<kN>& x = a.states[k];
    const JointState<kN>& y = b.states[k];
    if (x.t.ns != y.t.ns || x.q != y.q || x.qd != y.qd) {
      return false;
    }
  }
  return true;
}

TEST(CommDeterminism, SameSeedSameOutputsOtherSeedDiffers) {
  ImperfectionConfig<kN> c = comm(250.0, 0.3, 0.2, 2);
  c.seed = 9;
  const Recording a = record(c, 2000);
  const Recording b = record(c, 2000);
  c.seed = 10;
  const Recording other = record(c, 2000);
  EXPECT_TRUE(same(a, b));
  EXPECT_FALSE(same(a, other));
}

// The comm stream has its own salted generator: the values drawn once from the seed (offsets and
// torque scales) are the same with the comm group on or off.
TEST(CommDeterminism, CommDrawsLeaveTheExistingStreamsUntouched) {
  ImperfectionConfig<kN> off;
  off.seed = 21;
  off.sensor.q_offset_std_rad.setConstant(1e-3);
  off.actuator.torque_scale_std.setConstant(0.05);
  ImperfectionConfig<kN> on = off;
  on.comm.servo_rate_hz = 250.0;
  on.comm.packet_loss_prob = 0.5;
  on.comm.latency_burst_prob = 0.5;
  on.comm.burst_cycles = 3;
  RampInner inner_off;
  RampInner inner_on;
  const Io io_off(inner_off, off, kDt);
  const Io io_on(inner_on, on, kDt);
  EXPECT_EQ(io_on.q_offset_rad(), io_off.q_offset_rad());
  EXPECT_EQ(io_on.torque_scale(), io_off.torque_scale());
  // The noise stream is drawn per processed sample only: with a packet every cycle and nothing
  // lost the outputs are the same as with the comm group off.
  ImperfectionConfig<kN> noisy = off;
  noisy.sensor.q_noise_std_rad.setConstant(1e-4);
  noisy.sensor.encoder_bits.setConstant(18.0);
  ImperfectionConfig<kN> every_cycle = noisy;
  every_cycle.comm.servo_rate_hz = 1000.0;  // S = 1
  every_cycle.comm.burst_cycles = 5;        // inert: latency_burst_prob 0
  EXPECT_TRUE(same(record(noisy, 500), record(every_cycle, 500)));
}

TEST(CommNoAlloc, ReadAndWriteAllocateNothing) {
  RampInner inner;
  ImperfectionConfig<kN> c = comm(250.0, 0.2, 0.1, 4);
  c.sensor.sensor_delay_cycles = 2;
  c.actuator.command_delay_cycles = 1;
  Io io(inner, c, kDt);
  JointState<kN> s;
  JointCommand<kN> cmd;
  cmd.tau = cmd_at(0);
  g_alloc_count.store(0);
  g_count_allocs.store(true);
  Eigen::internal::set_is_malloc_allowed(false);
  bool ok = true;
  for (int k = 0; k < 2000; ++k) {
    ok = io.read(s).is_ok() && ok;
    ok = io.write(cmd).is_ok() && ok;
  }
  Eigen::internal::set_is_malloc_allowed(true);
  g_count_allocs.store(false);
  EXPECT_TRUE(ok);
  EXPECT_EQ(g_alloc_count.load(), 0);
}

}  // namespace
}  // namespace cobot::imperfect
