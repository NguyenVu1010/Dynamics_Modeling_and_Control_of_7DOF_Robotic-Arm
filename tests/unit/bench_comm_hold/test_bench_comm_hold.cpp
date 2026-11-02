#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>

#include <gtest/gtest.h>

#include "cobot/common/timestamp.hpp"
#include "cobot/dynamics/sim_plant.hpp"
#include "cobot/hal/joint_interface.hpp"
#include "cobot/imperfect/config.hpp"
#include "cobot/imperfect/imperfect_joint_interface.hpp"
#include "cobot/pipeline/control_loop.hpp"
#include "cobot/pipeline/sim_joint_interface.hpp"

#include "control_loop_test_util.hpp"

// Feature comm-timing, design 5 ("the safety hold on packet loss is a feature to demonstrate")
// and 8: ControlLoop + ImperfectJointInterface + SimPlant wired as bench::run_profile wires them
// (first read, start, then read -> step -> write -> advance), ideal apart from the comm group.
// tests/e2e/configs/ideal_world.yaml: 1 kHz, hal_timeout 5 ms.
//
// At servo 250 Hz a packet comes every S = 4 cycles, so the loop sees each stamp 4 times: three
// repeats (1, 2, 3 ms <= 5 ms), then a jump of 4 ms. One lost packet at cycle L (fresh sample of
// cycle L - 4 at t_f): repeats at L - 3 .. L + 1 are 1 .. 5 ms old, L + 2 is 6 ms > 5 ms -> hold at
// the cycle whose true time is t_f + 6 ms, in (t_f + 5 ms, t_f + 5 ms + 1 cycle]. With 12 ms one
// loss gives at most 7 ms of repeats and an 8 ms jump at L + 4: no hold.
// Also: SimJointInterface overrun (one draw per advance(); on success the plant runs
// 1 + overrun_cycles periods with the last command).
namespace cobot::pipeline::test {
namespace {

constexpr double kDt = 1e-3;
constexpr std::int64_t kNsPerMs = 1'000'000;
constexpr std::int64_t kThreeSeconds = 3000;  // cycles at 1 kHz

// Between the decorator and the SimJointInterface: remembers the true stamp of the last read.
class TrueStamp final : public hal::JointInterface<kN> {
 public:
  explicit TrueStamp(hal::JointInterface<kN>& inner) : inner_(inner) {}
  common::Status read(State& out) override {
    const common::Status s = inner_.read(out);
    last_ns = out.t.ns;
    return s;
  }
  common::Status write(const Command& cmd) override { return inner_.write(cmd); }
  std::int64_t last_ns = -1;

 private:
  hal::JointInterface<kN>& inner_;
};

imperfect::ImperfectionConfig<kN> servo_250(double loss, std::uint64_t seed) {
  imperfect::ImperfectionConfig<kN> c;
  c.comm.servo_rate_hz = 250.0;
  c.comm.packet_loss_prob = loss;
  c.seed = seed;
  return c;
}

struct CommRig {
  CommRig(const Setup& setup,
          const Config<kN>& config,
          const imperfect::ImperfectionConfig<kN>& imp)
      : plant(setup.model, config.home_q_rad),
        sim(plant, kDt),
        spy(sim),
        io(spy, imp, kDt),
        loop(setup.model, config) {}

  // Read 0 and start (as bench::run_profile).
  bool start() {
    State s;
    return io.read(s).is_ok() && loop.start(s).is_ok();
  }

  // One host cycle; `out_ns` gets the stamp the loop saw.
  void cycle(std::int64_t& out_ns) {
    State s;
    EXPECT_TRUE(io.read(s).is_ok());
    out_ns = s.t.ns;
    Command cmd;
    static_cast<void>(loop.step(s, cmd));
    EXPECT_TRUE(io.write(cmd).is_ok());
    sim.advance();
  }

  dynamics::SimPlant<kN> plant;
  SimJointInterface<kN> sim;
  TrueStamp spy;
  imperfect::ImperfectJointInterface<kN> io;
  ControlLoop<kN> loop;
};

TEST(BenchCommHold, ServoAt250HzWithoutLossNeverHolds) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  ASSERT_EQ(setup->config.safety.hal_timeout_s, 5.0 * 1.0e-3);
  const auto rig = std::make_unique<CommRig>(*setup, setup->config, servo_250(0.0, 1));
  ASSERT_TRUE(rig->start());
  std::int64_t out_ns = 0;
  for (std::int64_t k = 0; k < kThreeSeconds; ++k) {
    rig->cycle(out_ns);
    ASSERT_NE(rig->loop.mode(), LoopMode::kHolding)
        << "cycle " << k << ": " << to_string(rig->loop.hold_reason());
    ASSERT_NE(rig->loop.mode(), LoopMode::kFault) << "cycle " << k;
  }
  EXPECT_EQ(rig->loop.mode(), LoopMode::kTracking);  // 3 s < duration 10 s
  EXPECT_EQ(rig->loop.hold_reason(), HoldReason::kNone);
}

// What a run with servo 250 Hz and loss 1e-2 shows around its first lost packet.
struct FirstLoss {
  std::int64_t lost_cycle = -1;    // L, read count of the lost packet
  std::int64_t fresh_ns = -1;      // stamp of the last fresh sample before it
  std::int64_t hold_cycle = -1;    // read count of the step that entered kHolding, -1 if none
  std::int64_t hold_true_ns = -1;  // true (plant) time of that read
  HoldReason reason = HoldReason::kNone;
  bool single = false;  // the next packet (L + 4) was received
};

// Runs up to 3 s; stops 4 cycles after the next packet following the first loss. Read 0 is the
// one before start(), so the loop cycle k reads read count k + 1.
FirstLoss first_loss(const Setup& setup, double hal_timeout_s, std::uint64_t seed) {
  Config<kN> cfg = setup.config;
  cfg.safety.hal_timeout_s = hal_timeout_s;
  const auto rig = std::make_unique<CommRig>(setup, cfg, servo_250(1.0e-2, seed));
  FirstLoss f;
  EXPECT_TRUE(rig->start());
  std::int64_t prev_out_ns = 0;
  for (std::int64_t read = 1; read <= kThreeSeconds; ++read) {
    std::int64_t out_ns = 0;
    const std::int64_t lost_before = rig->io.packets_lost();
    rig->cycle(out_ns);
    if (f.lost_cycle < 0 && rig->io.packets_lost() > lost_before) {
      f.lost_cycle = rig->io.last_lost_cycle();
      EXPECT_EQ(f.lost_cycle, read);
      f.fresh_ns = prev_out_ns;  // read L - 1 still shows the fresh sample of L - 4
    }
    if (f.hold_cycle < 0 && rig->loop.mode() == LoopMode::kHolding) {
      f.hold_cycle = read;
      f.hold_true_ns = rig->spy.last_ns;
      f.reason = rig->loop.hold_reason();
    }
    prev_out_ns = out_ns;
    if (f.lost_cycle >= 0 && read == f.lost_cycle + 4) {
      f.single = rig->io.packets_lost() == 1;
    }
    if (f.lost_cycle >= 0 && read == f.lost_cycle + 7) {
      break;
    }
  }
  return f;
}

// The first seed whose first loss comes after start-up (L >= 8) and is a single loss.
std::optional<std::uint64_t> seed_with_single_first_loss(const Setup& setup) {
  for (std::uint64_t seed = 1; seed <= 40; ++seed) {
    const FirstLoss f = first_loss(setup, 5.0 * 1.0e-3, seed);
    if (f.lost_cycle >= 8 && f.single) {
      return seed;
    }
  }
  return std::nullopt;
}

TEST(BenchCommHold, OneLostPacketHoldsJustAfterTheTimeout) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const std::optional<std::uint64_t> seed = seed_with_single_first_loss(*setup);
  ASSERT_TRUE(seed) << "no seed in 1..40 gives a single first loss within 3 s";
  const FirstLoss f = first_loss(*setup, 5.0 * 1.0e-3, *seed);
  ASSERT_GE(f.lost_cycle, 8);
  EXPECT_EQ(f.lost_cycle % 4, 0) << "packets on read counts 0, 4, 8, ...";
  ASSERT_GE(f.hold_cycle, f.lost_cycle)
      << "held before the first loss, reason " << to_string(f.reason);
  // The fresh sample before the loss is the packet of read L - 4. Read 0 (before start) and read
  // 1 (before the first advance) are both at t = 0, read r >= 1 at (r - 1) ms: t_f = (L - 5) ms.
  EXPECT_EQ(f.fresh_ns, (f.lost_cycle - 5) * kNsPerMs);
  // Hold in (t_f + 5 ms, t_f + 5 ms + 1 cycle] of true time: t_f + 6 ms, cycle L + 2.
  EXPECT_GT(f.hold_true_ns, f.fresh_ns + 5 * kNsPerMs);
  EXPECT_LE(f.hold_true_ns, f.fresh_ns + 6 * kNsPerMs);
  EXPECT_EQ(f.hold_cycle, f.lost_cycle + 2);
  EXPECT_EQ(f.reason, HoldReason::kStateTimeout) << to_string(f.reason);
}

TEST(BenchCommHold, TwelveMillisecondTimeoutRidesThroughOneLostPacket) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  const std::optional<std::uint64_t> seed = seed_with_single_first_loss(*setup);
  ASSERT_TRUE(seed);
  const FirstLoss at5 = first_loss(*setup, 5.0 * 1.0e-3, *seed);
  const FirstLoss at12 = first_loss(*setup, 12.0 * 1.0e-3, *seed);
  // Same seed, same packet draws: the loss is at the same cycle.
  EXPECT_EQ(at12.lost_cycle, at5.lost_cycle);
  EXPECT_TRUE(at12.single);
  // Through L + 7 (the next packet at L + 4 was received): never held.
  EXPECT_EQ(at12.hold_cycle, -1) << "held at cycle " << at12.hold_cycle << " ("
                                 << to_string(at12.reason) << "), first loss at "
                                 << at12.lost_cycle;
}

// SimJointInterface overrun (bench only): constructor (plant, period, jitter_std_s, seed,
// overrun_prob, overrun_cycles). One draw per advance(); on success 1 + overrun_cycles periods.
using ArmSetup = Setup;  // inside a gtest fixture `Setup` names testing::Test::Setup

class Overrun : public ::testing::Test {
 protected:
  void SetUp() override {
    setup_ = load_setup();
    ASSERT_TRUE(setup_);
  }
  std::optional<ArmSetup> setup_;
};

TEST_F(Overrun, ProbabilityOneRunsOnePlusOverrunCyclesPeriods) {
  dynamics::SimPlant<kN> plant(setup_->model, setup_->config.home_q_rad);
  SimJointInterface<kN> sim(plant, kDt, 0.0, 1, 1.0, 2);
  State s;
  for (int k = 1; k <= 10; ++k) {
    sim.advance();
    // 3 periods per advance: 3 k ms.
    EXPECT_NEAR(plant.time_s(), 3.0 * k * kDt, 1e-12) << "advance " << k;
    ASSERT_TRUE(sim.read(s).is_ok());
    EXPECT_EQ(s.t.ns, std::int64_t{3} * k * kNsPerMs) << "advance " << k;
  }
}

TEST_F(Overrun, ProbabilityZeroIsTheNominalPeriod) {
  dynamics::SimPlant<kN> plant(setup_->model, setup_->config.home_q_rad);
  SimJointInterface<kN> sim(plant, kDt, 0.0, 1, 0.0, 5);
  for (int k = 1; k <= 10; ++k) {
    sim.advance();
  }
  EXPECT_NEAR(plant.time_s(), 10.0 * kDt, 1e-12);
}

// p = 0.5, 1 extra cycle, 4000 advances: each advance is 1 or 2 periods; overruns ~ 2000, std
// sqrt(4000 * 0.25) = 31.6, within 5 std: [1842, 2158].
TEST_F(Overrun, OneDrawPerAdvance) {
  dynamics::SimPlant<kN> plant(setup_->model, setup_->config.home_q_rad);
  // Zero torque would let the arm fall for 4 to 6 s; hold it at home with the hold law.
  control::HoldController<kN> hold(setup_->model, setup_->config.computed_torque);
  hold.engage(setup_->config.home_q_rad);
  SimJointInterface<kN> sim(plant, kDt, 0.0, 7, 0.5, 1);
  int overruns = 0;
  double t_prev = 0.0;
  State s;
  for (int k = 0; k < 4000; ++k) {
    ASSERT_TRUE(sim.read(s).is_ok());
    ASSERT_TRUE(sim.write(hold.compute(s)).is_ok());
    sim.advance();
    const double periods = (plant.time_s() - t_prev) / kDt;
    t_prev = plant.time_s();
    const bool one = std::abs(periods - 1.0) < 1e-6;
    const bool two = std::abs(periods - 2.0) < 1e-6;
    ASSERT_TRUE(one || two) << "advance " << k << ": " << periods << " periods";
    overruns += two ? 1 : 0;
  }
  EXPECT_GE(overruns, 1842);
  EXPECT_LE(overruns, 2158);
}

// The extra periods run with the last command: one overrun advance (2 periods) ends where two
// plain advances with the same command end.
TEST_F(Overrun, ExtraPeriodsUseTheLastCommand) {
  dynamics::SimPlant<kN> a(setup_->model, setup_->config.home_q_rad);
  dynamics::SimPlant<kN> b(setup_->model, setup_->config.home_q_rad);
  SimJointInterface<kN> over(a, kDt, 0.0, 1, 1.0, 1);
  SimJointInterface<kN> plain(b, kDt);
  Command cmd;
  cmd.tau << 5.0, -3.0, 2.0, 1.0, -0.5, 0.25, 0.1;
  ASSERT_TRUE(over.write(cmd).is_ok());
  ASSERT_TRUE(plain.write(cmd).is_ok());
  over.advance();
  plain.advance();
  plain.advance();
  EXPECT_NEAR(a.time_s(), b.time_s(), 1e-12);
  for (int j = 0; j < kN; ++j) {
    EXPECT_NEAR(a.q()(j), b.q()(j), 1e-9) << "joint " << j;
    EXPECT_NEAR(a.qd()(j), b.qd()(j), 1e-6) << "joint " << j;
  }
}

}  // namespace
}  // namespace cobot::pipeline::test
