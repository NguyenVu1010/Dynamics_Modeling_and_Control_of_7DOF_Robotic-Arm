#include <cstddef>
#include <cstdint>

#include <gtest/gtest.h>

#include "cobot/pipeline/control_loop.hpp"
#include "cobot/pipeline/recorder.hpp"

#include "loop_recorder_util.hpp"

// ControlLoop writing into a host-owned Recorder (feature pipeline-recorder, cobot_studio design
// 6.1): step() records the row it also keeps as last_record(); the recorder changes nothing in
// the control output; a ControlLoop rebuilt on the same Recorder continues the same log. Set-up
// of E2E-01 (config/default.yaml without friction compensation, ideal plant).
namespace cobot::pipeline::test {
namespace {

constexpr std::int64_t kStepNs = 1000000;  // 1 / 1000 Hz = 1 ms

// 2000 steps into a recorder of 2000 rows: one row per step, row k bit-equal to last_record()
// after step k.
TEST(ControlLoopRecorder, RecordsEveryStepAsLastRecord) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  constexpr int kSteps = 2000;
  HeapStorage<kN> storage(kSteps, 1);
  Recorder<kN> recorder(storage.memory(), FullPolicy::kStopWhenFull);
  const ClosedLoopRun run = run_closed_loop(*setup, &recorder, kSteps);
  EXPECT_TRUE(run.all_ok);
  EXPECT_EQ(recorder.dropped(), 0U);
  EXPECT_EQ(recorder.recorded(), static_cast<std::uint64_t>(kSteps));
  ASSERT_EQ(recorder.size(), static_cast<std::size_t>(kSteps));
  const RecordSpan<kN> rows = recorder.active();
  ASSERT_EQ(rows.size(), static_cast<std::size_t>(kSteps));
  for (std::size_t k = 0; k < rows.size(); ++k) {
    ASSERT_TRUE(same_record(rows[k], run.last_records[k])) << "row " << k;
    ASSERT_EQ(rows[k].t.ns, static_cast<std::int64_t>(k) * kStepNs) << "row " << k;
    ASSERT_EQ(rows[k].tau, run.commands[k].tau) << "row " << k;
  }
}

// The same run without a recorder: the commands and last_record() are bit-equal.
TEST(ControlLoopRecorder, NoRecorderGivesTheSameControlOutput) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  constexpr int kSteps = 2000;
  HeapStorage<kN> storage(kSteps, 1);
  Recorder<kN> recorder(storage.memory(), FullPolicy::kStopWhenFull);
  const ClosedLoopRun with = run_closed_loop(*setup, &recorder, kSteps);
  const ClosedLoopRun without = run_closed_loop(*setup, nullptr, kSteps);
  EXPECT_TRUE(with.all_ok);
  EXPECT_TRUE(without.all_ok);
  ASSERT_EQ(with.commands.size(), without.commands.size());
  for (std::size_t k = 0; k < with.commands.size(); ++k) {
    ASSERT_TRUE(same_bits(with.commands[k].tau, without.commands[k].tau)) << "step " << k;
    ASSERT_TRUE(same_record(with.last_records[k], without.last_records[k])) << "step " << k;
  }
}

// Loop A runs 100 steps, then loop B is built on the same recorder (as a host applying a new
// configuration) and runs 100 more on the same plant: 200 rows, t increasing, A's rows first.
TEST(ControlLoopRecorder, RebuiltLoopContinuesTheSameLog) {
  const auto setup = load_setup();
  ASSERT_TRUE(setup);
  constexpr int kEach = 100;
  HeapStorage<kN> storage(1000, 1);
  Recorder<kN> recorder(storage.memory(), FullPolicy::kStopWhenFull);
  dynamics::SimPlant<kN> plant(setup->model, setup->config.home_q_rad);
  SimJointInterface<kN> io(plant, 1.0 / setup->config.control_rate_hz);
  State s;
  Command cmd;
  Record last_of_a;
  {
    ControlLoop<kN> a(setup->model, setup->config, &recorder);
    ASSERT_TRUE(io.read(s).is_ok());
    ASSERT_TRUE(a.start(s).is_ok());
    for (int k = 0; k < kEach; ++k) {
      ASSERT_TRUE(io.read(s).is_ok());
      ASSERT_TRUE(a.step(s, cmd).is_ok()) << "A step " << k;
      ASSERT_TRUE(io.write(cmd).is_ok());
      io.advance();
    }
    last_of_a = a.last_record();
  }
  EXPECT_EQ(recorder.size(), static_cast<std::size_t>(kEach));
  ControlLoop<kN> b(setup->model, setup->config, &recorder);
  ASSERT_TRUE(io.read(s).is_ok());
  ASSERT_TRUE(b.start(s).is_ok());
  for (int k = 0; k < kEach; ++k) {
    ASSERT_TRUE(io.read(s).is_ok());
    ASSERT_TRUE(b.step(s, cmd).is_ok()) << "B step " << k;
    ASSERT_TRUE(io.write(cmd).is_ok());
    io.advance();
  }
  const RecordSpan<kN> rows = recorder.active();
  ASSERT_EQ(rows.size(), static_cast<std::size_t>(2 * kEach));
  EXPECT_EQ(recorder.recorded(), static_cast<std::uint64_t>(2 * kEach));
  for (std::size_t k = 1; k < rows.size(); ++k) {
    EXPECT_GT(rows[k].t.ns, rows[k - 1].t.ns) << "row " << k;
  }
  // The plant clock runs on through both loops: row k at k ms (B's start() reads the state of
  // step 100 again without advancing the plant).
  for (std::size_t k = 0; k < rows.size(); ++k) {
    EXPECT_EQ(rows[k].t.ns, static_cast<std::int64_t>(k) * kStepNs) << "row " << k;
  }
  EXPECT_TRUE(same_record(rows[kEach - 1], last_of_a));
  EXPECT_TRUE(same_record(rows[rows.size() - 1], b.last_record()));
}

}  // namespace
}  // namespace cobot::pipeline::test
