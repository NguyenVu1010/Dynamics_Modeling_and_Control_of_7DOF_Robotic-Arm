// The black-box log survives a crash of its writer (feature blackbox-log, cobot_studio design 6.1
// "Hộp đen" and 7): a forked child records into a ShmStorage and is killed with SIGKILL; the
// parent exports what the region holds. Every record published before the kill is exported
// exactly once, in order, and no half-written row ever appears.
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <optional>
#include <poll.h>
#include <random>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/dynamics/sim_plant.hpp"
#include "cobot/pipeline/control_loop.hpp"
#include "cobot/pipeline/recorder.hpp"
#include "cobot/pipeline/recorder_shm.hpp"
#include "cobot/pipeline/sim_joint_interface.hpp"

#include "control_loop_test_util.hpp"
#include "shm_test_util.hpp"

namespace cobot::pipeline::shm_test {
namespace {

constexpr std::int64_t kStepNs = 1000000;  // 1 / 1000 Hz
constexpr std::uint64_t kNever = std::numeric_limits<std::uint64_t>::max();
constexpr auto kDeadline = std::chrono::seconds(120);  // generous for the ASan build

// A forked child that is always killed and reaped, also when the test fails early.
class Child {
 public:
  explicit Child(pid_t pid) : pid_(pid) {}
  Child(const Child&) = delete;
  Child& operator=(const Child&) = delete;
  Child(Child&&) = delete;
  Child& operator=(Child&&) = delete;
  ~Child() { static_cast<void>(kill_and_wait()); }

  bool alive() const {
    int status = 0;
    return pid_ > 0 && ::waitpid(pid_, &status, WNOHANG) == 0;
  }

  // SIGKILL and wait; the wait status, or -1 when already reaped.
  int kill_and_wait() {
    if (pid_ <= 0) {
      return -1;
    }
    ::kill(pid_, SIGKILL);
    int status = 0;
    ::waitpid(pid_, &status, 0);
    pid_ = -1;
    return status;
  }

 private:
  pid_t pid_;
};

bool killed_by_sigkill(int status) {
  return status >= 0 && WIFSIGNALED(status) && WTERMSIG(status) == SIGKILL;
}

// --------------------------------------------------------------- ControlLoop child

// ControlLoop + SimPlant (config/default.yaml without friction compensation, ideal plant: the rig
// of tests/component/control_loop) stepping without pacing into a kOverwriteOldest Recorder over
// `memory`, until it has recorded `stop_at` records (then it waits to be killed).
[[noreturn]] void loop_child(const test::Setup& setup,
                             RecorderMemory<kDof> memory,
                             std::uint64_t stop_at) {
  Recorder<kDof> recorder(memory, FullPolicy::kOverwriteOldest);
  dynamics::SimPlant<kDof> plant(setup.model, setup.config.home_q_rad);
  SimJointInterface<kDof> io(plant, 1.0 / setup.config.control_rate_hz);
  ControlLoop<kDof> loop(setup.model, setup.config, &recorder);
  test::State s;
  if (!io.read(s).is_ok() || !loop.start(s).is_ok()) {
    ::_exit(3);
  }
  for (;;) {
    if (recorder.recorded() >= stop_at) {
      ::pause();
      continue;
    }
    test::Command cmd;
    static_cast<void>(io.read(s));
    static_cast<void>(loop.step(s, cmd));
    static_cast<void>(io.write(cmd));
    io.advance();
  }
}

struct KillOutcome {
  std::uint64_t recorded = 0;  // read through open_existing after the kill
  DumpResult dump;
  CsvFile csv;
};

// Creates a region of `capacity`, forks the loop child, polls recorded() through open_existing
// until it reaches `k`, kills the child with SIGKILL, then dumps the region to `csv`.
std::optional<KillOutcome> run_loop_and_kill(const test::Setup& setup,
                                             std::size_t capacity,
                                             std::uint64_t k,
                                             std::uint64_t stop_at,
                                             const fs::path& csv) {
  const RegionGuard g;
  auto created = Storage::create(g.name(), capacity, false);
  if (!created.is_ok()) {
    ADD_FAILURE() << "create: " << created.reason();
    return std::nullopt;
  }
  const pid_t pid = ::fork();
  if (pid == 0) {
    loop_child(setup, created.value().memory(), stop_at);
  }
  Child child(pid);
  auto opened = Storage::open_existing(g.name());
  if (!opened.is_ok()) {
    ADD_FAILURE() << "open_existing: " << opened.reason();
    return std::nullopt;
  }
  const RecorderControl& control = *opened.value().memory().control;
  const auto deadline = std::chrono::steady_clock::now() + kDeadline;
  while (control.recorded.load(std::memory_order_acquire) < k) {
    if (!child.alive() || std::chrono::steady_clock::now() > deadline) {
      ADD_FAILURE() << "child stopped or too slow at " << control.recorded.load() << " < " << k;
      return std::nullopt;
    }
    std::this_thread::yield();
  }
  const int status = child.kill_and_wait();
  if (!killed_by_sigkill(status)) {
    ADD_FAILURE() << "child not killed by SIGKILL, status " << status;
    return std::nullopt;
  }
  KillOutcome out;
  out.recorded = control.recorded.load(std::memory_order_acquire);
  const auto dumped = dump_shm_log<kDof>(g.name(), csv, false);
  if (!dumped.is_ok()) {
    ADD_FAILURE() << "dump_shm_log: " << dumped.reason();
    return std::nullopt;
  }
  out.dump = dumped.value();
  out.csv = read_csv(csv);
  return out;
}

// t = first, first + 1 ms, ..., last (ns).
std::vector<std::int64_t> stamps_ms(std::int64_t first_ms, std::int64_t last_ms) {
  std::vector<std::int64_t> out;
  for (std::int64_t t = first_ms; t <= last_ms; ++t) {
    out.push_back(t * kStepNs);
  }
  return out;
}

// 10 kill points K drawn from mt19937_64(11) in [lo, hi].
std::vector<std::uint64_t> kill_points(std::uint64_t lo, std::uint64_t hi) {
  std::mt19937_64 rng(11);
  std::uniform_int_distribution<std::uint64_t> pick(lo, hi);
  std::vector<std::uint64_t> out;
  out.reserve(10);
  for (int i = 0; i < 10; ++i) {
    out.push_back(pick(rng));
  }
  return out;
}

constexpr std::size_t kLoopCapacity = 3000;

// K in [100, 2500] < capacity 3000: nothing wrapped. The child stops stepping at 3000 records so
// the run can never wrap before the kill. Every record published before the kill is exported:
// rows == recorded (read after the kill), t = 0, 1, 2, ... ms, every row complete and q finite.
TEST(ShmCrash, KilledLoopWithoutWrapExportsEveryPublishedRecord) {
  const auto setup = test::load_setup();
  ASSERT_TRUE(setup);
  const fs::path dir = out_dir("crash_no_wrap");
  for (const std::uint64_t k : kill_points(100, 2500)) {
    SCOPED_TRACE("K = " + std::to_string(k));
    const auto out = run_loop_and_kill(
        *setup, kLoopCapacity, k, kLoopCapacity, dir / ("k" + std::to_string(k) + ".csv"));
    ASSERT_TRUE(out);
    ASSERT_GE(out->recorded, k);
    ASSERT_LE(out->recorded, kLoopCapacity);
    EXPECT_EQ(out->dump.rows, out->recorded);
    EXPECT_EQ(out->dump.lost, 0U);
    EXPECT_EQ(out->csv.header_lines, 1);
    ASSERT_EQ(out->csv.rows.size(), out->recorded);
    EXPECT_EQ(csv_stamps(out->csv), stamps_ms(0, static_cast<std::int64_t>(out->recorded) - 1));
    expect_rows_well_formed(out->csv, false);
  }
}

// K in [6000, 9000] > 2 * capacity: the buffer wrapped. The dump has the newest capacity - 1
// records (the slot the next record() would overwrite is skipped): with R recorded (record i at
// t = (i - 1) ms), rows are records R - 2998 .. R, t = (R - 2999) .. (R - 1) ms, contiguous;
// lost = R - 2999.
TEST(ShmCrash, KilledLoopAfterWrapExportsTheNewestCapacityMinusOne) {
  const auto setup = test::load_setup();
  ASSERT_TRUE(setup);
  const fs::path dir = out_dir("crash_wrapped");
  for (const std::uint64_t k : kill_points(6000, 9000)) {
    SCOPED_TRACE("K = " + std::to_string(k));
    const auto out = run_loop_and_kill(
        *setup, kLoopCapacity, k, kNever, dir / ("k" + std::to_string(k) + ".csv"));
    ASSERT_TRUE(out);
    ASSERT_GE(out->recorded, k);
    const auto r = static_cast<std::int64_t>(out->recorded);
    const auto cap = static_cast<std::int64_t>(kLoopCapacity);
    EXPECT_EQ(out->dump.rows, kLoopCapacity - 1);
    EXPECT_EQ(out->dump.lost, out->recorded - (kLoopCapacity - 1));
    ASSERT_EQ(out->csv.rows.size(), kLoopCapacity - 1);
    EXPECT_EQ(csv_stamps(out->csv), stamps_ms(r - cap + 1, r - 1));
    expect_rows_well_formed(out->csv, false);
  }
}

// --------------------------------------------------------------- swap child

// Writes one byte to the parent and waits to be killed.
[[noreturn]] void signal_and_wait(int fd, char byte) {
  if (::write(fd, &byte, 1) != 1) {
    ::_exit(4);
  }
  for (;;) {
    ::pause();
  }
}

// Waits up to kDeadline for the child's byte; '\0' when none came.
char wait_byte(int fd) {
  pollfd p{fd, POLLIN, 0};
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(kDeadline).count();
  if (::poll(&p, 1, static_cast<int>(ms)) != 1) {
    return '\0';
  }
  char byte = '\0';
  return ::read(fd, &byte, 1) == 1 ? byte : '\0';
}

enum class SwapKill { kBeforeAcknowledge, kAfterAcknowledgeBeforeExport };

// Child, as the controller with ~/dump_log: records 1 .. 500; its exporter thread requests a swap;
// records 501 .. 800 (record 501 performs the swap); the exporter appends retired() (1 .. 500) to
// `run_csv` and releases it. Then the exporter requests a second swap and
//  (a) kBeforeAcknowledge: no record follows, the swap stays pending (active buffer 501 .. 800);
//  (b) kAfterAcknowledgeBeforeExport: records 801 .. 1000 follow (record 801 performs the swap,
//      retired buffer 501 .. 800 not exported, active buffer 801 .. 1000).
// It then signals 'k' (or 'x' when a step went wrong) and waits to be killed.
[[noreturn]] void swap_child(RecorderMemory<kDof> memory,
                             SwapKill when,
                             const fs::path& run_csv,
                             int fd) {
  Recorder<kDof> rec(memory, FullPolicy::kOverwriteOldest);
  std::int64_t t = 0;
  const auto record_to = [&](std::int64_t last) {
    while (t < last) {
      ++t;
      rec.record(make_record(t));
    }
  };
  bool ok = true;
  record_to(500);
  std::thread([&] { ok = rec.request_swap() && ok; }).join();
  record_to(800);
  std::thread([&] {
    ok = rec.swap_done() && ok;
    ok = export_csv<kDof>(run_csv, rec.retired(), true).is_ok() && ok;
    rec.release_retired();
  }).join();
  std::thread([&] { ok = rec.request_swap() && ok; }).join();
  if (when == SwapKill::kAfterAcknowledgeBeforeExport) {
    record_to(1000);
    std::thread([&] { ok = rec.swap_done() && ok; }).join();
  } else {
    ok = !rec.swap_done() && ok;
  }
  signal_and_wait(fd, ok ? 'k' : 'x');
}

void run_swap_kill(SwapKill when, const std::string& test, std::int64_t last) {
  const RegionGuard g;
  const fs::path dir = out_dir(test);
  auto created = Storage::create(g.name(), 1000, false);
  ASSERT_TRUE(created.is_ok()) << created.reason();
  int fds[2] = {-1, -1};
  ASSERT_EQ(::pipe(fds), 0);
  const pid_t pid = ::fork();
  if (pid == 0) {
    ::close(fds[0]);
    swap_child(created.value().memory(), when, dir / "run.csv", fds[1]);
  }
  ::close(fds[1]);
  Child child(pid);
  const char byte = wait_byte(fds[0]);
  ::close(fds[0]);
  ASSERT_EQ(byte, 'k') << "child did not reach the kill point cleanly";
  ASSERT_TRUE(killed_by_sigkill(child.kill_and_wait()));

  const auto dumped = dump_shm_log<kDof>(g.name(), dir / "dump.csv", false);
  ASSERT_TRUE(dumped.is_ok()) << dumped.reason();
  const CsvFile run = read_csv(dir / "run.csv");
  const CsvFile dump = read_csv(dir / "dump.csv");
  // The controller's file has 1 .. 500; the dump has the rest, 501 .. last, each once, in order.
  EXPECT_EQ(csv_stamps(run), range(1, 500));
  EXPECT_EQ(dumped.value().rows, static_cast<std::size_t>(last - 500));
  EXPECT_EQ(dumped.value().lost, 0U);
  EXPECT_EQ(csv_stamps(dump), range(501, last));
  expect_rows_well_formed(dump, true);
  // Nothing left: a second dump is empty.
  const auto again = dump_shm_log<kDof>(g.name(), dir / "again.csv", false);
  ASSERT_TRUE(again.is_ok()) << again.reason();
  EXPECT_EQ(again.value().rows, 0U);
}

TEST(ShmCrash, KilledWithASwapPendingExportsEveryRowOnce) {
  run_swap_kill(SwapKill::kBeforeAcknowledge, "crash_swap_pending", 800);
}

TEST(ShmCrash, KilledAfterTheSwapBeforeItsExportExportsEveryRowOnce) {
  run_swap_kill(SwapKill::kAfterAcknowledgeBeforeExport, "crash_swap_done", 1000);
}

// --------------------------------------------------------------- recovery after a kill

// Child records 1 .. 123 into a region of 200 and is killed; at the next start-up
// recover_shm_log writes "<log>.recovered-YYYYmmdd-HHMMSS.csv" with all 123 rows and removes the
// region; a second call finds nothing.
TEST(ShmCrash, RecoveryAfterAKillWritesEveryRowOnce) {
  const RegionGuard g;
  const fs::path dir = out_dir("crash_recover");
  const fs::path log_path = dir / "run.csv";
  {
    auto created = Storage::create(g.name(), 200, false);
    ASSERT_TRUE(created.is_ok()) << created.reason();
    int fds[2] = {-1, -1};
    ASSERT_EQ(::pipe(fds), 0);
    const pid_t pid = ::fork();
    if (pid == 0) {
      ::close(fds[0]);
      Recorder<kDof> rec(created.value().memory(), FullPolicy::kOverwriteOldest);
      for (std::int64_t t = 1; t <= 123; ++t) {
        rec.record(make_record(t));
      }
      signal_and_wait(fds[1], 'k');
    }
    ::close(fds[1]);
    Child child(pid);
    const char byte = wait_byte(fds[0]);
    ::close(fds[0]);
    ASSERT_EQ(byte, 'k');
    ASSERT_TRUE(killed_by_sigkill(child.kill_and_wait()));
  }
  const auto r = recover_shm_log<kDof>(g.name(), log_path);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  ASSERT_TRUE(r.value().has_value());
  const fs::path recovered = *r.value();
  EXPECT_EQ(recovered.parent_path(), dir);
  EXPECT_TRUE(is_stamped_name(recovered.filename().string(), "run.csv.recovered-", ".csv"))
      << recovered;
  const CsvFile f = read_csv(recovered);
  EXPECT_EQ(f.header_lines, 1);
  EXPECT_EQ(csv_stamps(f), range(1, 123));
  expect_rows_well_formed(f, true);
  EXPECT_FALSE(Storage::exists(g.name()));
  EXPECT_FALSE(g.present());

  const auto again = recover_shm_log<kDof>(g.name(), log_path);
  ASSERT_TRUE(again.is_ok()) << again.reason();
  EXPECT_FALSE(again.value().has_value());
  EXPECT_FALSE(Storage::exists(g.name()));
}

}  // namespace
}  // namespace cobot::pipeline::shm_test
