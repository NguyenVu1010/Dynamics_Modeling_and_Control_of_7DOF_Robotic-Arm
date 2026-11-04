// ShmStorage<N>, dump_shm_log and recover_shm_log in one process (feature blackbox-log,
// cobot_studio design 6.1 "Hộp đen", ADR-020): region life cycle, layout checks, the /dev/shm
// reservation, exactly-once export and start-up recovery. The crash tests (fork + SIGKILL) are in
// test_recorder_shm_crash.cpp, the allocation test in test_recorder_shm_no_alloc.cpp.
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <optional>
#include <string>
#include <sys/statvfs.h>
#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "cobot/pipeline/recorder.hpp"
#include "cobot/pipeline/recorder_shm.hpp"

#include "shm_test_util.hpp"

namespace cobot::pipeline::shm_test {
namespace {

constexpr std::size_t kMiB = std::size_t{1024} * 1024;

std::string lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return s;
}

bool contains(const std::string& text, const std::string& part) {
  return text.find(part) != std::string::npos;
}

// Records t = first .. last into `rec`.
void record_range(Recorder<kDof>& rec, std::int64_t first, std::int64_t last) {
  for (std::int64_t t = first; t <= last; ++t) {
    rec.record(make_record(t));
  }
}

// --------------------------------------------------------------- type and fingerprint

static_assert(!std::is_copy_constructible_v<Storage>, "ShmStorage is move-only");
static_assert(!std::is_copy_assignable_v<Storage>, "ShmStorage is move-only");
static_assert(std::is_move_constructible_v<Storage>, "ShmStorage is movable");
static_assert(std::is_standard_layout_v<LogRecord<kDof>>, "ADR-020: LogRecord in shared memory");

TEST(ShmLogFingerprint, IsConstexprStableAndDiffersPerDof) {
  constexpr std::uint64_t kF1 = log_record_fingerprint<1>();
  constexpr std::uint64_t kF2 = log_record_fingerprint<2>();
  constexpr std::uint64_t kF6 = log_record_fingerprint<6>();
  constexpr std::uint64_t kF7 = log_record_fingerprint<7>();
  EXPECT_NE(kF6, kF7);
  EXPECT_NE(kF1, kF2);
  EXPECT_NE(kF2, kF7);
  EXPECT_NE(kF1, kF7);
  EXPECT_EQ(kF7, log_record_fingerprint<7>());
  EXPECT_GE(kShmLogLayoutVersion, 1U);
}

// --------------------------------------------------------------- life cycle

TEST(ShmStorage, CreateThenOpenExistingSeesTheSameRecords) {
  const RegionGuard g;
  EXPECT_FALSE(Storage::exists(g.name()));
  auto created = Storage::create(g.name(), 37, false);
  ASSERT_TRUE(created.is_ok()) << created.reason();
  EXPECT_TRUE(Storage::exists(g.name()));
  EXPECT_TRUE(g.present()) << g.dev_shm_path();
  EXPECT_TRUE(created.value().lock_error().empty()) << "no mlock requested";

  const RecorderMemory<kDof> m = created.value().memory();
  ASSERT_NE(m.control, nullptr);
  ASSERT_NE(m.buffers[0], nullptr);
  ASSERT_NE(m.buffers[1], nullptr);
  ASSERT_NE(m.sequence, nullptr);
  EXPECT_EQ(m.capacity, 37U);
  EXPECT_EQ(m.buffer_count, 2);
  // The two buffers do not overlap: 37 records each.
  EXPECT_TRUE(m.buffers[1] >= m.buffers[0] + 37 || m.buffers[0] >= m.buffers[1] + 37);

  Recorder<kDof> rec(m, FullPolicy::kOverwriteOldest);
  record_range(rec, 1, 20);

  auto opened = Storage::open_existing(g.name());
  ASSERT_TRUE(opened.is_ok()) << opened.reason();
  const RecorderMemory<kDof> o = opened.value().memory();
  ASSERT_NE(o.control, nullptr);
  EXPECT_EQ(o.capacity, 37U);
  EXPECT_EQ(o.buffer_count, 2);
  EXPECT_EQ(o.control->recorded.load(), 20U);
  EXPECT_EQ(o.control->written[0].load(), 20U);
  for (int i = 0; i < 20; ++i) {
    const Rec& got = o.buffers[0][i];
    const Rec want = make_record(i + 1);
    EXPECT_EQ(got.t.ns, want.t.ns) << "slot " << i;
    EXPECT_EQ(got.q, want.q) << "slot " << i;
    EXPECT_EQ(got.tau, want.tau) << "slot " << i;
    EXPECT_EQ(got.hold_reason, want.hold_reason) << "slot " << i;
  }
  // Same memory: a record made through the first mapping shows through the second.
  rec.record(make_record(21));
  EXPECT_EQ(o.control->recorded.load(), 21U);
  EXPECT_EQ(o.buffers[0][20].t.ns, 21);
}

TEST(ShmStorage, CreateOnAnExistingNameFails) {
  const RegionGuard g;
  auto first = Storage::create(g.name(), 16, false);
  ASSERT_TRUE(first.is_ok()) << first.reason();
  Recorder<kDof> rec(first.value().memory(), FullPolicy::kOverwriteOldest);
  record_range(rec, 1, 5);
  const auto second = Storage::create(g.name(), 16, false);
  EXPECT_FALSE(second.is_ok());
  // The failed create left the existing region and its records alone.
  EXPECT_TRUE(Storage::exists(g.name()));
  EXPECT_EQ(first.value().memory().control->recorded.load(), 5U);
}

TEST(ShmStorage, RemoveTwiceIsOkAndExistsFollows) {
  const RegionGuard g;
  ASSERT_TRUE(Storage::create(g.name(), 8, false).is_ok());
  ASSERT_TRUE(Storage::exists(g.name()));
  const common::Status first = Storage::remove(g.name());
  EXPECT_TRUE(first.is_ok()) << first.reason();
  EXPECT_FALSE(Storage::exists(g.name()));
  EXPECT_FALSE(g.present());
  const common::Status second = Storage::remove(g.name());
  EXPECT_TRUE(second.is_ok()) << "missing is ok: " << second.reason();
  // The name can be created again once removed.
  EXPECT_TRUE(Storage::create(g.name(), 8, false).is_ok());
}

TEST(ShmStorage, DestructorUnmapsButNeverUnlinks) {
  const RegionGuard g;
  {
    auto created = Storage::create(g.name(), 8, false);
    ASSERT_TRUE(created.is_ok()) << created.reason();
    Recorder<kDof> rec(created.value().memory(), FullPolicy::kOverwriteOldest);
    record_range(rec, 1, 3);
    Storage moved = std::move(created.value());  // move-only, the moved-to object owns it
    EXPECT_EQ(moved.memory().control->recorded.load(), 3U);
  }
  EXPECT_TRUE(Storage::exists(g.name()));
  auto opened = Storage::open_existing(g.name());
  ASSERT_TRUE(opened.is_ok()) << opened.reason();
  EXPECT_EQ(opened.value().memory().control->recorded.load(), 3U);
}

// Two buffers and the per-slot sequence (2 * capacity records + 2 * capacity uint64) are all
// in the region, and the pages are reserved (posix_fallocate) at create time.
TEST(ShmStorage, RegionHoldsTwoBuffersAndSequenceReservedUpFront) {
  const RegionGuard g;
  constexpr std::size_t kCapacity = 1000;
  auto created = Storage::create(g.name(), kCapacity, false);
  ASSERT_TRUE(created.is_ok()) << created.reason();
  const std::size_t need = 2 * kCapacity * (sizeof(Rec) + sizeof(std::uint64_t));
  struct stat st {};
  ASSERT_EQ(::stat(g.dev_shm_path().c_str(), &st), 0) << g.dev_shm_path();
  EXPECT_GE(static_cast<std::size_t>(st.st_size), need);
  // tmpfs counts reserved pages in st_blocks (512-byte units).
  EXPECT_GE(static_cast<std::size_t>(st.st_blocks) * 512, need) << "pages not reserved";
}

TEST(ShmStorage, LockMemoryIsAWarningNotAnError) {
  const RegionGuard g;
  auto created = Storage::create(g.name(), 64, true);
  ASSERT_TRUE(created.is_ok()) << created.reason();
  // mlock may fail under RLIMIT_MEMLOCK; then lock_error() says so and the storage works.
  Recorder<kDof> rec(created.value().memory(), FullPolicy::kOverwriteOldest);
  record_range(rec, 1, 10);
  EXPECT_EQ(rec.recorded(), 10U);
  if (!created.value().lock_error().empty()) {
    std::printf("lock_error: %s\n", created.value().lock_error().c_str());
  }
}

TEST(ShmStorage, BadNamesAndZeroCapacityAreErrors) {
  const std::vector<std::string> bad = {
      "", "a-b", "a/b", "a b", "../x", "a.b", std::string(65, 'a')};
  for (const std::string& name : bad) {
    const auto r = Storage::create(name, 8, false);
    EXPECT_FALSE(r.is_ok()) << "name '" << name << "'";
    if (!name.empty() && name.find('/') == std::string::npos) {
      static_cast<void>(::shm_unlink(region_of(name).c_str()));
    }
  }
  // 64 characters of [A-Za-z0-9_] is the longest valid name.
  std::string longest = unique_name();
  longest += std::string(64 - longest.size(), 'x');
  const RegionGuard g(longest);
  const auto ok = Storage::create(g.name(), 8, false);
  EXPECT_TRUE(ok.is_ok()) << ok.reason();

  const RegionGuard zero;
  EXPECT_FALSE(Storage::create(zero.name(), 0, false).is_ok());
  EXPECT_FALSE(Storage::exists(zero.name()));
}

TEST(ShmStorage, OpenExistingOnAMissingRegionFails) {
  const RegionGuard g;
  EXPECT_FALSE(Storage::open_existing(g.name()).is_ok());
  EXPECT_FALSE(Storage::exists(g.name()));
}

// --------------------------------------------------------------- layout checks

// Header offsets found by value in a raw mapping of a fresh region. The byte layout is the
// builder's (design 9); the tests rely only on: the magic starts at offset 0 (it is the first
// header field, design 6.1), the layout version follows it as a 4- or 8-byte integer, then N,
// the record size (4 or 8 bytes) and the 64-bit fingerprint, all within the first bytes.
struct HeaderOffsets {
  std::optional<std::size_t> version;
  std::optional<std::size_t> record_size;
  std::optional<std::size_t> fingerprint;
};

HeaderOffsets find_header(const RawRegion& raw) {
  HeaderOffsets h;
  h.version = raw.find(static_cast<std::uint32_t>(kShmLogLayoutVersion), 4, 64, 4);
  const std::size_t after_version = h.version ? *h.version + 4 : 4;
  h.record_size = raw.find(static_cast<std::uint32_t>(sizeof(Rec)), after_version, 256, 4);
  h.fingerprint = raw.find(log_record_fingerprint<kDof>(), 0, 4096, 4);
  return h;
}

// A region of capacity 37 (37 differs from N, sizeof(LogRecord<7>) and the version, so a value
// search cannot pick the capacity field by mistake), its storage already unmapped.
void make_region(const RegionGuard& g) {
  auto created = Storage::create(g.name(), 37, false);
  ASSERT_TRUE(created.is_ok()) << created.reason();
  Recorder<kDof> rec(created.value().memory(), FullPolicy::kOverwriteOldest);
  record_range(rec, 1, 5);
}

TEST(ShmLayout, FreshRegionOpens) {
  const RegionGuard g;
  make_region(g);
  const auto r = Storage::open_existing(g.name());
  EXPECT_TRUE(r.is_ok()) << r.reason();
}

TEST(ShmLayout, OtherDofIsRejectedNamingBothValues) {
  const RegionGuard g;
  ASSERT_TRUE(ShmStorage<2>::create(g.name(), 37, false).is_ok());
  const auto r = Storage::open_existing(g.name());
  ASSERT_FALSE(r.is_ok());
  // "dof: region 2, reader 7"
  EXPECT_TRUE(contains(lower(r.reason()), "dof")) << r.reason();
  EXPECT_TRUE(contains(r.reason(), "2")) << r.reason();
  EXPECT_TRUE(contains(r.reason(), "7")) << r.reason();
  // And the other way round.
  const RegionGuard h;
  make_region(h);
  const auto back = ShmStorage<2>::open_existing(h.name());
  ASSERT_FALSE(back.is_ok());
  EXPECT_TRUE(contains(lower(back.reason()), "dof")) << back.reason();
}

TEST(ShmLayout, CorruptMagicIsRejected) {
  const RegionGuard g;
  make_region(g);
  {
    RawRegion raw(g.name());
    ASSERT_TRUE(raw.ok());
    raw.bytes()[0] ^= 0xFFU;
  }
  const auto r = Storage::open_existing(g.name());
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(lower(r.reason()), "magic")) << r.reason();
}

TEST(ShmLayout, OtherLayoutVersionIsRejectedNamingBothValues) {
  const RegionGuard g;
  make_region(g);
  const std::uint32_t v = kShmLogLayoutVersion;
  {
    RawRegion raw(g.name());
    ASSERT_TRUE(raw.ok());
    const HeaderOffsets h = find_header(raw);
    ASSERT_TRUE(h.version) << "layout version " << v << " not found after the magic";
    raw.write(*h.version, v + 1);
  }
  const auto r = Storage::open_existing(g.name());
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(lower(r.reason()), "version")) << r.reason();
  EXPECT_TRUE(contains(r.reason(), std::to_string(v + 1))) << r.reason();
  EXPECT_TRUE(contains(r.reason(), std::to_string(v))) << r.reason();
}

TEST(ShmLayout, OtherRecordSizeIsRejectedNamingBothValues) {
  const RegionGuard g;
  make_region(g);
  const auto size = static_cast<std::uint32_t>(sizeof(Rec));
  {
    RawRegion raw(g.name());
    ASSERT_TRUE(raw.ok());
    const HeaderOffsets h = find_header(raw);
    ASSERT_TRUE(h.record_size) << "record size " << size << " not found in the header";
    raw.write(*h.record_size, size + 8);
  }
  const auto r = Storage::open_existing(g.name());
  ASSERT_FALSE(r.is_ok());
  const std::string why = lower(r.reason());
  EXPECT_TRUE(contains(why, "record") && contains(why, "size")) << r.reason();
  EXPECT_TRUE(contains(r.reason(), std::to_string(size + 8))) << r.reason();
  EXPECT_TRUE(contains(r.reason(), std::to_string(size))) << r.reason();
}

TEST(ShmLayout, OtherFingerprintIsRejected) {
  const RegionGuard g;
  make_region(g);
  {
    RawRegion raw(g.name());
    ASSERT_TRUE(raw.ok());
    const HeaderOffsets h = find_header(raw);
    ASSERT_TRUE(h.fingerprint) << "fingerprint not found in the header";
    raw.write(*h.fingerprint, ~log_record_fingerprint<kDof>());
  }
  const auto r = Storage::open_existing(g.name());
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(lower(r.reason()), "fingerprint")) << r.reason();
}

// Checks run in the order magic, version, N, record size, fingerprint: with both the version and
// the fingerprint wrong, the version is named.
TEST(ShmLayout, VersionIsCheckedBeforeFingerprint) {
  const RegionGuard g;
  make_region(g);
  {
    RawRegion raw(g.name());
    ASSERT_TRUE(raw.ok());
    const HeaderOffsets h = find_header(raw);
    ASSERT_TRUE(h.version && h.fingerprint);
    raw.write(*h.fingerprint, ~log_record_fingerprint<kDof>());
    raw.write(*h.version, static_cast<std::uint32_t>(kShmLogLayoutVersion + 1));
  }
  const auto r = Storage::open_existing(g.name());
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(lower(r.reason()), "version")) << r.reason();
  EXPECT_FALSE(contains(lower(r.reason()), "fingerprint")) << r.reason();
}

// A region shorter than its header says (capacity/size consistency) is an error, not a SIGBUS.
TEST(ShmLayout, TruncatedRegionIsRejected) {
  const RegionGuard g;
  make_region(g);
  {
    const RawRegion raw(g.name());
    ASSERT_TRUE(raw.ok());
    ASSERT_TRUE(raw.truncate(raw.size() / 2));
  }
  EXPECT_FALSE(Storage::open_existing(g.name()).is_ok());
  {
    const RawRegion raw(g.name());
    ASSERT_TRUE(raw.ok());
    ASSERT_TRUE(raw.truncate(16));
  }
  EXPECT_FALSE(Storage::open_existing(g.name()).is_ok());
  const fs::path csv = out_dir("truncated") / "dump.csv";
  EXPECT_FALSE(dump_shm_log<kDof>(g.name(), csv, false).is_ok());
}

// --------------------------------------------------------------- /dev/shm reservation

// A region larger than the free space of /dev/shm fails at create (posix_fallocate ENOSPC), never
// later with a SIGBUS, and leaves no region. Skipped where /dev/shm has more than 256 MB free:
// tmpfs would fill memory before failing. Runs in the arm64 build stage (64 MB) and in Docker
// (64 MB by default).
TEST(ShmStorage, RegionLargerThanDevShmFailsAtCreateAndLeavesNoRegion) {
  struct statvfs vfs {};
  ASSERT_EQ(::statvfs("/dev/shm", &vfs), 0);
  const std::uint64_t free_bytes = static_cast<std::uint64_t>(vfs.f_bavail) * vfs.f_frsize;
  if (free_bytes > 256 * kMiB) {
    GTEST_SKIP() << "/dev/shm has " << free_bytes / kMiB << " MB free (> 256 MB)";
  }
  // Region >= 2 * capacity * sizeof(Rec) >= free + 1 MB + 2 * sizeof(Rec) > free + 1 MB.
  const std::size_t capacity =
      static_cast<std::size_t>((free_bytes + kMiB) / (2 * sizeof(Rec))) + 1;
  const RegionGuard g;
  const auto r = Storage::create(g.name(), capacity, false);
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(r.reason(), "/dev/shm too small")) << r.reason();
  EXPECT_TRUE(contains(r.reason(), "need ") && contains(r.reason(), " MB")) << r.reason();
  EXPECT_FALSE(Storage::exists(g.name()));
  EXPECT_FALSE(g.present());
}

// --------------------------------------------------------------- dump_shm_log, one process

TEST(ShmDump, MissingRegionIsAnError) {
  const RegionGuard g;
  const fs::path csv = out_dir("dump_missing") / "dump.csv";
  EXPECT_FALSE(dump_shm_log<kDof>(g.name(), csv, false).is_ok());
}

TEST(ShmDump, ExportsAllRowsOnceThenNothing) {
  const RegionGuard g;
  const fs::path dir = out_dir("dump_once");
  auto created = Storage::create(g.name(), 100, false);
  ASSERT_TRUE(created.is_ok()) << created.reason();
  Recorder<kDof> rec(created.value().memory(), FullPolicy::kOverwriteOldest);
  record_range(rec, 1, 40);

  const auto first = dump_shm_log<kDof>(g.name(), dir / "a.csv", false);
  ASSERT_TRUE(first.is_ok()) << first.reason();
  EXPECT_EQ(first.value().csv, dir / "a.csv");
  EXPECT_EQ(first.value().rows, 40U);
  EXPECT_EQ(first.value().lost, 0U);
  const CsvFile a = read_csv(dir / "a.csv");
  EXPECT_EQ(a.header_lines, 1);
  EXPECT_EQ(csv_stamps(a), range(1, 40));
  expect_rows_well_formed(a, true);

  // Everything was marked exported: a second dump has nothing (cobot_logdump exit 1).
  const auto second = dump_shm_log<kDof>(g.name(), dir / "b.csv", false);
  ASSERT_TRUE(second.is_ok()) << second.reason();
  EXPECT_EQ(second.value().rows, 0U);
  if (fs::exists(dir / "b.csv")) {
    EXPECT_TRUE(read_csv(dir / "b.csv").rows.empty());
  }
  // The region stays (only --remove or recovery removes it).
  EXPECT_TRUE(Storage::exists(g.name()));
}

TEST(ShmDump, KeepLeavesTheRowsForTheNextDump) {
  const RegionGuard g;
  const fs::path dir = out_dir("dump_keep");
  auto created = Storage::create(g.name(), 100, false);
  ASSERT_TRUE(created.is_ok()) << created.reason();
  Recorder<kDof> rec(created.value().memory(), FullPolicy::kOverwriteOldest);
  record_range(rec, 1, 25);
  for (const char* file : {"k1.csv", "k2.csv"}) {
    const auto kept = dump_shm_log<kDof>(g.name(), dir / file, true);
    ASSERT_TRUE(kept.is_ok()) << kept.reason();
    EXPECT_EQ(kept.value().rows, 25U) << file;
    EXPECT_EQ(csv_stamps(read_csv(dir / file)), range(1, 25)) << file;
  }
  const auto taken = dump_shm_log<kDof>(g.name(), dir / "k3.csv", false);
  ASSERT_TRUE(taken.is_ok()) << taken.reason();
  EXPECT_EQ(taken.value().rows, 25U);
  const auto none = dump_shm_log<kDof>(g.name(), dir / "k4.csv", false);
  ASSERT_TRUE(none.is_ok()) << none.reason();
  EXPECT_EQ(none.value().rows, 0U);
}

// The recorder keeps writing after a dump: the next dump appends only the new rows to the same
// file, with one header.
TEST(ShmDump, LaterDumpAppendsOnlyNewRows) {
  const RegionGuard g;
  const fs::path csv = out_dir("dump_append") / "log.csv";
  auto created = Storage::create(g.name(), 100, false);
  ASSERT_TRUE(created.is_ok()) << created.reason();
  Recorder<kDof> rec(created.value().memory(), FullPolicy::kOverwriteOldest);
  record_range(rec, 1, 40);
  const auto first = dump_shm_log<kDof>(g.name(), csv, false);
  ASSERT_TRUE(first.is_ok()) << first.reason();
  EXPECT_EQ(first.value().rows, 40U);
  record_range(rec, 41, 70);
  const auto second = dump_shm_log<kDof>(g.name(), csv, false);
  ASSERT_TRUE(second.is_ok()) << second.reason();
  EXPECT_EQ(second.value().rows, 30U);  // 41 .. 70
  const CsvFile f = read_csv(csv);
  EXPECT_EQ(f.header_lines, 1);
  EXPECT_EQ(csv_stamps(f), range(1, 70));
  expect_rows_well_formed(f, true);
}

// Capacity 10, 25 records into one buffer (no swap): the slots hold records 16 .. 25; the slot
// the next record() overwrites (record 16, (16 - 1) % 10 = slot 5 = 25 % 10) is never exported,
// so the dump has 17 .. 25 = 9 rows = capacity - 1, and 25 - 9 = 16 records are lost (rows + lost
// = recorded). Then records 26 .. 30 overwrite 16 .. 20 (all exported or lost already): the next
// dump has exactly 26 .. 30 and loses nothing.
TEST(ShmDump, WrappedBufferSkipsTheSlotBeingOverwrittenAndCountsLost) {
  const RegionGuard g;
  const fs::path csv = out_dir("dump_wrapped") / "log.csv";
  auto created = Storage::create(g.name(), 10, false);
  ASSERT_TRUE(created.is_ok()) << created.reason();
  Recorder<kDof> rec(created.value().memory(), FullPolicy::kOverwriteOldest);
  record_range(rec, 1, 25);
  const auto first = dump_shm_log<kDof>(g.name(), csv, false);
  ASSERT_TRUE(first.is_ok()) << first.reason();
  EXPECT_EQ(first.value().rows, 9U);
  EXPECT_EQ(first.value().lost, 16U);
  EXPECT_EQ(csv_stamps(read_csv(csv)), range(17, 25));

  record_range(rec, 26, 30);
  const auto second = dump_shm_log<kDof>(g.name(), csv, false);
  ASSERT_TRUE(second.is_ok()) << second.reason();
  EXPECT_EQ(second.value().rows, 5U);
  EXPECT_EQ(second.value().lost, 0U);
  const CsvFile f = read_csv(csv);
  EXPECT_EQ(f.header_lines, 1);
  EXPECT_EQ(csv_stamps(f), range(17, 30));
  expect_rows_well_formed(f, true);
}

// A swap done but not yet exported: the retired buffer (1 .. 30) comes first, then the active one
// (31 .. 50; record 31 performed the swap).
TEST(ShmDump, RetiredBufferComesBeforeTheActiveOne) {
  const RegionGuard g;
  const fs::path csv = out_dir("dump_retired") / "log.csv";
  auto created = Storage::create(g.name(), 100, false);
  ASSERT_TRUE(created.is_ok()) << created.reason();
  Recorder<kDof> rec(created.value().memory(), FullPolicy::kOverwriteOldest);
  record_range(rec, 1, 30);
  ASSERT_TRUE(rec.request_swap());
  record_range(rec, 31, 50);
  ASSERT_TRUE(rec.swap_done());
  const auto r = dump_shm_log<kDof>(g.name(), csv, false);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().rows, 50U);
  EXPECT_EQ(csv_stamps(read_csv(csv)), range(1, 50));
}

// The controller's ~/dump_log path (swap, export retired(), release) followed by cobot_logdump:
// each record is exported exactly once. The controller exported 1 .. 30; the dump returns
// 31 .. 50 only.
TEST(ShmDump, RowsExportedByTheControllerAreNotDumpedAgain) {
  const RegionGuard g;
  const fs::path dir = out_dir("dump_after_controller");
  auto created = Storage::create(g.name(), 100, false);
  ASSERT_TRUE(created.is_ok()) << created.reason();
  Recorder<kDof> rec(created.value().memory(), FullPolicy::kOverwriteOldest);
  record_range(rec, 1, 30);
  ASSERT_TRUE(rec.request_swap());
  record_range(rec, 31, 50);
  ASSERT_TRUE(rec.swap_done());
  ASSERT_TRUE(export_csv<kDof>(dir / "run.csv", rec.retired(), true).is_ok());
  rec.release_retired();

  const auto r = dump_shm_log<kDof>(g.name(), dir / "dump.csv", false);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_EQ(r.value().rows, 20U);
  EXPECT_EQ(csv_stamps(read_csv(dir / "run.csv")), range(1, 30));
  EXPECT_EQ(csv_stamps(read_csv(dir / "dump.csv")), range(31, 50));
}

TEST(ShmDump, OtherDofIsAnError) {
  const RegionGuard g;
  ASSERT_TRUE(ShmStorage<2>::create(g.name(), 16, false).is_ok());
  const auto r = dump_shm_log<kDof>(g.name(), out_dir("dump_dof") / "log.csv", false);
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(lower(r.reason()), "dof")) << r.reason();
}

// --------------------------------------------------------------- recover_shm_log, one process

// "<log_path>.recovered-YYYYmmdd-HHMMSS.csv" next to log_path.
void expect_recovered_name(const fs::path& got, const fs::path& log_path) {
  EXPECT_EQ(got.parent_path(), log_path.parent_path()) << got;
  EXPECT_TRUE(is_stamped_name(got.filename().string(), "run.csv.recovered-", ".csv")) << got;
}

std::vector<fs::path> files_in(const fs::path& dir) {
  std::vector<fs::path> out;
  for (const auto& e : fs::directory_iterator(dir)) {
    out.push_back(e.path());
  }
  return out;
}

TEST(ShmRecover, NoRegionIsNothingToRecover) {
  const RegionGuard g;
  const fs::path dir = out_dir("recover_none");
  const auto r = recover_shm_log<kDof>(g.name(), dir / "run.csv");
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_FALSE(r.value().has_value());
  EXPECT_TRUE(files_in(dir).empty());
}

TEST(ShmRecover, UnexportedRowsAreRecoveredOnceAndTheRegionRemoved) {
  const RegionGuard g;
  const fs::path dir = out_dir("recover_rows");
  const fs::path log_path = dir / "run.csv";
  {
    auto created = Storage::create(g.name(), 100, false);
    ASSERT_TRUE(created.is_ok()) << created.reason();
    Recorder<kDof> rec(created.value().memory(), FullPolicy::kOverwriteOldest);
    record_range(rec, 1, 60);
  }
  const auto r = recover_shm_log<kDof>(g.name(), log_path);
  ASSERT_TRUE(r.is_ok()) << r.reason();
  ASSERT_TRUE(r.value().has_value());
  const fs::path recovered = *r.value();
  expect_recovered_name(recovered, log_path);
  const CsvFile f = read_csv(recovered);
  EXPECT_EQ(f.header_lines, 1);
  EXPECT_EQ(csv_stamps(f), range(1, 60));
  expect_rows_well_formed(f, true);
  EXPECT_FALSE(Storage::exists(g.name()));
  EXPECT_FALSE(fs::exists(log_path)) << "recovery writes its own file, not log_path";

  const auto again = recover_shm_log<kDof>(g.name(), log_path);
  ASSERT_TRUE(again.is_ok()) << again.reason();
  EXPECT_FALSE(again.value().has_value());
}

TEST(ShmRecover, FullyExportedRegionIsRemovedWithoutAFile) {
  const RegionGuard g;
  const fs::path dir = out_dir("recover_exported");
  {
    auto created = Storage::create(g.name(), 100, false);
    ASSERT_TRUE(created.is_ok()) << created.reason();
    Recorder<kDof> rec(created.value().memory(), FullPolicy::kOverwriteOldest);
    record_range(rec, 1, 10);
  }
  ASSERT_TRUE(dump_shm_log<kDof>(g.name(), dir / "dump.csv", false).is_ok());
  const auto r = recover_shm_log<kDof>(g.name(), dir / "run.csv");
  ASSERT_TRUE(r.is_ok()) << r.reason();
  EXPECT_FALSE(r.value().has_value());
  EXPECT_FALSE(Storage::exists(g.name()));
  EXPECT_EQ(files_in(dir), std::vector<fs::path>{dir / "dump.csv"});
}

TEST(ShmRecover, LayoutMismatchIsAnError) {
  const RegionGuard g;
  ASSERT_TRUE(ShmStorage<2>::create(g.name(), 16, false).is_ok());
  const auto r = recover_shm_log<kDof>(g.name(), out_dir("recover_dof") / "run.csv");
  ASSERT_FALSE(r.is_ok());
  EXPECT_TRUE(contains(lower(r.reason()), "dof")) << r.reason();
}

// --------------------------------------------------------------- record() cost, reported

// p50 / p95 of one record() call on heap and on shm memory over 100 000 records (two buffers of
// 4096, kOverwriteOldest; region about 2 * 4096 * (sizeof(LogRecord<7>) + 8) < 8 MB). Reported
// for the PR body, not gated.
TEST(ShmRecordCost, ReportsP50AndP95HeapVersusShm) {
  constexpr std::size_t kCapacity = 4096;
  constexpr int kRecords = 100000;
  const Rec a = make_record(1);
  const auto measure = [&](RecorderMemory<kDof> memory) {
    Recorder<kDof> rec(memory, FullPolicy::kOverwriteOldest);
    std::vector<double> ns(kRecords);
    for (int k = 0; k < kRecords; ++k) {
      const auto t0 = std::chrono::steady_clock::now();
      rec.record(a);
      const auto t1 = std::chrono::steady_clock::now();
      ns[static_cast<std::size_t>(k)] = std::chrono::duration<double, std::nano>(t1 - t0).count();
    }
    EXPECT_EQ(rec.recorded(), static_cast<std::uint64_t>(kRecords));
    std::sort(ns.begin(), ns.end());
    return std::pair<double, double>{ns[ns.size() / 2], ns[ns.size() * 95 / 100]};
  };
  HeapStorage<kDof> heap(kCapacity, 2);
  const auto h = measure(heap.memory());
  const RegionGuard g;
  auto shm = Storage::create(g.name(), kCapacity, false);
  ASSERT_TRUE(shm.is_ok()) << shm.reason();
  const auto s = measure(shm.value().memory());
  std::printf(
      "record() over %d records: heap p50 %.0f ns p95 %.0f ns; shm p50 %.0f ns p95 %.0f ns\n",
      kRecords,
      h.first,
      h.second,
      s.first,
      s.second);
  RecordProperty("heap_p50_ns", std::to_string(h.first));
  RecordProperty("heap_p95_ns", std::to_string(h.second));
  RecordProperty("shm_p50_ns", std::to_string(s.first));
  RecordProperty("shm_p95_ns", std::to_string(s.second));
}

}  // namespace
}  // namespace cobot::pipeline::shm_test
