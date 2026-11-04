// RunStore (feature studio-core, ADR-021, cobot/studio/run_store.hpp): owned by the GUI thread;
// add assigns increasing ids, runs() keeps insertion order, memory_bytes counts the records only.
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "cobot/pipeline/log_record.hpp"
#include "cobot/studio/run_store.hpp"

namespace cobot::studio {
namespace {

RunEntry entry(const std::string& label, std::size_t rows) {
  RunEntry e{};
  e.label = label;
  e.kind = RunKind::kBench;
  if (rows > 0) {
    auto data = std::make_shared<bench::RunData>();
    data->records.resize(rows);
    e.data = data;
  }
  return e;
}

TEST(RunStore, AddAssignsIncreasingIdsAndKeepsOrder) {
  RunStore store;
  const std::uint64_t a = store.add(entry("a", 0));
  const std::uint64_t b = store.add(entry("b", 0));
  const std::uint64_t c = store.add(entry("c", 0));
  EXPECT_LT(a, b);
  EXPECT_LT(b, c);
  ASSERT_EQ(store.runs().size(), 3U);
  EXPECT_EQ(store.runs()[0].label, "a");
  EXPECT_EQ(store.runs()[2].label, "c");
  EXPECT_EQ(store.runs()[1].id, b);
  ASSERT_NE(store.find(b), nullptr);
  EXPECT_EQ(store.find(b)->label, "b");
}

TEST(RunStore, RemoveAndClear) {
  RunStore store;
  const std::uint64_t a = store.add(entry("a", 0));
  const std::uint64_t b = store.add(entry("b", 0));
  EXPECT_TRUE(store.remove(a));
  EXPECT_FALSE(store.remove(a));
  EXPECT_EQ(store.find(a), nullptr);
  ASSERT_EQ(store.runs().size(), 1U);
  EXPECT_EQ(store.runs()[0].id, b);
  const std::uint64_t c = store.add(entry("c", 0));
  EXPECT_GT(c, b) << "ids are never reused";
  store.clear();
  EXPECT_TRUE(store.runs().empty());
  EXPECT_EQ(store.find(c), nullptr);
}

TEST(RunStore, MemoryCountsTheRecords) {
  RunStore store;
  static_cast<void>(store.add(entry("summary only", 0)));
  EXPECT_EQ(store.memory_bytes(), 0U);
  const std::uint64_t id = store.add(entry("records", 1000));
  constexpr std::size_t kRecords = 1000;
  EXPECT_GE(store.memory_bytes(), kRecords * sizeof(pipeline::LogRecord<7>));
  EXPECT_LT(store.memory_bytes(), 2 * kRecords * sizeof(pipeline::LogRecord<7>));
  static_cast<void>(store.remove(id));
  EXPECT_EQ(store.memory_bytes(), 0U);
}

}  // namespace
}  // namespace cobot::studio
